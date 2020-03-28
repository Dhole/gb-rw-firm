#![feature(trait_alias)]
#![no_main]
#![feature(asm)]
#![no_std]

mod gpio;
mod macros;
mod serial;

// Halt on panic
extern crate embedded_hal;
#[allow(unused_extern_crates)] // NOTE(allow) bug rust-lang/rust#53964
extern crate panic_halt; // panic handler

#[macro_use]
extern crate urpc;

// use core::fmt::Write;
// use core::fmt::Write;
use cortex_m::{self, asm::delay};
use cortex_m_rt::entry;
use hal::{nb::block, prelude::*, stm32};
use stm32f4xx_hal as hal;

use serde::Deserialize;
use urpc::{
    consts,
    server::{self, Request},
    OptBufNo, OptBufYes,
};

// use embedded_hal::fmt;

use crate::gpio::{GpioPortA, GpioPortB, GpioPortC};
use crate::serial::Read as _;
use crate::serial::Serial;
use crate::serial::Write as _;

#[derive(Debug, Deserialize)]
enum ReqMode {
    GB,
    GBA,
}

server_requests! {
    ServerRequest;
    (0, Ping([u8; 4], OptBufNo, [u8; 4], OptBufNo)),
    (1, SendBytes((), OptBufYes, (), OptBufNo)),
    (2, Add((u8, u8), OptBufNo, u8, OptBufNo)),
    (3, RecvBytes((), OptBufNo, (), OptBufYes)),
    (4, SendRecv((), OptBufYes, (), OptBufYes)),
    (5, GBRead((u16, u16), OptBufNo, (), OptBufYes)),
    (6, Mode(ReqMode, OptBufNo, bool, OptBufNo)),
    (7, GBWriteWord((u16, u8), OptBufNo, (), OptBufNo)),
    (8, GBWrite(u16, OptBufYes, (), OptBufNo)),
    // (9, GBFlashErase((), OptBufNo, (), OptBufNo)),
    (10, GBFlashWrite(u16, OptBufYes, Option<(u16, u8)>, OptBufNo))
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Set up the system clock. We want to run at 48MHz for this one.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(96.mhz()).freeze();

    // Set up the LED. On the Nucleo-411RE it's connected to pin PA5.
    let gpioa = dp.GPIOA.split();
    let mut led = gpioa.pa5.into_push_pull_output();

    // let gpioh = dp.GPIOH.split();

    // USART2
    // Configure pa2 as a push_pull output, this will be the tx pin
    let tx = gpioa.pa2.into_alternate_af7();
    // Take ownership over pa3
    let rx = gpioa.pa3.into_alternate_af7();

    // Set up the usart device. Taks ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.

    let (tx, rx) = hal::serial::Serial::usart2(
        dp.USART2,
        (tx, rx),
        // hal::serial::config::Config::default().baudrate(921600.bps()),
        // hal::serial::config::Config::default().baudrate(1152000.bps()),
        hal::serial::config::Config::default().baudrate(1500000.bps()),
        clocks,
    )
    .unwrap()
    .split();

    let mut serial = Serial::new(rx, tx);

    serial.write_all(b"\nHELLO\n").ok();

    let mut mode = Mode::GB(GB {
        cart: GBCart::take(Resources {
            gpioc: dp.GPIOC.split(),
            gpiob: dp.GPIOB.split(),
            gpioa_0: gpioa.pa0,
            gpioa_1: gpioa.pa1,
            gpioa_6: gpioa.pa6,
            gpioa_7: gpioa.pa7,
            gpioa_8: gpioa.pa8,
            gpioa_9: gpioa.pa9,
            gpioa_10: gpioa.pa10,
        }),
    });

    // let gpioa = GpioPortA::take(dp.GPIOA.split(), 1 << 5);
    // let gpioa = gpioa.into_push_pull_output();
    // gpioa.write(0xffff).ok();

    // Create a delay abstraction based on SysTick
    // let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    // use stm32::GPIOA;
    // use stm32f4::stm32f411 as stm32;

    // Set up the LED. On the Nucleo-411RE it's connected to pin PA5.
    // let gpioa = dp.GPIOA.split();
    // let mut led = gpioa.pa5.into_push_pull_output();

    // let i = 5;
    // let offset = 2 * i;

    // unsafe {
    //     &(*GPIOA::ptr())
    //         .pupdr
    //         .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
    //     &(*GPIOA::ptr())
    //         .otyper
    //         .modify(|r, w| w.bits(r.bits() & !(0b1 << i)));
    //     &(*GPIOA::ptr())
    //         .moder
    //         .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)))
    // };

    // On for 1s, off for 1s.
    // led.set_high().unwrap();
    // gpioa.write(0xffff).ok();
    // // unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (5 + 16))) };
    // // unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << i)) };
    // delay.delay_ms(500_u32);
    // // led.set_low().unwrap();
    // gpioa.write(0x0000).ok();
    // // unsafe { (*GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (i + 16))) };
    // delay.delay_ms(500_u32);

    // loop {
    //     // On for 1s, off for 1s.
    //     led.set_high().unwrap();
    //     delay.delay_ms(500_u32);
    //     led.set_low().unwrap();
    //     delay.delay_ms(500_u32);
    // }

    let mut rpc_server = server::RpcServer::new(0x4100 as u16);

    let mut rx_buf = [0; 0x4100];
    let mut tx_buf = [0; 0x4100];

    let mut rx_len = consts::REQ_HEADER_LEN;

    led.toggle().unwrap();
    loop {
        serial.read_exact(&mut rx_buf[..rx_len]).ok();
        rx_len = match ServerRequest::from_rpc(&mut rpc_server, &rx_buf[..rx_len]).unwrap() {
            server::ParseResult::NeedBytes(n) => n,
            server::ParseResult::Request(req) => {
                let tx_len = match req {
                    ServerRequest::Ping(req) => {
                        let res = ping(req.body);
                        req.reply(res, &mut tx_buf).unwrap()
                    }
                    ServerRequest::SendBytes((req, buf)) => {
                        let res = send_bytes(req.body);
                        req.reply(res, &mut tx_buf).unwrap()
                    }
                    ServerRequest::Add(req) => {
                        let res = add(req.body);
                        req.reply(res, &mut tx_buf).unwrap()
                    }
                    ServerRequest::RecvBytes(req) => {
                        let rep_buf_len = {
                            let rep_buf = req.get_opt_buf(&mut tx_buf);
                            let n = 8;
                            for i in 0..n {
                                rep_buf[i] = (i * 2) as u8;
                            }
                            n
                        };
                        req.reply((), rep_buf_len as u16, &mut tx_buf).unwrap()
                    }
                    ServerRequest::SendRecv((req, req_buf)) => {
                        let rep_buf_len = {
                            let rep_buf = req.get_opt_buf(&mut tx_buf);
                            let n = core::cmp::min(req_buf.len(), rep_buf.len());
                            for i in 0..n {
                                rep_buf[i] = req_buf[i] * 2;
                            }
                            n
                        };
                        req.reply((), rep_buf_len as u16, &mut tx_buf).unwrap()
                    }
                    ServerRequest::GBRead(req) => {
                        if let Mode::GB(gb) = &mut mode {
                            let (addr_start, size) = req.body;
                            let rep_buf_len = {
                                let rep_buf = req.get_opt_buf(&mut tx_buf);
                                gb.read_bytes(addr_start, &mut rep_buf[..size as usize]);
                                size
                            };
                            req.reply((), rep_buf_len as u16, &mut tx_buf).unwrap()
                        } else {
                            req.reply((), 0, &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::Mode(req) => {
                        let rs = match mode {
                            Mode::GB(gb) => gb.cart.release(),
                            Mode::GBA(gba) => gba.cart.release(),
                        };
                        match req.body {
                            ReqMode::GB => {
                                let gb = GB {
                                    cart: GBCart::take(rs),
                                };
                                mode = Mode::GB(gb);
                            }
                            ReqMode::GBA => {
                                let gba = GBA {
                                    cart: GBACart::take(rs),
                                };
                                mode = Mode::GBA(gba);
                            }
                        }
                        req.reply(false, &mut tx_buf).unwrap()
                    }
                    ServerRequest::GBWriteWord(req) => {
                        if let Mode::GB(gb) = mode {
                            let (addr, word) = req.body;
                            let mut gb = gb.to_write();
                            gb.write_byte(addr, word);
                            mode = Mode::GB(gb.to_read());
                            req.reply((), &mut tx_buf).unwrap()
                        } else {
                            req.reply((), &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBWrite((req, buf)) => {
                        if let Mode::GB(gb) = mode {
                            let addr = req.body;
                            let mut gb = gb.to_write();
                            gb.write(addr, buf);
                            mode = Mode::GB(gb.to_read());
                            req.reply((), &mut tx_buf).unwrap()
                        } else {
                            req.reply((), &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBFlashWrite((req, buf)) => {
                        if let Mode::GB(gb) = mode {
                            let addr = req.body;
                            let mut gb = gb.to_write();
                            let (mut gb, fail) = gb.flash_write(addr, buf);
                            mode = Mode::GB(gb.to_read());
                            req.reply(fail, &mut tx_buf).unwrap()
                        } else {
                            req.reply(None, &mut tx_buf).unwrap()
                        }
                    }
                };
                serial.write_all(&tx_buf[..tx_len]).ok();
                led.toggle().unwrap();
                consts::REQ_HEADER_LEN
            }
        };
    }
}

fn ping(body: [u8; 4]) -> [u8; 4] {
    body
}

fn send_bytes(body: ()) -> () {
    ()
}

fn send_recv(body: ()) -> () {
    ()
}

fn add(body: (u8, u8)) -> u8 {
    let (x, y) = body;
    x + y
}

use hal::gpio::{gpioa, gpiob, gpioc};
use hal::gpio::{Floating, Input, OpenDrain, Output, PullDown, PushPull};

enum Mode {
    GB(GB<GBRead>),
    GBA(GBA<GBARead>),
}

struct Resources {
    gpioc: gpioc::Parts,
    gpiob: gpiob::Parts,
    gpioa_0: gpioa::PA0<Input<Floating>>,
    gpioa_1: gpioa::PA1<Input<Floating>>,
    gpioa_6: gpioa::PA6<Input<Floating>>,
    gpioa_7: gpioa::PA7<Input<Floating>>,
    gpioa_8: gpioa::PA8<Input<Floating>>,
    gpioa_9: gpioa::PA9<Input<Floating>>,
    gpioa_10: gpioa::PA10<Input<Floating>>,
}

trait GBData {}

struct GBRead(GpioPortB<Input<PullDown>>);

impl GBRead {
    fn take(gpiob: gpiob::Parts) -> Self {
        Self(GpioPortB::take(gpiob, 0x00ff).into_pull_down_input())
    }
    fn release(self) -> gpiob::Parts {
        self.0.release()
    }
    fn read(&mut self) -> u8 {
        (self.0.read().unwrap() & 0x00ff) as u8
    }
    // fn to_gb_write(self) -> GBWrite {
    //     GBWrite {
    //         data: self.data.into_push_pull_output(),
    //     }
    // }
}

impl GBData for GBRead {}

struct GBWrite(GpioPortB<Output<PushPull>>);

impl GBWrite {
    fn write(&mut self, v: u8) {
        self.0.write(v as u16).unwrap()
    }
    // fn to_gb_read(self) -> GBRead {
    //     GBRead {
    //         data: self.data.into_pull_down_input(),
    //     }
    // }
}

impl GBData for GBWrite {}

struct GBCart<D: GBData> {
    addr_0_13: GpioPortC<Output<PushPull>>,
    addr_14: gpioa::PA0<Output<PushPull>>,
    addr_15: gpioa::PA1<Output<PushPull>>,
    // addr_0_13: GpioPortC<Output<OpenDrain>>,
    // addr_14: gpioh::PH0<Output<OpenDrain>>,
    // addr_15: gpioh::PH1<Output<OpenDrain>>,
    data: D,
    cs: gpioa::PA8<Output<PushPull>>,
    rd: gpioa::PA7<Output<PushPull>>,
    wr: gpioa::PA6<Output<PushPull>>,
    clk: gpioa::PA10<Output<PushPull>>,
    reset: gpioa::PA9<Output<PushPull>>,
}

impl GBCart<GBRead> {
    fn take(rs: Resources) -> Self {
        let mut s = Self {
            addr_0_13: GpioPortC::take(rs.gpioc, 0b0011_1111_1111_1111).into_push_pull_output(),
            addr_14: rs.gpioa_0.into_push_pull_output(),
            addr_15: rs.gpioa_1.into_push_pull_output(),
            // addr_0_13: GpioPortC::take(rs.gpioc, 0b0011_1111_1111_1111).into_open_drain_output(),
            // addr_14: rs.gpioh_0.into_open_drain_output(),
            // addr_15: rs.gpioh_1.into_open_drain_output(),
            data: GBRead::take(rs.gpiob),
            cs: rs.gpioa_8.into_push_pull_output(),
            rd: rs.gpioa_7.into_push_pull_output(),
            wr: rs.gpioa_6.into_push_pull_output(),
            clk: rs.gpioa_10.into_push_pull_output(),
            reset: rs.gpioa_9.into_push_pull_output(),
        };
        s.set_pin(GBPin::WR, false);
        s.set_pin(GBPin::RD, false);
        s.set_pin(GBPin::CS, false);
        s.set_pin(GBPin::RESET, false);
        s
    }
    fn release(self) -> Resources {
        Resources {
            gpioc: self.addr_0_13.release(),
            gpiob: self.data.release(),
            gpioa_0: self.addr_14.into_floating_input(),
            gpioa_1: self.addr_15.into_floating_input(),
            gpioa_6: self.wr.into_floating_input(),
            gpioa_7: self.rd.into_floating_input(),
            gpioa_8: self.cs.into_floating_input(),
            gpioa_9: self.reset.into_floating_input(),
            gpioa_10: self.clk.into_floating_input(),
        }
    }
    fn data(&mut self) -> u8 {
        self.data.read()
    }
    fn to_write(self) -> GBCart<GBWrite> {
        GBCart::<GBWrite> {
            data: GBWrite(self.data.0.into_push_pull_output()),
            addr_0_13: self.addr_0_13,
            addr_14: self.addr_14,
            addr_15: self.addr_15,
            cs: self.cs,
            rd: self.rd,
            wr: self.wr,
            clk: self.clk,
            reset: self.reset,
        }
    }
}

impl GBCart<GBWrite> {
    fn set_data(&mut self, v: u8) {
        self.data.write(v)
    }
    fn to_read(self) -> GBCart<GBRead> {
        GBCart::<GBRead> {
            data: GBRead(self.data.0.into_pull_down_input()),
            addr_0_13: self.addr_0_13,
            addr_14: self.addr_14,
            addr_15: self.addr_15,
            cs: self.cs,
            rd: self.rd,
            wr: self.wr,
            clk: self.clk,
            reset: self.reset,
        }
    }
}

impl<D: GBData> GBCart<D> {
    fn set_addr(&mut self, addr: u16) {
        self.addr_0_13.write(addr).unwrap();
        if addr & (1 << 14) != 0 {
            self.addr_14.set_high()
        } else {
            self.addr_14.set_low()
        }
        .unwrap();
        if addr & (1 << 15) != 0 {
            self.addr_15.set_high()
        } else {
            self.addr_15.set_low()
        }
        .unwrap();
    }
    #[rustfmt::skip]
    fn set_pin(&mut self, pin: GBPin, v: bool) {
        match pin {
            GBPin::CS => { if v { self.cs.set_low() } else { self.cs.set_high() } }
            GBPin::RD => { if v { self.rd.set_low() } else { self.rd.set_high() } }
            GBPin::WR => { if v { self.wr.set_low() } else { self.wr.set_high() } }
            GBPin::CLK => { if v { self.clk.set_low() } else { self.clk.set_high() } }
            GBPin::RESET => { if v { self.reset.set_low() } else { self.reset.set_high() } }
        }.unwrap()
    }
}

enum GBPin {
    CS,
    RD,
    WR,
    CLK,
    RESET,
}

struct GB<D: GBData> {
    cart: GBCart<D>,
}

impl GB<GBRead> {
    fn read_byte(&mut self, addr: u16) -> u8 {
        self.cart.set_addr(addr);
        self.cart.set_pin(GBPin::CS, true);
        delay(5);
        self.cart.set_pin(GBPin::RD, true);
        // wait ~200ns
        delay(20);
        let byte = self.cart.data();

        self.cart.set_pin(GBPin::RD, false);
        self.cart.set_pin(GBPin::CS, false);
        delay(10);

        return byte;
    }
    fn read_bytes(&mut self, addr: u16, buf: &mut [u8]) {
        for i in 0..buf.len() {
            buf[i] = self.read_byte(addr + i as u16)
        }
    }
    fn to_write(self) -> GB<GBWrite> {
        GB {
            cart: self.cart.to_write(),
        }
    }
}

impl GB<GBWrite> {
    fn write_byte(&mut self, addr: u16, byte: u8) {
        // Set address
        self.cart.set_addr(addr);
        // wait some nanoseconds
        delay(5);

        self.cart.set_pin(GBPin::CS, true);
        delay(5);
        self.cart.set_pin(GBPin::WR, true);

        // set data
        // gpio_data_setup_output();
        self.cart.set_data(byte);
        // wait ~200ns
        delay(20);

        self.cart.set_pin(GBPin::WR, false);
        // delay(5);
        // gpio_data_setup_input();
        self.cart.set_pin(GBPin::CS, false);
        delay(25);
    }
    fn write(&mut self, addr: u16, bytes: &[u8]) {
        for (i, byte) in bytes.iter().enumerate() {
            self.write_byte(addr + i as u16, *byte);
        }
    }
    // Returns Self, success, last byte read
    fn flash_write_byte(self, addr: u16, byte: u8) -> (Self, bool, u8) {
        let mut gb = self;
        gb.write_byte(0x0AAA, 0xA9);
        gb.write_byte(0x0555, 0x56);
        gb.write_byte(0x0AAA, 0xA0);
        gb.write_byte(addr, byte);
        let mut gb = gb.to_read();
        let (mut ok, mut b1) = (false, 0);
        // Poll Q6 to detect completion of the Flash Auto Program Algorithm
        for _ in 0..50 {
            delay(10);
            let b0 = gb.read_byte(addr);
            delay(5);
            b1 = gb.read_byte(addr);
            // Q6 toggles during Flash Auto Program Algorithm
            // Q5 = 1 indicates failure
            if b0 & (1 << 6) == b1 & (1 << 6) {
                ok = true;
                break;
            } else if b0 & (1 << 5) != 0 {
                break;
            }
        }
        (gb.to_write(), ok, b1)
    }
    fn flash_write(self, addr: u16, bytes: &[u8]) -> (Self, Option<(u16, u8)>) {
        let mut gb = self;
        let mut fail = None;
        const RETRIES: usize = 4;
        for (i, byte) in bytes.iter().enumerate() {
            for _ in 0..RETRIES {
                let (_gb, ok, b) = gb.flash_write_byte(addr + i as u16, *byte);
                gb = _gb;
                fail = Some((addr + i as u16, b));
                if !ok {
                    break;
                }
                // If flash_write_byte reported success but last byte read is different than
                // exepcted, repeat.
                if b == *byte {
                    fail = None;
                    break;
                }
            }
            if let Some(_) = fail {
                break;
            }
        }
        (gb, fail)
    }
    fn to_read(self) -> GB<GBRead> {
        GB {
            cart: self.cart.to_read(),
        }
    }
}

trait GBAData {}

struct GBA<D: GBAData> {
    cart: GBACart<D>,
}

struct GBACart<D: GBAData> {
    _panthom: core::marker::PhantomData<D>,
}

struct GBARead {}

impl GBAData for GBARead {}

impl GBACart<GBARead> {
    fn take(rs: Resources) -> Self {
        Self {
            _panthom: core::marker::PhantomData::<GBARead>,
        }
    }
    fn release(self) -> Resources {
        unimplemented!();
    }
}
