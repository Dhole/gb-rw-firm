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

use cortex_m::{self, asm::delay};
use cortex_m_rt::entry;
use hal::{prelude::*, stm32};
use stm32f4xx_hal as hal;

use serde::Deserialize;
use urpc::{
    consts,
    server::{self, Request},
    OptBufNo, OptBufYes,
};

use crate::gpio::{GpioPortB, GpioPortC};
use crate::serial::Read as _;
use crate::serial::Serial;
use crate::serial::Write as _;

#[derive(Debug, Deserialize)]
enum ReqMode {
    GB,
    GBA,
}

#[derive(Debug, Clone, Copy, Serialize)]
struct GBStats {
    flash_write_retries: u64,
    flash_write_errors: u64,
}

impl GBStats {
    fn new() -> Self {
        Self {
            flash_write_retries: 0,
            flash_write_errors: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub enum FlashType {
    F3 = 3,
}

server_requests! {
    ServerRequest;
    (0, ping, Ping([u8; 4], OptBufNo, [u8; 4], OptBufNo)),
    (1, mode, Mode(ReqMode, OptBufNo, bool, OptBufNo)),
    (2, gb_read, GBRead((u16, u16), OptBufNo, (), OptBufYes)),
    (3, gb_write_word, GBWriteWord((u16, u8), OptBufNo, (), OptBufNo)),
    (4, gb_write, GBWrite(u16, OptBufYes, (), OptBufNo)),
    // (5, gb_flash_erase, GBFlashErase((), OptBufNo, (), OptBufNo)),
    (6, gb_flash_write, GBFlashWrite(u16, OptBufYes, Option<(u16, u8)>, OptBufNo)),
    (7, gb_flash_erase_sector, GBFlashEraseSector(u16, OptBufNo, bool, OptBufNo)),
    (8, gb_flash_info, GBFlashInfo((), OptBufNo, (u8, u8), OptBufNo)),
    (9, gb_get_stats, GBGetStats((), OptBufNo, Option<GBStats>, OptBufNo)),
    (50, gba_read, GBARead((u32, u16), OptBufNo, (), OptBufYes)),
    (51, gba_write_word, GBAWriteWord((u32, u16), OptBufNo, (), OptBufNo)),
    (52, gba_flash_write, GBAFlashWrite((FlashType, u32), OptBufYes, Option<u32>, OptBufNo)),
    (53, gba_flash_unlock_sector, GBAFlashUnlockSector((FlashType, u32), OptBufNo, bool, OptBufNo)),
    (54, gba_flash_erase_sector, GBAFlashEraseSector((FlashType, u32), OptBufNo, bool, OptBufNo)),
    (255, test1, Test((), OptBufNo, Option<[(u16, u16); 8]>, OptBufNo)) // 1
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    // let cp = cortex_m::peripheral::Peripherals::take().unwrap();

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

    let mut mode = Mode::GB(GB::new(Resources {
        gpioc: dp.GPIOC.split(),
        gpiob: dp.GPIOB.split(),
        gpioa_0: gpioa.pa0,
        gpioa_1: gpioa.pa1,
        gpioa_6: gpioa.pa6,
        gpioa_7: gpioa.pa7,
        gpioa_8: gpioa.pa8,
        gpioa_9: gpioa.pa9,
        gpioa_10: gpioa.pa10,
    }));

    let mut rpc_server = server::RpcServer::new(0x8100 as u16);

    let mut rx_buf = [0; 0x8100];
    let mut tx_buf = [0; 0x8100];

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
                    ServerRequest::GBRead(req) => {
                        if let Mode::GB(gb) = &mut mode {
                            let (addr_start, size) = req.body;
                            let rep_buf_len = {
                                let rep_buf = req.get_opt_buf(&mut tx_buf);
                                gb.read(addr_start, &mut rep_buf[..size as usize]);
                                size
                            };
                            req.reply((), rep_buf_len as u16, &mut tx_buf).unwrap()
                        } else {
                            req.reply((), 0, &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::Mode(req) => {
                        let rs = match mode {
                            Mode::GB(gb) => gb.cart.unwrap().release(),
                            Mode::GBA(gba) => gba.cart.release(),
                        };
                        match req.body {
                            ReqMode::GB => {
                                let gb = GB::new(rs);
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
                        if let Mode::GB(ref mut gb) = mode {
                            let (addr, word) = req.body;
                            gb.write_byte(addr, word);
                            req.reply((), &mut tx_buf).unwrap()
                        } else {
                            req.reply((), &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBWrite((req, buf)) => {
                        if let Mode::GB(ref mut gb) = mode {
                            let addr = req.body;
                            gb.write(addr, buf);
                            req.reply((), &mut tx_buf).unwrap()
                        } else {
                            req.reply((), &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBFlashWrite((req, buf)) => {
                        if let Mode::GB(ref mut gb) = mode {
                            let addr = req.body;
                            let fail = gb.flash_write(addr, buf);
                            req.reply(fail, &mut tx_buf).unwrap()
                        } else {
                            req.reply(None, &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBFlashEraseSector(req) => {
                        if let Mode::GB(ref mut gb) = mode {
                            let addr = req.body;
                            let ok = gb.flash_erase_sector(addr);
                            req.reply(ok, &mut tx_buf).unwrap()
                        } else {
                            req.reply(false, &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBFlashInfo(req) => {
                        if let Mode::GB(ref mut gb) = mode {
                            let info = gb.flash_info();
                            req.reply(info, &mut tx_buf).unwrap()
                        } else {
                            req.reply((0, 0), &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBARead(req) => {
                        if let Mode::GBA(gba) = &mut mode {
                            let (addr_start, size) = req.body;
                            let rep_buf_len = {
                                let rep_buf = req.get_opt_buf(&mut tx_buf);
                                gba.read(addr_start, &mut rep_buf[..size as usize]);
                                size
                            };
                            req.reply((), rep_buf_len as u16, &mut tx_buf).unwrap()
                        } else {
                            req.reply((), 0, &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBGetStats(req) => {
                        if let Mode::GB(gb) = &mut mode {
                            req.reply(Some(gb.stats()), &mut tx_buf).unwrap()
                        } else {
                            req.reply(None, &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBAWriteWord(req) => {
                        if let Mode::GBA(gba) = &mut mode {
                            let (addr, word) = req.body;
                            gba.write_word(addr, word);
                            req.reply((), &mut tx_buf).unwrap()
                        } else {
                            req.reply((), &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBAFlashWrite((req, buf)) => {
                        if let Mode::GBA(gba) = &mut mode {
                            let (flash_type, addr) = req.body;
                            let fail = gba.flash_write(flash_type, addr, &buf);
                            req.reply(fail, &mut tx_buf).unwrap()
                        } else {
                            req.reply(None, &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBAFlashUnlockSector(req) => {
                        if let Mode::GBA(gba) = &mut mode {
                            let (flash_type, sector) = req.body;
                            let ok = gba.flash_unlock_sector(flash_type, sector);
                            req.reply(ok, &mut tx_buf).unwrap()
                        } else {
                            req.reply(false, &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::GBAFlashEraseSector(req) => {
                        if let Mode::GBA(gba) = &mut mode {
                            let (flash_type, sector) = req.body;
                            let ok = gba.flash_erase_sector(flash_type, sector);
                            req.reply(ok, &mut tx_buf).unwrap()
                        } else {
                            req.reply(false, &mut tx_buf).unwrap()
                        }
                    }
                    ServerRequest::Test(req) => {
                        if let Mode::GBA(gba) = &mut mode {
                            let result = gba.test1();
                            req.reply(Some(result), &mut tx_buf).unwrap()
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

use hal::gpio::{gpioa, gpiob, gpioc};
use hal::gpio::{Floating, Input, Output, PullDown, PushPull};

enum Mode {
    GB(GB),
    GBA(GBA),
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
}

impl GBData for GBRead {}

struct GBWrite(GpioPortB<Output<PushPull>>);

impl GBWrite {
    fn write(&mut self, v: u8) {
        self.0.write(v as u16).unwrap()
    }
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
            GBPin::_CLK => { if v { self.clk.set_low() } else { self.clk.set_high() } }
            GBPin::RESET => { if v { self.reset.set_low() } else { self.reset.set_high() } }
        }.unwrap()
    }
}

enum GBPin {
    CS,
    RD,
    WR,
    _CLK,
    RESET,
}

struct GB {
    cart: Option<GBCart<GBRead>>,
    stats: GBStats,
}

impl GB {
    fn new(rs: Resources) -> Self {
        Self {
            cart: Some(GBCart::take(rs)),
            stats: GBStats::new(),
        }
    }
    fn stats(&self) -> GBStats {
        self.stats
    }
    fn read_byte(&mut self, addr: u16) -> u8 {
        let cart = self.cart.as_mut().unwrap();
        cart.set_addr(addr);
        cart.set_pin(GBPin::CS, true);
        delay(5);
        cart.set_pin(GBPin::RD, true);
        // wait ~200ns
        delay(20);
        let byte = cart.data();

        cart.set_pin(GBPin::RD, false);
        cart.set_pin(GBPin::CS, false);
        delay(10);

        return byte;
    }
    fn read(&mut self, addr: u16, buf: &mut [u8]) {
        for i in 0..buf.len() {
            buf[i] = self.read_byte(addr + i as u16)
        }
    }

    fn write_byte(&mut self, addr: u16, byte: u8) {
        let mut cart = self.cart.take().unwrap().to_write();
        // Set address
        cart.set_addr(addr);
        // wait some nanoseconds
        delay(5);

        cart.set_pin(GBPin::CS, true);
        delay(5);
        cart.set_pin(GBPin::WR, true);

        // set data
        // gpio_data_setup_output();
        cart.set_data(byte);
        // wait ~200ns
        delay(20);

        cart.set_pin(GBPin::WR, false);
        // delay(5);
        // gpio_data_setup_input();
        cart.set_pin(GBPin::CS, false);
        delay(10);
        self.cart = Some(cart.to_read());
    }
    fn write(&mut self, addr: u16, bytes: &[u8]) {
        for (i, byte) in bytes.iter().enumerate() {
            self.write_byte(addr + i as u16, *byte);
        }
    }
    // Returns Self, success, last byte read
    fn flash_write_byte(&mut self, addr: u16, byte: u8) -> (bool, u8) {
        self.write_byte(0x0AAA, 0xA9);
        self.write_byte(0x0555, 0x56);
        self.write_byte(0x0AAA, 0xA0);
        self.write_byte(addr, byte);
        let (mut ok, mut b1) = (false, 0);
        // Poll Q6 to detect completion of the Flash Auto Program Algorithm
        for _ in 0..50 {
            delay(10);
            let b0 = self.read_byte(addr);
            delay(5);
            b1 = self.read_byte(addr);
            // Q6 toggles during Flash Auto Program Algorithm
            // Q5 = 1 indicates failure
            if b0 & (1 << 6) == b1 & (1 << 6) {
                self.write_byte(0x0000, 0xFF); // reset
                b1 = self.read_byte(addr);
                ok = true;
                break;
            } else if b0 & (1 << 5) != 0 {
                self.stats.flash_write_errors += 1;
                break;
            }
        }
        (ok, b1)
    }
    fn flash_erase_sector(&mut self, addr: u16) -> bool {
        self.write_byte(0x0AAA, 0xA9);
        self.write_byte(0x0555, 0x56);
        self.write_byte(0x0AAA, 0x80);
        self.write_byte(0x0AAA, 0xA9);
        self.write_byte(0x0555, 0x56);
        self.write_byte(addr, 0x30);
        let mut ok = false;
        for _ in 0..100000 {
            delay(1000);
            let b = self.read_byte(addr);
            if b == 0xff {
                ok = true;
                break;
            }
        }
        ok
    }
    fn flash_info(&mut self) -> (u8, u8) {
        self.write_byte(0x0AAA, 0xA9);
        self.write_byte(0x0555, 0x56);
        self.write_byte(0x0AAA, 0x90);
        self.write_byte(0x0AAA, 0xA9);
        self.write_byte(0x0555, 0x56);
        self.write_byte(0x0AAA, 0x90);
        let manufacturer_id = self.read_byte(0x0000);
        let device_id = self.read_byte(0x0002);
        self.write_byte(0x0000, 0xFF); // reset
        (manufacturer_id, device_id)
    }
    fn flash_write(&mut self, addr: u16, bytes: &[u8]) -> Option<(u16, u8)> {
        let mut fail = None;
        const RETRIES: usize = 4;
        for (i, byte) in bytes.iter().enumerate() {
            for _ in 0..RETRIES {
                let (ok, b) = self.flash_write_byte(addr + i as u16, *byte);
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
                self.stats.flash_write_retries += 1;
            }
            if let Some(_) = fail {
                break;
            }
        }
        fail
    }
}

trait GBAAddrData {}

struct GBADataRead {
    data_0_13: GpioPortC<Input<PullDown>>,
    data_14: gpioa::PA0<Input<PullDown>>,
    data_15: gpioa::PA1<Input<PullDown>>,
    _unused: GpioPortB<Output<PushPull>>,
}

impl GBAAddrData for GBADataRead {}

impl GBADataRead {
    fn to_gba_addr(self) -> GBAAddr {
        GBAAddr {
            addr_0_13: self.data_0_13.into_push_pull_output(),
            addr_14: self.data_14.into_push_pull_output(),
            addr_15: self.data_15.into_push_pull_output(),
            addr_16_24: self._unused,
        }
    }

    fn get(&mut self) -> u16 {
        let mut data = self.data_0_13.read().unwrap() & 0b0011_1111_1111_1111;
        data |= if self.data_14.is_high().unwrap() {
            1 << 14
        } else {
            0
        };
        data |= if self.data_15.is_high().unwrap() {
            1 << 15
        } else {
            0
        };

        data
    }
}

struct GBADataWrite {
    data_0_13: GpioPortC<Output<PushPull>>,
    data_14: gpioa::PA0<Output<PushPull>>,
    data_15: gpioa::PA1<Output<PushPull>>,
    _unused: GpioPortB<Output<PushPull>>,
}

impl GBAAddrData for GBADataWrite {}

impl GBADataWrite {
    fn to_gba_addr(self) -> GBAAddr {
        GBAAddr {
            addr_0_13: self.data_0_13.into_push_pull_output(),
            addr_14: self.data_14.into_push_pull_output(),
            addr_15: self.data_15.into_push_pull_output(),
            addr_16_24: self._unused,
        }
    }
    fn set(&mut self, word: u16) {
        self.data_0_13.write(word).unwrap();
        if word & (1 << 14) != 0 {
            self.data_14.set_high()
        } else {
            self.data_14.set_low()
        }
        .unwrap();
        if word & (1 << 15) != 0 {
            self.data_15.set_high()
        } else {
            self.data_15.set_low()
        }
        .unwrap();
    }
}

struct GBAAddr {
    addr_0_13: GpioPortC<Output<PushPull>>,
    addr_14: gpioa::PA0<Output<PushPull>>,
    addr_15: gpioa::PA1<Output<PushPull>>,
    addr_16_24: GpioPortB<Output<PushPull>>,
}

impl GBAAddrData for GBAAddr {}

impl GBAAddr {
    fn to_gba_data_read(self) -> GBADataRead {
        GBADataRead {
            data_0_13: self.addr_0_13.into_pull_down_input(),
            data_14: self.addr_14.into_pull_down_input(),
            data_15: self.addr_15.into_pull_down_input(),
            _unused: self.addr_16_24,
        }
    }
    fn to_gba_data_write(self) -> GBADataWrite {
        GBADataWrite {
            data_0_13: self.addr_0_13,
            data_14: self.addr_14,
            data_15: self.addr_15,
            _unused: self.addr_16_24,
        }
    }
    fn set(&mut self, addr: u32) {
        let addr = addr >> 1;

        self.addr_0_13.write((addr & 0x0000ffff) as u16).unwrap();
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
        self.addr_16_24.write((addr >> 16) as u16).unwrap();
    }
}

struct GBACart {
    addr: Option<GBAAddr>,
    cs: gpioa::PA8<Output<PushPull>>,
    rd: gpioa::PA7<Output<PushPull>>,
    wr: gpioa::PA6<Output<PushPull>>,
    clk: gpioa::PA10<Output<PushPull>>,
    cs2: gpioa::PA9<Output<PushPull>>,
}

enum GBAPin {
    CS,
    _CS2,
    RD,
    WR,
    _CLK,
}

impl GBACart {
    fn take(rs: Resources) -> Self {
        Self {
            addr: Some(GBAAddr {
                addr_0_13: GpioPortC::take(rs.gpioc, 0b0011_1111_1111_1111).into_push_pull_output(),
                addr_14: rs.gpioa_0.into_push_pull_output(),
                addr_15: rs.gpioa_1.into_push_pull_output(),
                addr_16_24: GpioPortB::take(rs.gpiob, 0x00ff).into_push_pull_output(),
            }),
            cs: rs.gpioa_8.into_push_pull_output(),
            rd: rs.gpioa_7.into_push_pull_output(),
            wr: rs.gpioa_6.into_push_pull_output(),
            clk: rs.gpioa_10.into_push_pull_output(),
            cs2: rs.gpioa_9.into_push_pull_output(),
        }
    }

    fn release(mut self) -> Resources {
        Resources {
            gpioc: self.addr.take().unwrap().addr_0_13.release(),
            gpiob: self.addr.take().unwrap().addr_16_24.release(),
            gpioa_0: self.addr.take().unwrap().addr_14.into_floating_input(),
            gpioa_1: self.addr.take().unwrap().addr_15.into_floating_input(),
            gpioa_6: self.wr.into_floating_input(),
            gpioa_7: self.rd.into_floating_input(),
            gpioa_8: self.cs.into_floating_input(),
            gpioa_9: self.cs2.into_floating_input(),
            gpioa_10: self.clk.into_floating_input(),
        }
    }

    #[rustfmt::skip]
    fn set_pin(&mut self, pin: GBAPin, v: bool) {
        match pin {
            GBAPin::CS => { if v { self.cs.set_low() } else { self.cs.set_high() } }
            GBAPin::_CS2 => { if v { self.cs2.set_low() } else { self.cs2.set_high() } }
            GBAPin::RD => { if v { self.rd.set_low() } else { self.rd.set_high() } }
            GBAPin::WR => { if v { self.wr.set_low() } else { self.wr.set_high() } }
            GBAPin::_CLK => { if v { self.clk.set_low() } else { self.clk.set_high() } }
        }.unwrap()
    }
}

struct GBA {
    cart: GBACart,
}

impl GBA {
    fn latch_addr(&mut self, addr: u32) {
        let cart_addr = self.cart.addr.as_mut().unwrap();
        cart_addr.set(addr);
        self.cart.set_pin(GBAPin::CS, true);
        delay(200 * 3);
    }
    fn latch_addr_fast(&mut self, addr: u32) {
        let cart_addr = self.cart.addr.as_mut().unwrap();
        cart_addr.set(addr);
        self.cart.set_pin(GBAPin::CS, true);
        delay(10 * 3);
    }
    fn unlatch_addr(&mut self) {
        self.cart.set_pin(GBAPin::CS, false);
        delay(5 * 3);
    }
    fn _read_word(&mut self, data_read: &mut GBADataRead) -> u16 {
        self.cart.set_pin(GBAPin::RD, true);
        delay(50 * 1);
        let word = data_read.get();
        self.cart.set_pin(GBAPin::RD, false);
        delay(20 * 1);
        return word;
    }
    fn _read_word_fast(&mut self, data_read: &mut GBADataRead) -> u16 {
        self.cart.set_pin(GBAPin::RD, true);
        delay(5);
        let word = data_read.get();
        self.cart.set_pin(GBAPin::RD, false);
        delay(5);
        return word;
    }
    fn _write_word(&mut self, word: u16, data_write: &mut GBADataWrite) {
        self.cart.set_pin(GBAPin::WR, true);
        delay(5);
        data_write.set(word);
        self.cart.set_pin(GBAPin::WR, false);
        delay(5);
    }
    fn read_word(&mut self, addr: u32) -> u16 {
        self.latch_addr(addr);
        let mut data_read = self.cart.addr.take().unwrap().to_gba_data_read();
        let word = self._read_word(&mut data_read);
        self.unlatch_addr();

        self.cart.addr = Some(data_read.to_gba_addr());
        return word;
    }
    fn write_word(&mut self, addr: u32, word: u16) {
        self.latch_addr(addr);
        let mut data_write = self.cart.addr.take().unwrap().to_gba_data_write();
        self._write_word(word, &mut data_write);
        self.unlatch_addr();

        self.cart.addr = Some(data_write.to_gba_addr());
    }
    fn read(&mut self, addr: u32, buf: &mut [u8]) {
        self.latch_addr(addr);
        let mut data_read = self.cart.addr.take().unwrap().to_gba_data_read();
        for i in (0..buf.len()).step_by(2) {
            let word = self._read_word(&mut data_read);

            buf[i] = (word & 0x00ff) as u8;
            buf[i + 1] = ((word & 0xff00) >> 8) as u8;
        }
        self.unlatch_addr();

        self.cart.addr = Some(data_read.to_gba_addr());
    }
    fn flash_f3_unlock_sector(&mut self, sector: u32) -> bool {
        self.latch_addr_fast(sector);
        let mut data_write = self.cart.addr.take().unwrap().to_gba_data_write();
        self._write_word(0xff, &mut data_write);
        self._write_word(0x60, &mut data_write);
        self._write_word(0xd0, &mut data_write);
        self._write_word(0x90, &mut data_write);
        self.unlatch_addr();
        self.cart.addr = Some(data_write.to_gba_addr());

        true
        // for i in 0..1_000_000 {
        //     let b = self.read_word(sector + 2);
        //     if b & 0x0003 == 0x0000 {
        //         return true;
        //     }
        //     delay(100);
        // }
        // false
    }
    fn flash_f3_erase_sector(&mut self, sector: u32) -> bool {
        self.latch_addr_fast(sector);
        let mut data_write = self.cart.addr.take().unwrap().to_gba_data_write();
        self._write_word(0xff, &mut data_write);
        self._write_word(0x20, &mut data_write);
        self._write_word(0xd0, &mut data_write);
        self.unlatch_addr();
        self.cart.addr = Some(data_write.to_gba_addr());

        for _ in 0..1_000_000 {
            delay(1_000);
            let b = self.read_word(sector);
            if b == 0x80 {
                return true;
            }
        }
        false
    }
    fn flash_f3_write(&mut self, addr: u32, buf: &[u8]) -> Option<u32> {
        for i in (0..buf.len()).step_by(2) {
            self.latch_addr_fast(addr);
            let mut data_write = self.cart.addr.take().unwrap().to_gba_data_write();
            self._write_word(0xff, &mut data_write);
            self._write_word(0x40, &mut data_write);
            self.unlatch_addr();
            self.cart.addr = Some(data_write.to_gba_addr());

            let word = u16::from_le_bytes([buf[i], buf[i + 1]]);
            self.write_word(addr + i as u32, word);

            let mut ok = false;
            for _ in 0..1_000_000 {
                delay(1_000);
                let b = self.read_word(addr);
                if b == 0x80 {
                    ok = true;
                    break;
                }
            }
            if !ok {
                return Some(addr + i as u32);
            }
        }
        self.write_word(addr, 0xff);
        None
    }
    fn flash_write(&mut self, flash_type: FlashType, addr: u32, buf: &[u8]) -> Option<u32> {
        match flash_type {
            FlashType::F3 => self.flash_f3_write(addr, &buf),
        }
    }
    fn flash_unlock_sector(&mut self, flash_type: FlashType, sector: u32) -> bool {
        match flash_type {
            FlashType::F3 => self.flash_f3_unlock_sector(sector),
        }
    }
    fn flash_erase_sector(&mut self, flash_type: FlashType, sector: u32) -> bool {
        match flash_type {
            FlashType::F3 => self.flash_f3_erase_sector(sector),
        }
    }
    // test1: Unlock sector
    fn test1(&mut self) -> [(u16, u16); 8] {
        let mut result = [(0, 0); 8];

        self.latch_addr_fast(0);
        let mut data_write = self.cart.addr.take().unwrap().to_gba_data_write();
        self._write_word(0xff, &mut data_write);
        self._write_word(0x60, &mut data_write);
        self._write_word(0xd0, &mut data_write);
        self._write_word(0x90, &mut data_write);
        self.unlatch_addr();
        self.cart.addr = Some(data_write.to_gba_addr());

        for i in 0..2 {
            let a = self.read_word(0);
            let b = self.read_word(2);
            result[i] = (a, b);
            delay(10);
        }

        self.latch_addr_fast(0);
        let mut data_write = self.cart.addr.take().unwrap().to_gba_data_write();
        self._write_word(0xff, &mut data_write);
        self._write_word(0x20, &mut data_write);
        self._write_word(0xd0, &mut data_write);
        self.unlatch_addr();
        self.cart.addr = Some(data_write.to_gba_addr());

        for i in 2..4 {
            let a = self.read_word(0);
            let b = self.read_word(2);
            result[i] = (a, b);
            delay(10);
        }
        for _ in 0..1_000_000 {
            delay(1_000);
            let b = self.read_word(0);
            if b == 0x80 {
                result[0] = (1, 1);
                break;
            }
        }

        for v in 0..16 {
            self.latch_addr_fast(0);
            let mut data_write = self.cart.addr.take().unwrap().to_gba_data_write();
            self._write_word(0xff, &mut data_write);
            self._write_word(0x40, &mut data_write);
            self.unlatch_addr();
            self.cart.addr = Some(data_write.to_gba_addr());

            self.write_word(v * 2, v as u16);

            for _ in 0..1_000_000 {
                delay(1_000);
                let b = self.read_word(0);
                if b == 0x80 {
                    result[1] = (2, 2);
                    break;
                }
            }
            let b = self.read_word(v * 2);
            result[2] = (b, b);
        }
        // delay(1000000);

        self.write_word(0, 0xff);
        result
    }
}
