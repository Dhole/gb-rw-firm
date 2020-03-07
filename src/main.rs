#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Halt on panic
extern crate embedded_hal;
#[allow(unused_extern_crates)] // NOTE(allow) bug rust-lang/rust#53964
extern crate panic_halt; // panic handler

// use core::fmt::Write;
use core::fmt::Write;
use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use hal::{nb::block, prelude::*, serial::Serial, stm32};

// https://docs.rs/embedded-hal/0.2.3/src/embedded_hal/fmt.rs.html#1-17
// fn write_str<Word, Error>(&mut hal::serial::Write<Word, Error=Error>, s: &str) -> Result {
//     let _ = s.as_bytes()
//         .into_iter()
//         .map(|c| block!(self.write(Word::from(*c))))
//         .last();
//     Ok(())
// }

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();
    // Set up the LED. On the Nucleo-411RE it's connected to pin PA5.
    let gpioa = dp.GPIOA.split();
    let mut led = gpioa.pa5.into_push_pull_output();

    // Set up the system clock. We want to run at 48MHz for this one.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(96.mhz()).freeze();

    // USART2
    // Configure pa2 as a push_pull output, this will be the tx pin
    let tx = gpioa.pa2.into_alternate_af7();
    // Take ownership over pa3
    let rx = gpioa.pa3.into_alternate_af7();

    // Set up the usart device. Taks ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let (mut serial_tx, mut serial_rx) = hal::serial::Serial::usart2(
        // let mut serial = hal::serial::Serial::usart2(
        dp.USART2,
        (tx, rx),
        hal::serial::config::Config::default().baudrate(921600.bps()),
        clocks,
    )
    .unwrap()
    .split();

    // serial_tx.write_str("\nHELLO\n").ok();
    //serial.write(b'A').ok();
    // serial.write_str("\nHELLO\n").ok();
    // serial_tx.bwrite_all(b"\nHELLO\n").ok();
    serial_tx.bwrite_all(b"AAA").ok();

    // Create a delay abstraction based on SysTick
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    loop {
        // On for 1s, off for 1s.
        led.set_high().unwrap();
        delay.delay_ms(500_u32);
        led.set_low().unwrap();
        delay.delay_ms(500_u32);
    }
}
