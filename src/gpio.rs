use core::convert::Infallible;
use core::marker::PhantomData;
use stm32f4::stm32f411 as stm32;
use stm32f4xx_hal as hal;

/// No configured mode (type state)
pub struct None;

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;

/// Pulled down input (type state)
pub struct PullDown;

/// Pulled up input (type state)
pub struct PullUp;

/// Open drain input or output (type state)
pub struct OpenDrain;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;

struct RegValues {
    clr: u32,
    v00: u32,
    v01: u32,
    v10: u32,
    // v11: u32,
}

struct RegSnapshot {
    pupdr: u32,
    moder: u32,
    otyper: u16,
}

impl RegValues {
    fn new(mask: u16) -> Self {
        let iter = (0..16).filter(|i| mask & 1u16 << i != 0u16);
        Self {
            clr: !iter.clone().map(|i| 0b11 << (2 * i)).sum::<u32>(),
            v00: iter.clone().map(|i| 0b00 << (2 * i)).sum(),
            v01: iter.clone().map(|i| 0b01 << (2 * i)).sum(),
            v10: iter.map(|i| 0b10 << (2 * i)).sum(),
            // v11: (0..16).map(|i| 0b11 << 2 * i).sum(),
        }
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $GpioPort:ident, $px:ident) => {
        use hal::gpio::$gpiox;
        use stm32::$GPIOX;

        pub struct $GpioPort<MODE> {
            gpio_parts: $gpiox::Parts,
            reg_snap: RegSnapshot,
            mask: u16,
            reg_vals: RegValues,
            _mode: PhantomData<MODE>,
        }

        impl<MODE> $GpioPort<Output<MODE>> {
            pub fn write(&self, word: u16) -> Result<(), Infallible> {
                Ok(unsafe {
                    (*$GPIOX::ptr())
                        .odr
                        .modify(|r, w| w.bits((r.bits() & 0xffff0000) | word as u32))
                })
            }
        }

        impl<MODE> $GpioPort<Input<MODE>> {
            pub fn read(&self) -> Result<u16, Infallible> {
                Ok(unsafe { ((*$GPIOX::ptr()).idr.read().bits() & 0x0000ffff) as u16 })
            }
        }

        impl $GpioPort<None> {
            pub fn take(gpio_parts: $gpiox::Parts, mask: u16) -> $GpioPort<Input<Floating>> {
                let reg_snap = RegSnapshot {
                    pupdr: unsafe { (*$GPIOX::ptr()).pupdr.read().bits() },
                    moder: unsafe { (*$GPIOX::ptr()).moder.read().bits() },
                    otyper: unsafe { ((*$GPIOX::ptr()).otyper.read().bits() & 0x0000ffff) as u16 },
                };
                $GpioPort {
                    gpio_parts,
                    reg_snap,
                    mask,
                    reg_vals: RegValues::new(mask),
                    _mode: PhantomData,
                }
            }
        }

        impl<MODE> $GpioPort<MODE> {
            pub fn release(self) -> $gpiox::Parts {
                // Restore all the GPIOs to the they where before taking the port
                unsafe {
                    &(*$GPIOX::ptr())
                        .pupdr
                        .write(|w| w.bits(self.reg_snap.pupdr));
                    &(*$GPIOX::ptr())
                        .moder
                        .write(|w| w.bits(self.reg_snap.moder));
                    &(*$GPIOX::ptr()).otyper.modify(|r, w| {
                        w.bits((r.bits() & 0xffff0000) | self.reg_snap.otyper as u32)
                    });
                };
                self.gpio_parts
            }

            /// Configures the port to operate as a floating input port
            pub fn into_floating_input(self) -> $GpioPort<Input<Floating>> {
                unsafe {
                    &(*$GPIOX::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00));
                    &(*$GPIOX::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00));
                };

                $GpioPort {
                    gpio_parts: self.gpio_parts,
                    reg_snap: self.reg_snap,
                    mask: self.mask,
                    reg_vals: self.reg_vals,
                    _mode: PhantomData,
                }
            }

            /// Configures the port to operate as a pulled down input port
            pub fn into_pull_down_input(self) -> $GpioPort<Input<PullDown>> {
                unsafe {
                    &(*$GPIOX::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v10));
                    &(*$GPIOX::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00));
                };

                $GpioPort {
                    gpio_parts: self.gpio_parts,
                    reg_snap: self.reg_snap,
                    mask: self.mask,
                    reg_vals: self.reg_vals,
                    _mode: PhantomData,
                }
            }

            /// Configures the port to operate as a pulled up input port
            pub fn into_pull_up_input(self) -> $GpioPort<Input<PullUp>> {
                unsafe {
                    &(*$GPIOX::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v01));
                    &(*$GPIOX::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00));
                };

                $GpioPort {
                    gpio_parts: self.gpio_parts,
                    reg_snap: self.reg_snap,
                    mask: self.mask,
                    reg_vals: self.reg_vals,
                    _mode: PhantomData,
                }
            }

            /// Configures the port to operate as an open drain output port
            pub fn into_open_drain_output(self) -> $GpioPort<Output<OpenDrain>> {
                unsafe {
                    &(*$GPIOX::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00));
                    &(*$GPIOX::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() | (self.mask as u32)));
                    &(*$GPIOX::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v01));
                };

                $GpioPort {
                    gpio_parts: self.gpio_parts,
                    reg_snap: self.reg_snap,
                    mask: self.mask,
                    reg_vals: self.reg_vals,
                    _mode: PhantomData,
                }
            }

            /// Configures the port to operate as an push pull output port
            pub fn into_push_pull_output(self) -> $GpioPort<Output<PushPull>> {
                unsafe {
                    &(*$GPIOX::ptr())
                        .pupdr
                        .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00));
                    &(*$GPIOX::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(self.mask as u32)));
                    &(*$GPIOX::ptr())
                        .moder
                        .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v01));
                };

                $GpioPort {
                    gpio_parts: self.gpio_parts,
                    reg_snap: self.reg_snap,
                    mask: self.mask,
                    reg_vals: self.reg_vals,
                    _mode: PhantomData,
                }
            }
        }
    };
}

gpio!(GPIOA, gpioa, GpioPortA, pa);
gpio!(GPIOB, gpiob, GpioPortB, pb);
gpio!(GPIOC, gpioc, GpioPortC, pc);
