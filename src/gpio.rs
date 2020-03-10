use core::convert::Infallible;
use core::marker::PhantomData;
use stm32f4::stm32f411 as stm32;

use stm32::GPIOC;

pub struct GpioPortC<MODE> {
    gpio: GPIOC,
    mask: u16,
    reg_vals: RegValues,
    _mode: PhantomData<MODE>,
}

impl<MODE> GpioPortC<Output<MODE>> {
    pub fn write(&self, word: u16) -> Result<(), Infallible> {
        Ok(unsafe {
            (*GPIOC::ptr())
                .odr
                .modify(|r, w| w.bits((r.bits() & 0xffff0000) | word as u32))
        })
    }
}

impl<MODE> GpioPortC<Input<MODE>> {
    pub fn read(&self) -> Result<u16, Infallible> {
        Ok(unsafe { ((*GPIOC::ptr()).idr.read().bits() & 0x0000ffff) as u16 })
    }
}

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

// fn loop_mask(mask: u16, f: fn(u32)) {
//     for i in 0..16 {
//         if 1 << i & mask == 0 {
//             continue;
//         }
//         f(i);
//     }
// }

struct RegValues {
    clr: u32,
    v00: u32,
    v01: u32,
    v10: u32,
    // v11: u32,
}

impl RegValues {
    fn new(mask: u16) -> Self {
        Self {
            clr: !(0..16).map(|i| 0b11 << 2 * i).sum::<u32>(),
            v00: (0..16).map(|i| 0b00 << 2 * i).sum(),
            v01: (0..16).map(|i| 0b01 << 2 * i).sum(),
            v10: (0..16).map(|i| 0b10 << 2 * i).sum(),
            // v11: (0..16).map(|i| 0b11 << 2 * i).sum(),
        }
    }
}

impl GpioPortC<None> {
    pub fn take(gpio: GPIOC, mask: u16) -> GpioPortC<Input<Floating>> {
        GpioPortC {
            gpio,
            mask,
            reg_vals: RegValues::new(mask),
            _mode: PhantomData,
        }
    }
}

impl<MODE> GpioPortC<MODE> {
    pub fn release(self) -> GPIOC {
        self.gpio
    }

    /// Configures the port to operate as a floating input port
    pub fn into_floating_input(self) -> GpioPortC<Input<Floating>> {
        unsafe {
            &(*GPIOC::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00));
            &(*GPIOC::ptr())
                .moder
                .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00))
        };

        GpioPortC {
            gpio: self.gpio,
            mask: self.mask,
            reg_vals: self.reg_vals,
            _mode: PhantomData,
        }
    }

    /// Configures the port to operate as a pulled down input port
    pub fn into_pull_down_input(self) -> GpioPortC<Input<PullDown>> {
        unsafe {
            &(*GPIOC::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v10));
            &(*GPIOC::ptr())
                .moder
                .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00))
        };

        GpioPortC {
            gpio: self.gpio,
            mask: self.mask,
            reg_vals: self.reg_vals,
            _mode: PhantomData,
        }
    }

    /// Configures the port to operate as a pulled up input port
    pub fn into_pull_up_input(self) -> GpioPortC<Input<PullUp>> {
        unsafe {
            &(*GPIOC::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v01));
            &(*GPIOC::ptr())
                .moder
                .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00))
        };

        GpioPortC {
            gpio: self.gpio,
            mask: self.mask,
            reg_vals: self.reg_vals,
            _mode: PhantomData,
        }
    }

    /// Configures the port to operate as an open drain output port
    pub fn into_open_drain_output(self) -> GpioPortC<Output<OpenDrain>> {
        unsafe {
            &(*GPIOC::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00));
            &(*GPIOC::ptr())
                .otyper
                .modify(|r, w| w.bits(r.bits() | (self.mask as u32)));
            &(*GPIOC::ptr())
                .moder
                .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v01))
        };

        GpioPortC {
            gpio: self.gpio,
            mask: self.mask,
            reg_vals: self.reg_vals,
            _mode: PhantomData,
        }
    }

    /// Configures the port to operate as an push pull output port
    pub fn into_push_pull_output(self) -> GpioPortC<Output<PushPull>> {
        unsafe {
            &(*GPIOC::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v00));
            &(*GPIOC::ptr())
                .otyper
                .modify(|r, w| w.bits(r.bits() & !(self.mask as u32)));
            &(*GPIOC::ptr())
                .moder
                .modify(|r, w| w.bits((r.bits() & self.reg_vals.clr) | self.reg_vals.v01))
        };

        GpioPortC {
            gpio: self.gpio,
            mask: self.mask,
            reg_vals: self.reg_vals,
            _mode: PhantomData,
        }
    }
}
