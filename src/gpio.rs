use core::convert::Infallible;
use stm32f4::stm32f411 as stm32;

use stm32::GPIOC;

pub struct GpioPortC {}

impl GpioPortC {
    fn write(&self, w: u16) -> Result<(), Infallible> {
        unsafe { (*GPIOC::ptr()).odr.write(|reg| reg.bits(w.into())) };
        Ok(())
    }
}

impl GpioPortC {
    fn read(&self) -> Result<u16, Infallible> {
        Ok(unsafe { (*GPIOC::ptr()).idr.read().bits() as u16 })
    }
}

// macro_rules! gpio {
//     ($GPIOX:ident, $gpiox:ident, $iopxenr:ident, $PXx:ident, $extigpionr:expr) => {
//     }
// }
