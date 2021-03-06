#![macro_use]

macro_rules! _loop_nop {
    ($n:expr) => {
        for i in 0..$n {
            unsafe {
                asm!("NOP");
            }
        }
    };
}
