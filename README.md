# gb-rw-firm

Firmware for my GB/GBA cart reader/writer/flasher.

- [gb-rw-host](https://github.com/Dhole/gb-rw-host): Host part of the program
- [gb-cart-nucleo-shield](https://github.com/Dhole/gb-cart-nucleo-shield): PCB
  design to beused on top of a STM32 Nucleo compatible development board.

This firmware uses the
[`stm32f4xx-hal`](https://github.com/stm32-rs/stm32f4xx-hal) crate  and as such
should be easily adaptable to other stm32fYXX microcontrollers as well as any
other microcontroller that implements the
[`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.

It also uses my [`uRPC` library](https://github.com/Dhole/urpc) (server side)
to communicate with the host via UART.
