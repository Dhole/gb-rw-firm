#!/bin/sh

set -ex

NAME="$1"
if [ "$NAME" = "" ]; then
    NAME=`basename ${PWD}`
else
    EXAMPLE=yes
fi

if [ "$EXAMPLE" = "yes" ]; then
    cargo build --example ${NAME} --release
    arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/examples/${NAME} ${NAME}.bin
else
    cargo build --release
    arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/${NAME} ${NAME}.bin
fi

# stlink version
# st-flash erase
# st-flash write ${NAME}.bin 0x8000000

# OpenOCD version
# http://openocd.org/doc/html/Flash-Programming.html
openocd -f openocd.cfg -c "program ${NAME}.bin reset exit 0x8000000"
