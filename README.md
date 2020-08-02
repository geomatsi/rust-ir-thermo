# Overview
Rust firmware for IR thermometer based on STM32L151CBT6 and MLX90614.

# Quick start guide
The board does not have JTAG or SWD. So it has to be programmed through UART using ST bootloader.
To simplify build and flash procedure, custom cargo runner is implemented. This script does
the following two things:
* run ```cargo-binutils``` to generate ```ihex``` binary format
* run ```stm32flash``` to program device through UART

Note that BOOT0 jumper shall be properly connected to switch MCU into UART boot mode.

Custom runner calls stm32flash and llvm-objdump using certain default parameters,
including baudrate, serial port, and binary format. Those changes can be modified
using custom runner script input parameters:

```
cargo run --release --example test-led -- -h

Examples:
$ cargo run --example test-led
$ cargo run --release --example test-led
$ cargo run --example test-led -- -p /dev/ttyUSB1 -r 9600
$ cargo run --example test-led -- -t bin

Supported additional options:
Options:
  -t, --type          bin or ihex, default is ihex
  -p, --port          serial port, default is /dev/ttyUSB0
  -r, --rate          serial port baud rate, default is 115200
```

Tests and binaries support two different i2c implementations:
* i2c hardware block on stm32l1x microcontroller
* i2c software bitbang implementation

Any of the options can be selected using ```cargo``` features: ```i2c_hw``` or ```i2c_bb```.
By default ```i2c_hw``` option is selected:

```
$ cargo run --release --example test-i2c-at24
$ cargo run --release --example test-i2c-mlx90614
$ cargo run --release --no-default-features --features i2c_bb --example test-i2c-at24
$ cargo run --release --no-default-features --features i2c_bb --example test-i2c-mlx90614
```

# TODO

* investigate why MLX90614 sensor does not work with ```s/w bitbang i2c``` (data always 0xFF)
