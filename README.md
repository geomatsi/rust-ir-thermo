# Overview
Rust firmware for IR thermometer based on STM32L151CBT6 and MLX90614.

# Quick start guide
## Custom runner
The board does not have JTAG or SWD connector. So it has to be programmed through UART using ST bootloader.
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

## I2C selection using cargo features
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

## Firmware size checks
Use ```cargo-binutils`` to check firmware sections sizes:
```
$ cargo size --release --bin rust-ir-thermo
   text	   data	    bss	    dec	    hex	filename
  55480	      0	    212	  55692	   d98c	rust-ir-thermo

$ cargo size --release --example test-i2c-at24
   text	   data	    bss	    dec	    hex	filename
  19124	      0	     12	  19136	   4ac0	test-i2c-at24
  
$ cargo size --release --example test-i2c-at24 -- -A
test-i2c-at24  :
section               size        addr
.vector_table          292   0x8000000
.text                13696   0x8000124
.rodata               5136   0x80036b0
.data                    0  0x20000000
.bss                    12  0x20000000
.uninit                  0  0x2000000c
 ...
```

Use ```cargo-bloat``` to find out what takes most of the space in firmware binary:
```
$ cargo bloat --release --bin rust-ir-thermo 
File  .text    Size          Crate Name
0.7%  11.6%  5.2KiB      [Unknown] EXTI2
0.7%  11.1%  5.0KiB            std core::num::flt2dec::strategy::dragon::format_shortest
0.6%   9.1%  4.0KiB            std core::num::flt2dec::strategy::dragon::format_exact
0.3%   5.4%  2.4KiB            std core::num::flt2dec::strategy::grisu::format_shortest_opt
0.3%   4.6%  2.0KiB rust_ir_thermo rust_ir_thermo::init
```
