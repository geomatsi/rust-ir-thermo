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
Use ```cargo-binutils`` to check firmware size summary:
```
$ cargo size --release --bin rust-ir-thermo
   text	   data	    bss	    dec	    hex	filename
  55480	      0	    212	  55692	   d98c	rust-ir-thermo

$ cargo size --release --example test-i2c-at24
   text	   data	    bss	    dec	    hex	filename
  19124	      0	     12	  19136	   4ac0	test-i2c-at24
```

Use additional option to get all sections sizes:
```
$ cargo size --release --bin rust-ir-thermo -- -A   
rust-ir-thermo  :
section               size        addr
.vector_table          292   0x8000000
.text                46264   0x8000124
.rodata               9856   0x800b5e0
.data                    0  0x20000000
.bss                   104  0x20000000
.uninit                132  0x20000068
.ARM.attributes         50         0x0
.debug_str          171382         0x0
.debug_abbrev         3517         0x0
.debug_info         130921         0x0
.debug_aranges        6816         0x0
.debug_ranges        90784         0x0
.debug_pubnames      35430         0x0
.debug_pubtypes        378         0x0
.debug_frame         21924         0x0
.debug_line         142910         0x0
.comment               147         0x0
Total               660907
```

Use ```cargo-bloat``` to find out what takes most of the space in firmware binary:
```
$ cargo bloat --release --bin rust-ir-thermo 
File  .text    Size          Crate Name
0.7%  11.5%  5.2KiB      [Unknown] EXTI2
0.7%  11.0%  5.0KiB            std core::num::flt2dec::strategy::dragon::format_shortest
0.6%   9.0%  4.0KiB            std core::num::flt2dec::strategy::dragon::format_exact
0.3%   5.3%  2.4KiB            std core::num::flt2dec::strategy::grisu::format_shortest_opt
0.3%   5.1%  2.3KiB rust_ir_thermo rust_ir_thermo::init
0.2%   3.6%  1.6KiB            std core::num::flt2dec::strategy::grisu::format_exact_opt
0.1%   2.3%  1.0KiB            std compiler_builtins::int::udiv::__udivmoddi4
0.1%   2.3%  1.0KiB            std core::fmt::float::float_to_decimal_common_exact
0.1%   1.8%    816B            std core::fmt::float::float_to_decimal_common_shortest
0.1%   1.7%    808B            std core::fmt::Formatter::pad
0.1%   1.7%    796B       mlx9061x mlx9061x::crc8::crc8
0.1%   1.7%    776B            std core::str::slice_error_fail
0.1%   1.7%    774B            std core::num::bignum::Big32x40::mul_digits
0.1%   1.5%    690B            std core::fmt::Formatter::write_formatted_parts
0.1%   1.5%    688B            std core::unicode::printable::is_printable
0.1%   1.4%    664B            std core::fmt::Formatter::pad_integral
0.1%   1.4%    656B  stm32l1xx_hal <stm32l1::stm32l151::RCC as stm32l1xx_hal::rcc::RccExt>::freeze
0.1%   1.4%    636B            std core::num::flt2dec::strategy::dragon::mul_pow10
0.1%   1.3%    604B            std core::fmt::num::<impl core::fmt::Debug for usize>::fmt
0.1%   1.1%    506B rust_ir_thermo rust_ir_thermo::APP::schedule_sleep_task_S0
2.0%  31.7% 14.3KiB                And 198 smaller methods. Use -n N to show more.
6.4% 100.0% 45.2KiB                .text section size, the file size is 707.0KiB
```
