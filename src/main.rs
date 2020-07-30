#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m as cm;
use cortex_m_rt as rt;
use stm32l1xx_hal as hal;

use core::panic::PanicInfo;
use hal::prelude::*;
use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let gpiob = dp.GPIOB.split();
    let mut led1 = gpiob.pb0.into_push_pull_output();
    let mut led2 = gpiob.pb1.into_push_pull_output();

    loop {
        led1.set_high();
        led2.set_low();

        delay(500);

        led1.set_low();
        led1.set_low();

        delay(500);
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

fn delay(count: u32) {
    for _ in 0..count {
        cm::asm::nop();
    }
}
