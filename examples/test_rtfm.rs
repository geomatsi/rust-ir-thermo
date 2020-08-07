#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m as cm;
use rtic::app;
use stm32l1xx_hal as hal;

use core::panic::PanicInfo;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;

#[app(device = stm32l1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led1: hal::gpio::gpiob::PB0<hal::gpio::Output<hal::gpio::PushPull>>,
        led2: hal::gpio::gpiob::PB1<hal::gpio::Output<hal::gpio::PushPull>>,
        tmr2: hal::timer::Timer<stm32::TIM2>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let mut rcc = cx.device.RCC.freeze(Config::hsi());

        let mut tmr2 = cx.device.TIM2.timer(1.hz(), &mut rcc);
        tmr2.listen();

        let gpiob = cx.device.GPIOB.split();
        let mut led1 = gpiob.pb0.into_push_pull_output();
        let mut led2 = gpiob.pb1.into_push_pull_output();

        led1.set_high().unwrap();
        led2.set_low().unwrap();

        init::LateResources {
            led1: led1,
            led2: led2,
            tmr2: tmr2,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cm::asm::wfi();
        }
    }

    #[task(binds = TIM2, resources = [led1, led2, tmr2])]
    fn tim2(cx: tim2::Context) {
        cx.resources.led1.toggle().unwrap();
        cx.resources.led2.toggle().unwrap();
        cx.resources.tmr2.clear_irq();
    }
};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
