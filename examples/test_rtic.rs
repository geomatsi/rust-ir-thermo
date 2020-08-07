#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m as cm;
use rtic::app;
use rtic::cyccnt::Instant;
use rtic::cyccnt::U32Ext;
use stm32l1xx_hal as hal;

use core::panic::PanicInfo;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;

/* HSI clock : 16MHz  */

const TEST_PERIOD: u32 = 8_000_000; /* 0.5 sec */

#[app(device = stm32l1xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        led1: hal::gpio::gpiob::PB0<hal::gpio::Output<hal::gpio::PushPull>>,
        led2: hal::gpio::gpiob::PB1<hal::gpio::Output<hal::gpio::PushPull>>,
        tmr2: hal::timer::Timer<stm32::TIM2>,
    }

    #[init(schedule = [test_task])]
    fn init(mut cx: init::Context) -> init::LateResources {
        /* init hardware */
        let mut rcc = cx.device.RCC.freeze(Config::hsi());

        let mut tmr2 = cx.device.TIM2.timer(1.hz(), &mut rcc);
        tmr2.listen();

        let gpiob = cx.device.GPIOB.split();
        let mut led1 = gpiob.pb0.into_push_pull_output();
        let mut led2 = gpiob.pb1.into_push_pull_output();

        led1.set_high().unwrap();
        led2.set_low().unwrap();

        /* enable monotonic timer */

        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        /* schedule tasks */

        cx.schedule
            .test_task(Instant::now() + TEST_PERIOD.cycles())
            .unwrap();

        /* init late resources */

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

    #[task(binds = TIM2, resources = [led1, tmr2])]
    fn tim2(cx: tim2::Context) {
        cx.resources.led1.toggle().unwrap();
        cx.resources.tmr2.clear_irq();
    }

    #[task(schedule = [test_task], resources = [led2])]
    fn test_task(cx: test_task::Context) {
        cx.resources.led2.toggle().unwrap();
        cx.schedule
            .test_task(cx.scheduled + TEST_PERIOD.cycles())
            .unwrap();
    }

    // needed for RTIC timer queue and task management
    extern "C" {
        fn EXTI2();
    }
};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
