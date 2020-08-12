#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m as cm;
use rtic::app;
use rtic::cyccnt::Instant;
use rtic::cyccnt::U32Ext;
use stm32l1xx_hal as hal;

use hd44780_driver::bus::FourBitBus;
use hd44780_driver::Cursor;
use hd44780_driver::CursorBlink;
use hd44780_driver::Display;
use hd44780_driver::DisplayMode;
use hd44780_driver::HD44780;

use core::fmt::Write;
use core::panic::PanicInfo;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v1_compat::OldOutputPin;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::timer::CountDown;

use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;
use hal::stm32::TIM3;
use hal::time::Hertz;
use hal::timer::Timer;

use heapless::consts::*;
use heapless::spsc::Queue;

use hal::gpio::gpioa::{PA4, PA5, PA8};
use hal::gpio::gpiob::{PB0, PB1, PB10, PB12, PB13, PB14, PB15};
use hal::gpio::{Floating, Input, Output, PushPull};

use nb::block;

#[derive(Debug, Clone, Copy)]
pub enum Menu {
    Shot,
    ShotMem,
    Cont,
    ContMem,
}

#[derive(Debug, Clone, Copy)]
pub enum State {
    Select(Menu),
    Active(Menu),
}

impl State {
    pub fn next(state: State) -> State {
        match state {
            State::Select(Menu::Shot) => State::Select(Menu::ShotMem),
            State::Select(Menu::ShotMem) => State::Select(Menu::Cont),
            State::Select(Menu::Cont) => State::Select(Menu::ContMem),
            State::Select(Menu::ContMem) => State::Select(Menu::Shot),
            _ => state,
        }
    }

    pub fn prev(state: State) -> State {
        match state {
            State::Select(Menu::Shot) => State::Select(Menu::ContMem),
            State::Select(Menu::ShotMem) => State::Select(Menu::Shot),
            State::Select(Menu::Cont) => State::Select(Menu::ShotMem),
            State::Select(Menu::ContMem) => State::Select(Menu::Cont),
            _ => state,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Event {
    Button1,
    Button2,
    Enter,
}

pub struct DelayTimer<Timer>
where
    Timer: CountDown,
    Timer::Time: From<Hertz>,
{
    timer: Timer,
}

impl<Timer> DelayTimer<Timer>
where
    Timer: CountDown,
    Timer::Time: From<Hertz>,
{
    pub fn new(timer: Timer) -> Self {
        Self { timer }
    }
}

impl<Timer> DelayMs<u8> for DelayTimer<Timer>
where
    Timer: CountDown,
    Timer::Time: From<Hertz>,
{
    fn delay_ms(&mut self, ms: u8) {
        if ms == 0 {
            return;
        }

        self.timer.start((1000u32 / ms as u32).hz());
        block!(self.timer.wait()).unwrap();
    }
}

impl<Timer> DelayUs<u16> for DelayTimer<Timer>
where
    Timer: CountDown,
    Timer::Time: From<Hertz>,
{
    fn delay_us(&mut self, us: u16) {
        if us == 0 {
            return;
        }

        self.timer.start((1000000u32 / us as u32).hz());
        block!(self.timer.wait()).unwrap();
    }
}

/* HSI clock : 16MHz  */
const TEST_PERIOD: u32 = 8_000_000; /* 0.5 sec */
const PROC_PERIOD: u32 = 4_000_000; /* 0.25 sec */

/*  */

type LcdType = HD44780<
    DelayTimer<Timer<TIM3>>,
    FourBitBus<
        OldOutputPin<PB10<Output<PushPull>>>,
        OldOutputPin<PA8<Output<PushPull>>>,
        OldOutputPin<PB12<Output<PushPull>>>,
        OldOutputPin<PB13<Output<PushPull>>>,
        OldOutputPin<PB14<Output<PushPull>>>,
        OldOutputPin<PB15<Output<PushPull>>>,
    >,
>;

/* */

#[app(device = stm32l1xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // resources
        #[init(0)]
        cb1: u8,
        #[init(0)]
        cb2: u8,
        // late resources
        led1: PB0<Output<PushPull>>,
        led2: PB1<Output<PushPull>>,
        button1: PA4<Input<Floating>>,
        button2: PA5<Input<Floating>>,
        tmr2: hal::timer::Timer<stm32::TIM2>,
        queue: Queue<Event, U4>,
        state: State,
        lcd: LcdType,
    }

    #[init(schedule = [test_task, proc_task])]
    fn init(mut cx: init::Context) -> init::LateResources {
        /* init hardware */

        let mut rcc = cx.device.RCC.freeze(Config::hsi());
        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();

        /* init buttons */

        let mut tmr2 = cx.device.TIM2.timer(10.hz(), &mut rcc);
        tmr2.listen();

        let button1 = gpioa.pa4.into_floating_input();
        let button2 = gpioa.pa5.into_floating_input();

        /* init LEDs */

        let mut led1 = gpiob.pb0.into_push_pull_output();
        let mut led2 = gpiob.pb1.into_push_pull_output();

        led1.set_high().unwrap();
        led2.set_low().unwrap();

        /* enable monotonic timer */

        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        /* init LCD */

        let tmr3 = cx.device.TIM3.timer(1.mhz(), &mut rcc);
        let delay = DelayTimer { timer: tmr3 };

        let mut vo = gpiob.pb6.into_push_pull_output();
        let mut v5 = gpiob.pb7.into_push_pull_output();
        let mut rw = gpiob.pb11.into_push_pull_output();

        let en = gpioa.pa8.into_push_pull_output();
        let rs = gpiob.pb10.into_push_pull_output();

        let b4 = gpiob.pb12.into_push_pull_output();
        let b5 = gpiob.pb13.into_push_pull_output();
        let b6 = gpiob.pb14.into_push_pull_output();
        let b7 = gpiob.pb15.into_push_pull_output();

        let mut lcd = HD44780::new_4bit(
            OldOutputPin::from(rs),
            OldOutputPin::from(en),
            OldOutputPin::from(b4),
            OldOutputPin::from(b5),
            OldOutputPin::from(b6),
            OldOutputPin::from(b7),
            delay,
        );

        vo.set_low().unwrap();
        v5.set_high().unwrap();
        rw.set_low().unwrap();

        lcd.reset();
        lcd.clear();
        lcd.set_display_mode(DisplayMode {
            display: Display::On,
            cursor_visibility: Cursor::Invisible,
            cursor_blink: CursorBlink::Off,
        });

        /* initial state */

        let state = State::Select(Menu::Shot);

        lcd.clear();
        lcd.set_cursor_pos(0);
        lcd.write_str("Shot").unwrap();

        /* priority queue */

        let queue = Queue(heapless::i::Queue::new());

        /* schedule tasks */

        cx.schedule
            .test_task(Instant::now() + TEST_PERIOD.cycles())
            .unwrap();

        cx.schedule.proc_task(Instant::now()).unwrap();

        /* init late resources */

        init::LateResources {
            state,
            queue,
            lcd,
            led1,
            led2,
            tmr2,
            button1,
            button2,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cm::asm::wfi();
        }
    }

    #[task(binds = TIM2, resources = [tmr2, queue, button1, cb1, button2, cb2])]
    fn tim2(cx: tim2::Context) {
        if cx.resources.button1.is_low().unwrap() {
            *cx.resources.cb1 += 1;
        } else {
            match *cx.resources.cb1 {
                x if 1 <= x && x <= 3 => {
                    cx.resources.queue.enqueue(Event::Button1).ok();
                    *cx.resources.cb1 = 0;
                }
                x if x > 3 => {
                    cx.resources.queue.enqueue(Event::Enter).ok();
                    *cx.resources.cb1 = 0;
                }
                _ => {
                    *cx.resources.cb1 = 0;
                }
            }
        }

        if cx.resources.button2.is_low().unwrap() {
            *cx.resources.cb2 += 1;
        } else {
            match *cx.resources.cb2 {
                x if 1 <= x && x <= 3 => {
                    cx.resources.queue.enqueue(Event::Button2).ok();
                    *cx.resources.cb2 = 0;
                }
                x if x > 3 => {
                    cx.resources.queue.enqueue(Event::Enter).ok();
                    *cx.resources.cb2 = 0;
                }
                _ => {
                    *cx.resources.cb2 = 0;
                }
            }
        }

        cx.resources.tmr2.clear_irq();
    }

    #[task(schedule = [test_task], resources = [led1, led2])]
    fn test_task(cx: test_task::Context) {
        cx.resources.led1.toggle().unwrap();
        cx.resources.led2.toggle().unwrap();
        cx.schedule
            .test_task(cx.scheduled + TEST_PERIOD.cycles())
            .unwrap();
    }

    #[task(schedule = [proc_task], resources = [queue, state, lcd])]
    fn proc_task(cx: proc_task::Context) {
        let state = cx.resources.state;
        let queue = cx.resources.queue;
        let lcd = cx.resources.lcd;

        while !queue.is_empty() {
            if let Some(e) = queue.dequeue() {
                match state {
                    State::Select(x) => match e {
                        Event::Button1 => *state = State::next(*state),
                        Event::Button2 => *state = State::prev(*state),
                        Event::Enter => *state = State::Active(*x),
                    },
                    State::Active(x) => match e {
                        _ => *state = State::Select(*x),
                    },
                }
            }

            match state {
                State::Select(Menu::Shot) => {
                    lcd.clear();
                    lcd.set_cursor_pos(0);
                    lcd.write_str("Shot").unwrap();
                }
                State::Select(Menu::ShotMem) => {
                    lcd.clear();
                    lcd.set_cursor_pos(0);
                    lcd.write_str("Shot Mem").unwrap();
                }
                State::Select(Menu::Cont) => {
                    lcd.clear();
                    lcd.set_cursor_pos(0);
                    lcd.write_str("Cont").unwrap();
                }
                State::Select(Menu::ContMem) => {
                    lcd.clear();
                    lcd.set_cursor_pos(0);
                    lcd.write_str("Cont Mem").unwrap();
                }
                State::Active(_) => {
                    lcd.clear();
                    lcd.set_cursor_pos(0);
                    lcd.write_str("NOT YET").unwrap();
                }
            }
        }

        cx.schedule
            .proc_task(cx.scheduled + PROC_PERIOD.cycles())
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
