#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m as cm;
use rtic::app;
use rtic::cyccnt::Instant;
use rtic::cyccnt::U32Ext;
use stm32l1xx_hal as hal;

use hd44780_driver::bus::FourBitBus;
use hd44780_driver::HD44780;

use core::cell::RefCell;
use core::fmt;
use core::fmt::Write;
use core::panic::PanicInfo;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v1_compat::OldOutputPin;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::timer::CountDown;

use hal::gpio::gpioa::{PA14, PA4, PA5, PA8};
use hal::gpio::gpiob::{PB0, PB1, PB10, PB12, PB13, PB14, PB15, PB5, PB8, PB9};
use hal::gpio::{Floating, Input, OpenDrain, Output, PushPull};
use hal::i2c::I2c;
use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;
use hal::stm32::I2C1;
use hal::stm32::{TIM3, TIM4};
use hal::time::Hertz;
use hal::timer::Timer;

use heapless::consts::*;
use heapless::spsc::Queue;

use eeprom24x::addr_size::TwoBytes;
use eeprom24x::page_size::B256;
use eeprom24x::Eeprom24x;

use mlx9061x::ic::Mlx90614;
use mlx9061x::Mlx9061x;

use nb::block;

#[derive(Clone, Copy, PartialEq)]
pub enum Menu {
    Shot,
    ShotMem,
    Cont,
    ContMem,
    View,
    Stream,
}

#[derive(Clone, Copy, PartialEq)]
pub enum State {
    Select(Menu),
    Active(Menu),
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let s = match *self {
            State::Select(Menu::Shot) | State::Active(Menu::Shot) => "Shot",
            State::Select(Menu::Cont) | State::Active(Menu::Cont) => "Cont",
            State::Select(Menu::ShotMem) => "ShotMem",
            State::Active(Menu::ShotMem) => "SM",
            State::Select(Menu::ContMem) => "ContMem",
            State::Active(Menu::ContMem) => "CM",
            State::Select(Menu::View) => "View",
            State::Active(Menu::View) => "VM",
            State::Select(Menu::Stream) => "Stream",
            State::Active(Menu::Stream) => "SM",
        };
        write!(f, "{}", s)
    }
}

impl State {
    pub fn next(state: State) -> State {
        match state {
            State::Select(Menu::Shot) => State::Select(Menu::ShotMem),
            State::Select(Menu::ShotMem) => State::Select(Menu::Cont),
            State::Select(Menu::Cont) => State::Select(Menu::ContMem),
            State::Select(Menu::ContMem) => State::Select(Menu::View),
            State::Select(Menu::View) => State::Select(Menu::Stream),
            State::Select(Menu::Stream) => State::Select(Menu::Shot),
            _ => state,
        }
    }

    pub fn prev(state: State) -> State {
        match state {
            State::Select(Menu::Shot) => State::Select(Menu::Stream),
            State::Select(Menu::ShotMem) => State::Select(Menu::Shot),
            State::Select(Menu::Cont) => State::Select(Menu::ShotMem),
            State::Select(Menu::ContMem) => State::Select(Menu::Cont),
            State::Select(Menu::View) => State::Select(Menu::ContMem),
            State::Select(Menu::Stream) => State::Select(Menu::View),
            _ => state,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Event {
    Button1,
    Button2,
    Enter,
    Repeat,
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

const CONT_PERIOD: u32 = 16_000_000; /* 1 sec */
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

type TypeI2cBus = I2c<I2C1, (PB8<Output<OpenDrain>>, PB9<Output<OpenDrain>>)>;
type TempType<'a> = Mlx9061x<
    shared_bus::BusProxy<'a, cm::interrupt::Mutex<RefCell<TypeI2cBus>>, TypeI2cBus>,
    Mlx90614,
>;
type EepromType<'a> = Eeprom24x<
    shared_bus::BusProxy<'a, cm::interrupt::Mutex<RefCell<TypeI2cBus>>, TypeI2cBus>,
    B256,
    TwoBytes,
>;

/* */

/*
 * Dark magic with shared_bus lifetimes: see discussion at https://github.com/Rahix/shared-bus/issues/4
 */
static mut I2C_BUS: Option<
    shared_bus::BusManager<cortex_m::interrupt::Mutex<RefCell<TypeI2cBus>>, TypeI2cBus>,
> = None;

/* */

#[app(device = stm32l1xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // resources
        #[init(0)]
        cb1: u8,
        #[init(0)]
        cb2: u8,
        #[init(None)]
        pos: Option<u32>,

        // late resources
        led1: PB0<Output<PushPull>>,
        led2: PB1<Output<PushPull>>,
        button1: PA4<Input<Floating>>,
        button2: PA5<Input<Floating>>,
        tmr2: hal::timer::Timer<stm32::TIM2>,
        queue: Queue<Event, U4>,
        state: State,
        lcd: LcdType,
        temp: TempType<'static>,
        temp_en: PA14<Output<PushPull>>,
        eeprom: EepromType<'static>,
        eeprom_wp: PB5<Output<PushPull>>,
        eeprom_tmr: DelayTimer<Timer<TIM4>>,
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
        lcd.set_display_mode(hd44780_driver::DisplayMode {
            display: hd44780_driver::Display::On,
            cursor_visibility: hd44780_driver::Cursor::Invisible,
            cursor_blink: hd44780_driver::CursorBlink::Off,
        });

        /*
         * init I2C shared bus
         * see discussion at https://github.com/Rahix/shared-bus/issues/4
         */

        let scl = gpiob.pb8.into_open_drain_output();
        let sda = gpiob.pb9.into_open_drain_output();

        let i2c_bus = {
            let i2c = I2c::i2c1(cx.device.I2C1, (scl, sda), 100.khz(), &mut rcc);
            let bus = shared_bus::BusManager::<
                cortex_m::interrupt::Mutex<RefCell<TypeI2cBus>>,
                TypeI2cBus,
            >::new(i2c);

            unsafe {
                I2C_BUS = Some(bus);
                // This reference is now &'static
                &I2C_BUS.as_ref().unwrap()
            }
        };

        /* init IR temp sensor */

        let temp =
            Mlx9061x::new_mlx90614(i2c_bus.acquire(), mlx9061x::SlaveAddr::default(), 5).unwrap();
        let mut temp_en = gpioa.pa14.into_push_pull_output();
        temp_en.set_low().unwrap();

        /* init AT24 EEPROM */

        let eeprom = Eeprom24x::new_24xm01(i2c_bus.acquire(), eeprom24x::SlaveAddr::default());
        let mut eeprom_wp = gpiob.pb5.into_push_pull_output();
        let tmr4 = cx.device.TIM4.timer(1.mhz(), &mut rcc);
        let eeprom_tmr = DelayTimer { timer: tmr4 };

        eeprom_wp.set_high().unwrap();

        /* initial state */

        let state = State::Select(Menu::Shot);

        lcd.clear();
        lcd.set_cursor_pos(0);
        lcd.write_fmt(format_args!("{}", state)).unwrap();

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
            temp,
            temp_en,
            eeprom,
            eeprom_wp,
            eeprom_tmr,
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

    #[task(resources = [queue])]
    fn cont_task(cx: cont_task::Context) {
        cx.resources.queue.enqueue(Event::Repeat).ok();
    }

    #[task(schedule = [proc_task, cont_task], resources = [queue, state, pos, lcd, temp, temp_en, eeprom, eeprom_wp, eeprom_tmr])]
    fn proc_task(cx: proc_task::Context) {
        let eeprom_tmr = cx.resources.eeprom_tmr;
        let eeprom_wp = cx.resources.eeprom_wp;
        let eeprom = cx.resources.eeprom;
        let state = cx.resources.state;
        let queue = cx.resources.queue;
        let temp = cx.resources.temp;
        let lcd = cx.resources.lcd;
        let pos = cx.resources.pos;

        let mut val: Option<f32> = None;

        while !queue.is_empty() {
            if let Some(e) = queue.dequeue() {
                match state {
                    State::Select(x) => match e {
                        Event::Button1 => *state = State::next(*state),
                        Event::Button2 => *state = State::prev(*state),
                        Event::Enter => {
                            queue.enqueue(Event::Repeat).ok();
                            *state = State::Active(*x);
                            *pos = None;
                        }
                        _ => {}
                    },
                    State::Active(Menu::Shot) => match e {
                        Event::Enter => *state = State::Select(Menu::Shot),
                        Event::Button1 | Event::Button2 => {
                            if let Ok(t) = temp.object1_temperature() {
                                val = Some(t);
                            }
                        }
                        _ => {}
                    },
                    State::Active(Menu::Cont) => match e {
                        Event::Enter => *state = State::Select(Menu::Cont),
                        Event::Repeat => {
                            if let Ok(t) = temp.object1_temperature() {
                                val = Some(t);
                            }
                            cx.schedule
                                .cont_task(Instant::now() + CONT_PERIOD.cycles())
                                .ok();
                        }
                        _ => {}
                    },
                    State::Active(Menu::ShotMem) => match e {
                        Event::Enter => {
                            // store number of written measurements

                            let p = match *pos {
                                Some(x) => x,
                                None => 0u32,
                            };

                            eeprom_wp.set_low().unwrap();
                            eeprom.write_page(0, &p.to_le_bytes()).unwrap();
                            eeprom_tmr.delay_ms(5);
                            eeprom_wp.set_high().unwrap();

                            *state = State::Select(Menu::ShotMem);
                            *pos = None;
                        }

                        Event::Button1 | Event::Button2 => {
                            if let Ok(t) = temp.object1_temperature() {
                                let p = match *pos {
                                    Some(x) => x + 1,
                                    None => 1u32,
                                };

                                eeprom_wp.set_low().unwrap();
                                eeprom.write_page(p * 4, &t.to_le_bytes()).unwrap();
                                eeprom_tmr.delay_ms(5);
                                eeprom_wp.set_high().unwrap();

                                *pos = Some(p);
                                val = Some(t);
                            }
                        }
                        _ => {}
                    },

                    State::Active(Menu::ContMem) => match e {
                        Event::Enter => {
                            // store number of written measurements
                            let p = match *pos {
                                Some(p) => p,
                                None => 0u32,
                            };

                            eeprom_wp.set_low().unwrap();
                            eeprom.write_page(0x0, &p.to_le_bytes()).unwrap();
                            eeprom_tmr.delay_ms(5);
                            eeprom_wp.set_high().unwrap();

                            *state = State::Select(Menu::ContMem);
                            *pos = None;
                        }
                        Event::Repeat => {
                            if let Ok(t) = temp.object1_temperature() {
                                let p = match *pos {
                                    Some(x) => x + 1,
                                    None => 1u32,
                                };

                                eeprom_wp.set_low().unwrap();
                                eeprom.write_page(p * 4, &t.to_le_bytes()).unwrap();
                                eeprom_tmr.delay_ms(5);
                                eeprom_wp.set_high().unwrap();

                                *pos = Some(p);
                                val = Some(t);
                            }

                            cx.schedule
                                .cont_task(Instant::now() + CONT_PERIOD.cycles())
                                .ok();
                        }
                        _ => {}
                    },
                    State::Active(Menu::View) => match e {
                        Event::Enter => {
                            *state = State::Select(Menu::View);
                            *pos = None;
                        }
                        Event::Button1 | Event::Button2 => {
                            let mut data: [u8; 4] = [0; 4];
                            let max: u32;

                            eeprom.read_data(0, &mut data).unwrap();
                            max = u32::from_le_bytes(data);

                            let p = match *pos {
                                Some(x) if x < max => x + 1,
                                _ => 1u32,
                            };

                            eeprom.read_data(4 * p, &mut data).unwrap();
                            val = Some(f32::from_le_bytes(data));
                            *pos = Some(p);
                        }
                        _ => {}
                    },
                    State::Active(Menu::Stream) => match e {
                        Event::Enter => {
                            *state = State::Select(Menu::Stream);
                            *pos = None;
                        }
                        Event::Repeat => {
                            let mut data: [u8; 4] = [0; 4];
                            let max: u32;

                            eeprom.read_data(0, &mut data).unwrap();
                            max = u32::from_le_bytes(data);

                            let p = match *pos {
                                Some(x) if x < max => x + 1,
                                _ => 1u32,
                            };

                            eeprom.read_data(4 * p, &mut data).unwrap();
                            val = Some(f32::from_le_bytes(data));
                            *pos = Some(p);

                            cx.schedule
                                .cont_task(Instant::now() + CONT_PERIOD.cycles())
                                .ok();
                        }
                        _ => {}
                    },
                }
            }

            match state {
                State::Select(_) => {
                    lcd.clear();
                    lcd.set_cursor_pos(0);
                    lcd.write_fmt(format_args!("{}", state)).unwrap();
                }
                State::Active(_) => {
                    lcd.clear();
                    lcd.set_cursor_pos(0);
                    match *pos {
                        Some(p) => {
                            lcd.write_fmt(format_args!("{}:{:05}", state, p)).unwrap();
                        }
                        None => {
                            lcd.write_fmt(format_args!("{}", state)).unwrap();
                        }
                    }
                    lcd.set_cursor_pos(40);
                    match val {
                        Some(v) => {
                            lcd.write_fmt(format_args!("{:.2}", v)).unwrap();
                        }
                        None => {
                            lcd.write_fmt(format_args!("---")).unwrap();
                        }
                    }
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
