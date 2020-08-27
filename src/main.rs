#![deny(warnings)]
#![no_main]
#![no_std]

use core::fmt::Write;
use core::panic::PanicInfo;

use embedded_hal::digital::v1_compat::OldOutputPin;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;

use hd44780_driver::bus::FourBitBus;
use hd44780_driver::HD44780;

use eeprom24x::addr_size::TwoBytes;
use eeprom24x::page_size::B256;
use eeprom24x::Eeprom24x;

use mlx9061x::ic::Mlx90614;
use mlx9061x::Mlx9061x;

use rtic::app;
use rtic::cyccnt::Instant;
use rtic::cyccnt::U32Ext;

use rust_ir_thermo::delay_timer::DelayTimer;
use rust_ir_thermo::event::Event;
use rust_ir_thermo::event::EventQueue;

use rust_ir_thermo::logger::u_init;
use rust_ir_thermo::state::Menu;
use rust_ir_thermo::state::State;
use rust_ir_thermo::u_println;

use shared_bus_rtic::SharedBus;

use core::f32::NAN;
use core::u32::MAX;
use stm32l1xx_hal::adc::{Adc, Align, Precision, SampleTime, VRef};
use stm32l1xx_hal::gpio::gpioa::{PA0, PA14, PA4, PA5, PA8};
use stm32l1xx_hal::gpio::gpiob::{PB0, PB1, PB5, PB7, PB8, PB9};
use stm32l1xx_hal::gpio::gpiob::{PB10, PB12, PB13, PB14, PB15};
use stm32l1xx_hal::gpio::{Analog, Floating, Input, OpenDrain, Output, PushPull};
use stm32l1xx_hal::i2c::I2c;
use stm32l1xx_hal::prelude::*;
use stm32l1xx_hal::rcc::Config;
use stm32l1xx_hal::rcc::HSI_FREQ;
use stm32l1xx_hal::serial;
use stm32l1xx_hal::serial::SerialExt;
use stm32l1xx_hal::stm32;
use stm32l1xx_hal::stm32::I2C1;
use stm32l1xx_hal::stm32::{TIM3, TIM4};
use stm32l1xx_hal::timer::Timer;
use stm32l1xx_hal::watchdog::IndependedWatchdog;

/* AT24CM01 => 131_072 8-bit words => 32_768 4-byte words
 * EEPROM layout:
 *  - 1st word (address 0x0) to store the number of measurements
 *  - 32_767 words to keep measurements
 */

const MAX_LEN: u32 = 32_767;
const LOW_BATTERY: f32 = 2.5;

/* Firmware uses HSI clock */

const VBAT_PERIOD: u32 = HSI_FREQ * 20; /* 20 sec */
const IDLE_PERIOD: u32 = HSI_FREQ * 10; /* 10 sec */
const CONT_PERIOD: u32 = HSI_FREQ; /* 1 sec */
const IWDG_PERIOD: u32 = HSI_FREQ / 2; /* 0.5 sec */
const BEAT_PERIOD: u32 = HSI_FREQ / 2; /* 0.5 sec */
const PROC_PERIOD: u32 = HSI_FREQ / 4; /* 0.25 sec */

/* types */

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

/* device aggregates: multiple peripherals to control one device */

pub struct I2cTemp<TypeI2cBus: 'static> {
    i2c: Mlx9061x<SharedBus<TypeI2cBus>, Mlx90614>,
    pwr: PA14<Output<PushPull>>,
}

pub struct I2cEeprom<TypeI2cBus: 'static> {
    i2c: Eeprom24x<SharedBus<TypeI2cBus>, B256, TwoBytes>,
    tmr: DelayTimer<Timer<TIM4>>,
    wp: PB5<Output<PushPull>>,
}

pub struct LcdDev {
    bus: LcdType,
    pwr: PB7<Output<PushPull>>,
}

/*
 * shared-bus-rtic aggregate: multiple peripherals on a single i2c bus
 *
 * According to shared-bus-rtic docs:
 * Note that all of the drivers that use the same underlying bus **must** be stored within a single
 * resource (e.g. as one larger `struct`) within the RTIC resources. This ensures that RTIC will
 * prevent one driver from interrupting another while they are using the same underlying bus.
 */

pub struct I2cDev<TypeI2cBus: 'static> {
    temp: I2cTemp<TypeI2cBus>,
    eeprom: I2cEeprom<TypeI2cBus>,
}

/* */

#[app(device = stm32l1xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // resources
        #[init(false)]
        low_battery: bool,
        #[init(0)]
        cb1: u8,
        #[init(0)]
        cb2: u8,
        #[init(None)]
        pos: Option<u32>,
        #[init(None)]
        max: Option<u32>,

        // late resources
        adc: Adc,
        vref: VRef,
        chan: PA0<Analog>,
        wdg: IndependedWatchdog,
        heartbeat_led: PB0<Output<PushPull>>,
        low_vbat_led: PB1<Output<PushPull>>,
        button1: PA4<Input<Floating>>,
        button2: PA5<Input<Floating>>,
        tmr2: Timer<stm32::TIM2>,
        queue: EventQueue,
        state: State,
        lcd_dev: LcdDev,
        i2c_dev: I2cDev<TypeI2cBus>,
    }

    #[init(schedule = [beat_task, proc_task, sleep_task, wdg_task, vbat_task])]
    fn init(mut cx: init::Context) -> init::LateResources {
        /* init hardware */

        let mut rcc = cx.device.RCC.freeze(Config::hsi());
        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();

        /* init serial logger */

        let tx = gpioa.pa9;
        let rx = gpioa.pa10;
        let cfg = serial::Config {
            baudrate: 115_200_u32.bps(),
            wordlength: serial::WordLength::DataBits9,
            parity: serial::Parity::ParityOdd,
            stopbits: serial::StopBits::STOP1,
        };

        let serial = cx.device.USART1.usart((tx, rx), cfg, &mut rcc).unwrap();
        let (tx, _) = serial.split();

        u_init(tx);

        /* init adc */

        let mut adc = cx.device.ADC.adc(&mut rcc);
        let chan = gpioa.pa0.into_analog();
        let mut vref = VRef::new();

        adc.set_align(Align::Right);
        adc.set_precision(Precision::B_12);
        adc.set_sample_time(SampleTime::T_16);

        vref.enable(&mut adc);

        /* init watchdog */

        let mut wdg = cx.device.IWDG.watchdog();
        wdg.start(1000.ms());

        /* init buttons */

        let mut tmr2 = cx.device.TIM2.timer(10.hz(), &mut rcc);
        tmr2.listen();

        let button1 = gpioa.pa4.into_floating_input();
        let button2 = gpioa.pa5.into_floating_input();

        /* init LEDs */

        let mut heartbeat_led = gpiob.pb0.into_push_pull_output();
        let mut low_vbat_led = gpiob.pb1.into_push_pull_output();

        heartbeat_led.set_high().unwrap();
        low_vbat_led.set_high().unwrap();

        /* enable monotonic timer */

        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        /* init LCD */

        let tmr3 = cx.device.TIM3.timer(1.mhz(), &mut rcc);
        let delay = DelayTimer::new(tmr3);

        let mut vo = gpiob.pb6.into_push_pull_output();
        let mut rw = gpiob.pb11.into_push_pull_output();
        let mut lcd_pwr = gpiob.pb7.into_push_pull_output();

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

        lcd_pwr.set_high().unwrap();
        vo.set_low().unwrap();
        rw.set_low().unwrap();

        lcd.reset();
        lcd.clear();
        lcd.set_display_mode(hd44780_driver::DisplayMode {
            display: hd44780_driver::Display::On,
            cursor_visibility: hd44780_driver::Cursor::Invisible,
            cursor_blink: hd44780_driver::CursorBlink::Off,
        });

        /* init I2C shared bus */

        let scl = gpiob.pb8.into_open_drain_output();
        let sda = gpiob.pb9.into_open_drain_output();
        let i2c = I2c::i2c1(cx.device.I2C1, (scl, sda), 100.khz(), &mut rcc);
        let manager = shared_bus_rtic::new!(i2c, TypeI2cBus);

        /* init IR temp sensor */

        let temp =
            Mlx9061x::new_mlx90614(manager.acquire(), mlx9061x::SlaveAddr::default(), 5).unwrap();
        let mut temp_pwr = gpioa.pa14.into_push_pull_output();

        temp_pwr.set_low().unwrap();

        /* init AT24 EEPROM */

        let eeprom = Eeprom24x::new_24xm01(manager.acquire(), eeprom24x::SlaveAddr::default());
        let mut eeprom_wp = gpiob.pb5.into_push_pull_output();
        let tmr4 = cx.device.TIM4.timer(1.mhz(), &mut rcc);
        let eeprom_tmr = DelayTimer::new(tmr4);

        eeprom_wp.set_high().unwrap();

        /* initial state */

        let state = State::Select(Menu::Shot);

        lcd.clear();
        lcd.set_cursor_pos(0);
        lcd.write_fmt(format_args!("{}", state)).unwrap();

        /* priority queue */

        let queue = EventQueue::new();

        /* schedule tasks */

        cx.schedule
            .wdg_task(Instant::now() + IWDG_PERIOD.cycles())
            .unwrap();

        cx.schedule
            .beat_task(Instant::now() + BEAT_PERIOD.cycles())
            .unwrap();

        cx.schedule
            .vbat_task(Instant::now() + VBAT_PERIOD.cycles())
            .unwrap();

        cx.schedule
            .sleep_task(Instant::now() + IDLE_PERIOD.cycles())
            .unwrap();

        cx.schedule.proc_task(Instant::now()).unwrap();

        /* init late resources */

        init::LateResources {
            wdg,
            adc,
            chan,
            vref,
            state,
            queue,
            heartbeat_led,
            low_vbat_led,
            tmr2,
            button1,
            button2,
            lcd_dev: LcdDev {
                bus: lcd,
                pwr: lcd_pwr,
            },
            i2c_dev: I2cDev {
                temp: I2cTemp {
                    i2c: temp,
                    pwr: temp_pwr,
                },
                eeprom: I2cEeprom {
                    i2c: eeprom,
                    tmr: eeprom_tmr,
                    wp: eeprom_wp,
                },
            },
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = TIM2, resources = [tmr2, queue, button1, cb1, button2, cb2])]
    fn tim2(cx: tim2::Context) {
        if cx.resources.button1.is_low().unwrap() {
            *cx.resources.cb1 += 1;
        } else {
            match *cx.resources.cb1 {
                x if 1 <= x && x <= 3 => {
                    cx.resources.queue.enqueue(Event::Button1);
                    *cx.resources.cb1 = 0;
                }
                x if x > 3 => {
                    cx.resources.queue.enqueue(Event::Enter);
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
                    cx.resources.queue.enqueue(Event::Button2);
                    *cx.resources.cb2 = 0;
                }
                x if x > 3 => {
                    cx.resources.queue.enqueue(Event::Enter);
                    *cx.resources.cb2 = 0;
                }
                _ => {
                    *cx.resources.cb2 = 0;
                }
            }
        }

        cx.resources.tmr2.clear_irq();
    }

    #[task(schedule = [beat_task], resources = [state, heartbeat_led, low_battery, low_vbat_led])]
    fn beat_task(cx: beat_task::Context) {
        let low_battery = cx.resources.low_battery;
        let green_led = cx.resources.heartbeat_led;
        let red_led = cx.resources.low_vbat_led;
        let state = cx.resources.state;

        if *state != State::Idle {
            green_led.toggle().unwrap();
        }

        if *low_battery {
            red_led.toggle().unwrap();
        }

        cx.schedule
            .beat_task(cx.scheduled + BEAT_PERIOD.cycles())
            .unwrap();
    }

    #[task(schedule = [vbat_task], resources = [low_battery, low_vbat_led, adc, chan, vref])]
    fn vbat_task(cx: vbat_task::Context) {
        let low_battery = cx.resources.low_battery;
        let red_led = cx.resources.low_vbat_led;
        let chan = cx.resources.chan;
        let vref = cx.resources.vref;
        let adc = cx.resources.adc;

        if let Ok(u) = adc.read(chan) as Result<u16, _> {
            if let Ok(v) = adc.read(vref) as Result<u16, _> {
                let vrefcal = VRef::get_vrefcal() as f32;
                // ADC is in 12-bit mode
                let scale = 4095_f32;
                // See TRM (RM0038): chapter 12.12
                let mut vbat: f32 = 3.0 * vrefcal * u as f32 / v as f32 / scale;
                // Schematics: voltage divider
                vbat *= 2.0;

                match vbat {
                    _ if vbat < LOW_BATTERY => {
                        red_led.set_low().unwrap();
                        *low_battery = true;
                    }
                    _ if vbat > LOW_BATTERY * 1.1 => {
                        red_led.set_high().unwrap();
                        *low_battery = false;
                    }
                    _ => { /* keep current behavior */ }
                }
            }
        }

        cx.schedule
            .vbat_task(cx.scheduled + VBAT_PERIOD.cycles())
            .unwrap();
    }

    #[task(schedule = [wdg_task], resources = [wdg])]
    fn wdg_task(cx: wdg_task::Context) {
        cx.resources.wdg.feed();

        cx.schedule
            .wdg_task(cx.scheduled + IWDG_PERIOD.cycles())
            .unwrap();
    }

    #[task(schedule = [sleep_task], resources = [queue, state, lcd_dev, i2c_dev, heartbeat_led])]
    fn sleep_task(cx: sleep_task::Context) {
        let temp = &mut cx.resources.i2c_dev.temp;
        let green_led = cx.resources.heartbeat_led;
        let lcd = cx.resources.lcd_dev;
        let state = cx.resources.state;
        let queue = cx.resources.queue;

        if queue.get_stats() > 0 {
            cx.schedule
                .sleep_task(cx.scheduled + IDLE_PERIOD.cycles())
                .unwrap();
            queue.reset_stats();
            return;
        }

        // enter idle mode: disable peripherals
        green_led.set_high().unwrap();
        temp.pwr.set_high().unwrap();
        lcd.pwr.set_low().unwrap();

        *state = State::Idle;
    }

    #[task(resources = [queue])]
    fn cont_task(cx: cont_task::Context) {
        cx.resources.queue.enqueue(Event::Repeat);
    }

    #[task(schedule = [proc_task, cont_task, sleep_task], resources = [queue, state, adc, chan, vref, pos, max, lcd_dev, i2c_dev])]
    fn proc_task(cx: proc_task::Context) {
        let eeprom = &mut cx.resources.i2c_dev.eeprom;
        let temp = &mut cx.resources.i2c_dev.temp;
        let lcd = cx.resources.lcd_dev;
        let state = cx.resources.state;
        let queue = cx.resources.queue;
        let chan = cx.resources.chan;
        let vref = cx.resources.vref;
        let adc = cx.resources.adc;
        let pos = cx.resources.pos;
        let max = cx.resources.max;

        let mut val: Option<f32> = None;

        while !queue.is_empty() {
            if let Some(e) = queue.dequeue() {
                match state {
                    State::Idle => {
                        cx.schedule
                            .sleep_task(cx.scheduled + IDLE_PERIOD.cycles())
                            .unwrap();
                        *state = State::Select(Menu::Shot);
                        queue.enqueue(Event::Repeat);

                        // exit idle mode: re-enable peripherals
                        temp.pwr.set_low().unwrap();
                        lcd.pwr.set_high().unwrap();
                    }
                    State::Select(x) => match e {
                        Event::Button1 => *state = State::next(*state),
                        Event::Button2 => *state = State::prev(*state),
                        Event::Enter => {
                            queue.enqueue(Event::Repeat);
                            *state = State::Active(*x);
                            *pos = None;
                            *max = None;
                        }
                        _ => {}
                    },
                    State::Active(Menu::Shot) => match e {
                        Event::Enter => *state = State::Select(Menu::Shot),
                        Event::Button1 | Event::Button2 | Event::Repeat => {
                            if let Ok(t) = temp.i2c.object1_temperature() {
                                val = Some(t);
                            }
                        }
                    },
                    State::Active(Menu::Cont) => match e {
                        Event::Enter => *state = State::Select(Menu::Cont),
                        Event::Repeat => {
                            if let Ok(t) = temp.i2c.object1_temperature() {
                                val = Some(t);
                            }
                            cx.schedule
                                .cont_task(cx.scheduled + CONT_PERIOD.cycles())
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

                            eeprom.wp.set_low().unwrap();
                            eeprom.tmr.delay_ms(2);
                            eeprom.i2c.write_page(0, &p.to_le_bytes()).unwrap();
                            eeprom.tmr.delay_ms(5);
                            eeprom.wp.set_high().unwrap();

                            *state = State::Select(Menu::ShotMem);
                            *pos = None;
                        }

                        Event::Button1 | Event::Button2 => {
                            if let Ok(t) = temp.i2c.object1_temperature() {
                                let p = match *pos {
                                    Some(x) if x < MAX_LEN => x + 1,
                                    _ => 1u32,
                                };

                                eeprom.wp.set_low().unwrap();
                                eeprom.tmr.delay_ms(2);
                                eeprom.i2c.write_page(p * 4, &t.to_le_bytes()).unwrap();
                                eeprom.tmr.delay_ms(5);
                                eeprom.wp.set_high().unwrap();

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
                                Some(x) => x,
                                None => 0u32,
                            };

                            eeprom.wp.set_low().unwrap();
                            eeprom.tmr.delay_ms(2);
                            eeprom.i2c.write_page(0, &p.to_le_bytes()).unwrap();
                            eeprom.tmr.delay_ms(5);
                            eeprom.wp.set_high().unwrap();

                            *state = State::Select(Menu::ContMem);
                            *pos = None;
                        }
                        Event::Repeat => {
                            if let Ok(t) = temp.i2c.object1_temperature() {
                                let p = match *pos {
                                    Some(x) if x < MAX_LEN => x + 1,
                                    _ => 1u32,
                                };

                                eeprom.wp.set_low().unwrap();
                                eeprom.tmr.delay_ms(2);
                                eeprom.i2c.write_page(p * 4, &t.to_le_bytes()).unwrap();
                                eeprom.tmr.delay_ms(5);
                                eeprom.wp.set_high().unwrap();

                                *pos = Some(p);
                                val = Some(t);
                            }

                            cx.schedule
                                .cont_task(cx.scheduled + CONT_PERIOD.cycles())
                                .ok();
                        }
                        _ => {}
                    },
                    State::Active(Menu::View) => match e {
                        Event::Enter => {
                            *state = State::Select(Menu::View);
                            *pos = None;
                            *max = None;
                        }
                        Event::Button1 | Event::Button2 => {
                            let mut data: [u8; 4] = [0; 4];

                            let m = match *max {
                                None => {
                                    eeprom.i2c.read_data(0, &mut data).unwrap();
                                    eeprom.tmr.delay_ms(5);
                                    u32::from_le_bytes(data)
                                }
                                Some(x) => x,
                            };

                            let p = match *pos {
                                Some(x) if x < m => x + 1,
                                _ => 1u32,
                            };

                            eeprom.i2c.read_data(4 * p, &mut data).unwrap();
                            val = Some(f32::from_le_bytes(data));
                            *pos = Some(p);

                            u_println!("{:05}:{:.2}", pos.unwrap_or(MAX), val.unwrap_or(NAN));
                        }
                        _ => {}
                    },
                    State::Active(Menu::Stream) => match e {
                        Event::Enter => {
                            *state = State::Select(Menu::Stream);
                            *pos = None;
                            *max = None;
                        }
                        Event::Repeat => {
                            let mut data: [u8; 4] = [0; 4];

                            let m = match *max {
                                None => {
                                    eeprom.i2c.read_data(0, &mut data).unwrap();
                                    eeprom.tmr.delay_ms(5);
                                    u32::from_le_bytes(data)
                                }
                                Some(x) => x,
                            };

                            let p = match *pos {
                                Some(x) if x < m => x + 1,
                                _ => 1u32,
                            };

                            eeprom.i2c.read_data(4 * p, &mut data).unwrap();
                            val = Some(f32::from_le_bytes(data));
                            *pos = Some(p);

                            u_println!("{:05}:{:.2}", pos.unwrap_or(MAX), val.unwrap_or(NAN));

                            cx.schedule
                                .cont_task(cx.scheduled + CONT_PERIOD.cycles())
                                .ok();
                        }
                        _ => {}
                    },
                    State::Active(Menu::Battery) => match e {
                        Event::Enter => *state = State::Select(Menu::Battery),
                        Event::Repeat => {
                            // first read: warm-up
                            (adc.read(chan) as Result<u16, _>).ok();
                            (adc.read(vref) as Result<u16, _>).ok();
                        }
                        Event::Button1 | Event::Button2 => {
                            if let Ok(u) = adc.read(chan) as Result<u16, _> {
                                if let Ok(v) = adc.read(vref) as Result<u16, _> {
                                    let vrefcal = VRef::get_vrefcal() as f32;
                                    // ADC is in 12-bit mode
                                    let scale = 4095_f32;
                                    // See TRM (RM0038): chapter 12.12
                                    let vbat: f32 = 3.0 * vrefcal * u as f32 / v as f32 / scale;
                                    // Schematics: voltage divider
                                    val = Some(vbat * 2.0);
                                }
                            }
                        }
                    },
                }
            }

            match state {
                State::Select(_) => {
                    lcd.bus.clear();
                    lcd.bus.set_cursor_pos(0);
                    lcd.bus.write_fmt(format_args!("{}", state)).unwrap();
                }
                State::Active(_) => {
                    lcd.bus.clear();
                    lcd.bus.set_cursor_pos(0);
                    match *pos {
                        Some(p) => {
                            lcd.bus
                                .write_fmt(format_args!("{}:{:05}", state, p))
                                .unwrap();
                        }
                        None => {
                            lcd.bus.write_fmt(format_args!("{}", state)).unwrap();
                        }
                    }
                    lcd.bus.set_cursor_pos(40);
                    match val {
                        Some(v) => {
                            lcd.bus.write_fmt(format_args!("{:.2}", v)).unwrap();
                        }
                        None => {
                            lcd.bus.write_fmt(format_args!("---")).unwrap();
                        }
                    }
                }
                State::Idle => {}
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
fn panic(info: &PanicInfo) -> ! {
    u_println!("{}", info);
    loop {
        cortex_m::asm::nop();
    }
}
