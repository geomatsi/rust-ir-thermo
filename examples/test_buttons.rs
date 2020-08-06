#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m as cm;
use cortex_m_rt as rt;
use embedded_hal::digital::v2::InputPin;
use stm32l1xx_hal as hal;

use core::cell::RefCell;
use core::fmt::Write;
use core::ops::DerefMut;
use core::panic::PanicInfo;
use cortex_m::interrupt::Mutex;
use hal::gpio::gpioa::PA4;
use hal::gpio::gpioa::PA5;
use hal::prelude::*;
use hal::rcc::Config;
use hal::serial;
use hal::serial::SerialExt;
use hal::serial::Tx;
use hal::stm32::{self, interrupt, Interrupt};
use hal::timer::Timer;
use rt::entry;
use stm32l1xx_hal::gpio::{Floating, Input};

static G_BUTTON_0: Mutex<RefCell<Option<PA4<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static G_BUTTON_1: Mutex<RefCell<Option<PA5<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static G_DBG_TX: Mutex<RefCell<Option<Tx<stm32::USART1>>>> = Mutex::new(RefCell::new(None));
static G_TIMER: Mutex<RefCell<Option<Timer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(Config::hsi());
    let nvic = &mut cp.NVIC;

    // init serial output
    let gpioa = dp.GPIOA.split();
    let tx = gpioa.pa9;
    let rx = gpioa.pa10;
    let cfg = serial::Config {
        baudrate: 115_200_u32.bps(),
        wordlength: serial::WordLength::DataBits9,
        parity: serial::Parity::ParityOdd,
        stopbits: serial::StopBits::STOP1,
    };
    let serial = dp.USART1.usart((tx, rx), cfg, &mut rcc).unwrap();
    let (tx, _) = serial.split();

    cm::interrupt::free(|cs| {
        G_DBG_TX.borrow(cs).replace(Some(tx));
    });

    // init button gpio
    let button0 = gpioa.pa4.into_floating_input();
    let button1 = gpioa.pa5.into_floating_input();

    cortex_m::interrupt::free(|cs| {
        G_BUTTON_0.borrow(cs).replace(Some(button0));
        G_BUTTON_1.borrow(cs).replace(Some(button1));
    });

    // init timer
    let mut timer = dp.TIM2.timer(10.hz(), &mut rcc);
    timer.listen();

    unsafe {
        nvic.set_priority(Interrupt::TIM2, 1);
        cm::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    cortex_m::interrupt::free(|cs| {
        G_TIMER.borrow(cs).replace(Some(timer));
    });

    loop {}
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cm::interrupt::free(|cs| {
        if let Some(mut tx) = G_DBG_TX.borrow(cs).replace(None) {
            tx.write_fmt(format_args!("{}\r\n", info)).unwrap();
        }
    });

    loop {}
}

#[interrupt]
fn TIM2() {
    static mut B0_COUNT: u32 = 0;
    static mut B1_COUNT: u32 = 0;

    cortex_m::interrupt::free(|cs| {
        if let (Some(ref mut tim), Some(ref mut dbg), Some(ref mut b0), Some(ref mut b1)) = (
            G_TIMER.borrow(cs).borrow_mut().deref_mut(),
            G_DBG_TX.borrow(cs).borrow_mut().deref_mut(),
            G_BUTTON_0.borrow(cs).borrow_mut().deref_mut(),
            G_BUTTON_1.borrow(cs).borrow_mut().deref_mut(),
        ) {
            if b0.is_low().unwrap() {
                *B0_COUNT += 1;

                if *B0_COUNT > 3 {
                    dbg.write_fmt(format_args!("button 0\r\n")).unwrap();
                    *B0_COUNT = 0;
                }
            }

            if b1.is_low().unwrap() {
                *B1_COUNT += 1;

                if *B1_COUNT > 3 {
                    dbg.write_fmt(format_args!("button 1\r\n")).unwrap();
                    *B1_COUNT = 0;
                }
            }

            tim.clear_irq();
        }
    });
}
