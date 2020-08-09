#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m as cm;
use cortex_m_rt as rt;
use embedded_hal::digital::v1_compat::OldOutputPin;
use embedded_hal::digital::v2::OutputPin;
use hd44780_driver;
use stm32l1xx_hal as hal;

use cm::interrupt::Mutex;
use core::cell::RefCell;
use core::fmt::Write;
use core::ops::DerefMut;
use core::panic::PanicInfo;
use hal::prelude::*;
use hal::rcc::Config;
use hal::serial;
use hal::serial::SerialExt;
use hal::serial::Tx;
use hal::stm32;
use hd44780_driver::Cursor;
use hd44780_driver::CursorBlink;
use hd44780_driver::Display;
use hd44780_driver::DisplayMode;
use hd44780_driver::HD44780;
use rt::entry;

static G_DBG_TX: Mutex<RefCell<Option<Tx<stm32::USART1>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(Config::hsi());

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

    // init lcd
    let delay = cp.SYST.delay(rcc.clocks);
    let gpiob = dp.GPIOB.split();

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

    loop {
        lcd.clear();
        lcd.set_cursor_pos(0);
        lcd.write_str("HELLO").unwrap();
        for c in 0..=20 {
            lcd.set_cursor_pos(40);
            lcd.write_fmt(format_args!("ID: {:02}", c)).unwrap();
            sleep(500_000);
        }

        lcd.clear();
        lcd.set_cursor_pos(0);
        lcd.write_str("WORLD").unwrap();
        for c in (0..=20).rev() {
            lcd.set_cursor_pos(40);
            lcd.write_fmt(format_args!("ID: {:02}", c)).unwrap();
            sleep(500_000);
        }
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cm::interrupt::free(|cs| {
        if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
            tx.write_fmt(format_args!("{}", info)).unwrap();
        }
    });

    loop {}
}

fn sleep(count: u32) {
    for _ in 0..count {
        cm::asm::nop();
    }
}
