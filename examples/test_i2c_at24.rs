#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(feature = "i2c_bb")]
use bitbang_hal;

use cortex_m as cm;
use cortex_m_rt as rt;
use eeprom24x;
use embedded_hal::digital::v2::OutputPin;
use stm32l1xx_hal as hal;

use cm::interrupt::Mutex;
use core::cell::RefCell;
use core::fmt::Write;
use core::ops::DerefMut;
use core::panic::PanicInfo;
use eeprom24x::Eeprom24x;
use eeprom24x::SlaveAddr;
use hal::prelude::*;
use hal::rcc::Config;
use hal::serial;
use hal::serial::SerialExt;
use hal::serial::Tx;
use hal::stm32;
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

    // init delay
    let mut delay = cp.SYST.delay(rcc.clocks);

    // init gpio for eeprom
    let gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8.into_open_drain_output();
    let sda = gpiob.pb9.into_open_drain_output();
    let mut wp = gpiob.pb5.into_push_pull_output();

    // init i2c: h/w (mcu) or s/w (bitbang)
    #[cfg(feature = "i2c_hw")]
    let i2c = dp.I2C1.i2c((scl, sda), 100.khz(), &mut rcc);
    #[cfg(feature = "i2c_bb")]
    let i2c = {
        let tmr = dp.TIM2.timer(200.khz(), &mut rcc);
        bitbang_hal::i2c::I2cBB::new(scl, sda, tmr)
    };

    // init eeprom24x
    let mut eeprom = Eeprom24x::new_24xm01(i2c, SlaveAddr::default());

    loop {
        wp.set_high().unwrap();

        for addr in 0..=15 {
            match eeprom.read_byte(addr as u32) {
                Ok(byte) => cm::interrupt::free(|cs| {
                    if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
                        tx.write_fmt(format_args!("read #1 from addr {} : {}\r\n", addr, byte))
                            .unwrap();
                    }
                }),
                Err(err) => cm::interrupt::free(|cs| {
                    if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
                        tx.write_fmt(format_args!("read #1 err: {:?}\r\n", err))
                            .unwrap();
                    }
                }),
            };

            delay.delay(10.ms());
        }

        wp.set_low().unwrap();

        for addr in 0..=15 {
            match eeprom.write_byte(addr as u32, addr) {
                Ok(_) => cm::interrupt::free(|cs| {
                    if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
                        tx.write_fmt(format_args!("write #1 {} to addr {}\r\n", addr, addr))
                            .unwrap();
                    }
                }),
                Err(err) => cm::interrupt::free(|cs| {
                    if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
                        tx.write_fmt(format_args!("write #1 err: {:?}\r\n", err))
                            .unwrap();
                    }
                }),
            }

            delay.delay(10.ms());
        }

        wp.set_high().unwrap();

        for addr in 0..=15 {
            match eeprom.read_byte(addr as u32) {
                Ok(byte) => cm::interrupt::free(|cs| {
                    if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
                        tx.write_fmt(format_args!("read #2 from addr {} : {}\r\n", addr, byte))
                            .unwrap();
                    }
                }),
                Err(err) => cm::interrupt::free(|cs| {
                    if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
                        tx.write_fmt(format_args!("read #2 err: {:?}\r\n", err))
                            .unwrap();
                    }
                }),
            }

            delay.delay(10.ms());
        }

        wp.set_low().unwrap();

        for addr in 0..=15 {
            match eeprom.write_byte(addr as u32, 15 - addr) {
                Ok(_) => cm::interrupt::free(|cs| {
                    if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
                        tx.write_fmt(format_args!("write #2 {} to addr {}\r\n", 15 - addr, addr))
                            .unwrap();
                    }
                }),
                Err(err) => cm::interrupt::free(|cs| {
                    if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
                        tx.write_fmt(format_args!("write #2 err: {:?}\r\n", err))
                            .unwrap();
                    }
                }),
            }

            delay.delay(10.ms());
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
