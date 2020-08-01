#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

//use bitbang_hal;
use cortex_m as cm;
use cortex_m_rt as rt;
use eeprom24x;
use embedded_hal::digital::v2::OutputPin;
use stm32l1xx_hal as hal;

use core::fmt::Write;
use core::panic::PanicInfo;
use eeprom24x::Eeprom24x;
use eeprom24x::SlaveAddr;
use hal::prelude::*;
use hal::rcc::Config;
use hal::serial;
use hal::serial::SerialExt;
use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
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
    let (mut tx, _) = serial.split();

    // init gpio for i2c
    let gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8.into_open_drain_output();
    let sda = gpiob.pb9.into_open_drain_output();

    // init i2c: h/w (mcu) or s/w (bitbang)
    let i2c = dp.I2C1.i2c((scl, sda), 1.khz(), &mut rcc);
    //let tmr = dp.TIM2.timer(200.khz(), &mut rcc);
    //let i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, tmr);

    // init eeprom24x
    let mut eeprom = Eeprom24x::new_24xm01(i2c, SlaveAddr::default());

    // write-protect gpio for EEPROM
    let mut wp = gpiob.pb5.into_push_pull_output();

    loop {
        wp.set_high().unwrap();

        for addr in 0..=15 {
            match eeprom.read_byte(addr as u32) {
                Ok(byte) => tx
                    .write_fmt(format_args!("read #1 from addr {} : {}\r\n", addr, byte))
                    .unwrap(),
                Err(err) => tx
                    .write_fmt(format_args!("read #1 err: {:?}\r\n", err))
                    .unwrap(),
            };
            delay(500_000);
        }

        wp.set_low().unwrap();

        for addr in 0..=15 {
            match eeprom.write_byte(addr as u32, addr) {
                Ok(_) => tx
                    .write_fmt(format_args!("write #1 {} to addr {}\r\n", addr, addr))
                    .unwrap(),
                Err(err) => tx
                    .write_fmt(format_args!("write #1 err: {:?}\r\n", err))
                    .unwrap(),
            }
            delay(500_000);
        }

        wp.set_high().unwrap();

        for addr in 0..=15 {
            match eeprom.read_byte(addr as u32) {
                Ok(byte) => tx
                    .write_fmt(format_args!("read #2 from addr {} : {}\r\n", addr, byte))
                    .unwrap(),
                Err(err) => tx
                    .write_fmt(format_args!("read #2 err: {:?}\r\n", err))
                    .unwrap(),
            }
            delay(500_000);
        }

        wp.set_low().unwrap();

        for addr in 0..=15 {
            match eeprom.write_byte(addr as u32, 15 - addr) {
                Ok(_) => tx
                    .write_fmt(format_args!("write #2 {} to addr {}\r\n", 15 - addr, addr))
                    .unwrap(),
                Err(err) => tx
                    .write_fmt(format_args!("write #2 err: {:?}\r\n", err))
                    .unwrap(),
            }
            delay(500_000);
        }
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
