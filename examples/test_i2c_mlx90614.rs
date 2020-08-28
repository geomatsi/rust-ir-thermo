#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(feature = "i2c_bb")]
use bitbang_hal;

use cortex_m as cm;
use cortex_m_rt as rt;
use embedded_hal::digital::v2::OutputPin;
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
use mlx9061x::Mlx9061x;
use mlx9061x::SlaveAddr;
use rt::entry;

static G_DBG_TX: Mutex<RefCell<Option<Tx<stm32::USART1>>>> = Mutex::new(RefCell::new(None));

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
    let (tx, _) = serial.split();

    cm::interrupt::free(|cs| {
        G_DBG_TX.borrow(cs).replace(Some(tx));
    });

    // init gpio for i2c
    let gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8.into_open_drain_output();
    let sda = gpiob.pb9.into_open_drain_output();

    // init i2c: h/w (mcu) or s/w (bitbang)
    #[cfg(feature = "i2c_hw")]
    let i2c = dp.I2C1.i2c((scl, sda), 100.khz(), &mut rcc);
    #[cfg(feature = "i2c_bb")]
    let i2c = {
        let tmr = dp.TIM2.timer(200.khz(), &mut rcc);
        bitbang_hal::i2c::I2cBB::new(scl, sda, tmr)
    };

    // init temperature sensor
    let mut temp = Mlx9061x::new_mlx90614(i2c, SlaveAddr::default(), 5).unwrap();

    // EN gpio for MLX sensor
    let mut en = gpioa.pa14.into_push_pull_output();
    en.set_low().unwrap();

    loop {
        match temp.object1_temperature() {
            Ok(byte) => cm::interrupt::free(|cs| {
                if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
                    tx.write_fmt(format_args!("object temperature: {:.2}\r\n", byte))
                        .unwrap();
                }
            }),
            Err(err) => cm::interrupt::free(|cs| {
                if let Some(ref mut tx) = G_DBG_TX.borrow(cs).borrow_mut().deref_mut() {
                    tx.write_fmt(format_args!("read err: {:?}\r\n", err))
                        .unwrap();
                }
            }),
        };

        delay(500_000);
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

fn delay(count: u32) {
    for _ in 0..count {
        cm::asm::nop();
    }
}
