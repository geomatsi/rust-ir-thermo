#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m as cm;
use cortex_m_rt as rt;
use stm32l1xx_hal as hal;
use stm32l1xx_hal::serial::SerialExt;

use core::convert::Infallible;
use core::fmt::Write;
use core::panic::PanicInfo;
use hal::prelude::*;
use hal::rcc::Config;
use hal::serial;
use hal::serial::Tx;
use hal::stm32;
use hal::time::U32Ext;
use rt::entry;
use ufmt::uwrite;
use ufmt_write::uWrite;

struct TxWrapper {
    tx: Tx<hal::stm32::USART1>,
}

impl uWrite for TxWrapper {
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.tx.write_str(s).ok();
        Ok(())
    }
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(Config::hsi());

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
    let mut txw = TxWrapper { tx };

    loop {
        for i in 0..1000 {
            uwrite!(&mut txw, "test: {}\r\n", i).unwrap();
            delay(100_000);
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
