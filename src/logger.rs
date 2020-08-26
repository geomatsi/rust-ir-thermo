use core::cell::RefCell;
use core::fmt;
use core::fmt::Write;
use core::ops::DerefMut;
use cortex_m::interrupt::Mutex;
use stm32l1xx_hal::serial::Tx;
use stm32l1xx_hal::stm32;

static STDOUT: Mutex<RefCell<Option<Tx<stm32::USART1>>>> = Mutex::new(RefCell::new(None));

pub fn u_init(tx: Tx<stm32::USART1>) {
    cortex_m::interrupt::free(|cs| {
        STDOUT.borrow(cs).replace(Some(tx));
    });
}

pub fn u_str(s: &str) {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut tx) = STDOUT.borrow(cs).borrow_mut().deref_mut() {
            tx.write_str(s).ok();
        }
    });
}

pub fn u_fmt(args: fmt::Arguments) {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut tx) = STDOUT.borrow(cs).borrow_mut().deref_mut() {
            tx.write_fmt(args).ok();
        }
    });
}

#[macro_export]
macro_rules! u_println {
    () => {
        $crate::logger::u_str("\r\n")
    };
    ($s:expr) => {
        $crate::logger::u_str(concat!($s, "\r\n"))
    };
    ($s:expr, $($tt:tt)*) => {
        $crate::logger::u_fmt(format_args!(concat!($s, "\r\n"), $($tt)*))
    };
}
