use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::timer::CountDown;
use nb::block;
use stm32l1xx_hal::time::Hertz;
use stm32l1xx_hal::time::U32Ext;

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
