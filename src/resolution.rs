use core::time::Duration;

use embedded_hal_async::delay::DelayNs;

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum Resolution {
    Bits9 = 0b00011111,
    Bits10 = 0b00111111,
    Bits11 = 0b01011111,
    Bits12 = 0b01111111,
}

impl Resolution {
    pub fn max_measurement_time_millis(&self) -> Duration {
        match self {
            Resolution::Bits9 => Duration::from_millis(94),
            Resolution::Bits10 => Duration::from_millis(188),
            Resolution::Bits11 => Duration::from_millis(375),
            Resolution::Bits12 => Duration::from_millis(750),
        }
    }

    /// Waits for the amount of time required to finish measuring temperature
    /// using this resolution.
    pub async fn delay_for_measurement_time(&self, delay: &mut impl DelayNs) {
        delay.delay_ms(self.max_measurement_time_millis().as_millis() as u32).await;
    }

    pub(crate) fn from_config_register(config: u8) -> Option<Resolution> {
        match config {
            0b00011111 => Some(Resolution::Bits9),
            0b00111111 => Some(Resolution::Bits10),
            0b01011111 => Some(Resolution::Bits11),
            0b01111111 => Some(Resolution::Bits12),
            _ => None,
        }
    }

    pub(crate) fn to_config_register(self) -> u8 {
        self as u8
    }
}
