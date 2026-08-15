#![no_std]

use embedded_hal_async::delay::DelayNs;
use embedded_onewire::{
    OneWireAsync, OneWireCrc, OneWireError, OneWireResult, OneWireSearchAsync, OneWireSearchKind,
};

pub const FAMILY_CODE: u8 = 0x28;

pub mod commands;
mod resolution;

pub use resolution::Resolution;

/// Converts the raw temperature register to degrees Celsius.
///
/// The register is a **signed** two's-complement value (DS18B20 datasheet,
/// "Temperature Register Format") — bits above the resolution's data bits are
/// sign-extension bits, not part of an unsigned magnitude. Reading it as
/// unsigned turns any reading at or below 0 °C into a huge positive value:
/// raw `0xFFFF` is `-0.0625 °C` as signed (`-1 / 16.0`), but `4095.9375 °C` if
/// misread as unsigned `65535 / 16.0` — which a downstream `as i16` cast (a
/// saturating conversion, not a panic) then collapses to exactly `i16::MAX`
/// once scaled to centi-Celsius, indistinguishable from a real, if
/// implausible, reading.
///
/// The scale is **always** 1/16 °C per LSB, regardless of the configured
/// resolution -- this was previously a `match` dividing by 2/4/8/16 per
/// resolution, which is wrong. The datasheet's own worked examples ("Table 1.
/// Temperature/Data Relationship") use one constant scale throughout; the
/// resolution setting only changes which of the register's low-order bits
/// are guaranteed meaningful (masked to a fixed pattern at lower resolution,
/// for a faster conversion), never the register's place-value. This only
/// stayed hidden because nothing in this codebase had ever configured a
/// resolution other than the factory-default 12-bit until now: at 9-bit, the
/// old per-resolution divisor (÷2) produced real ~25 °C ambient readings as
/// ~204-212 °C -- confirmed live, raw register 408 at "9-bit" decoded as
/// 204.0 °C by the old code, and as 25.5 °C (matching ambient) once divided
/// by 16 like every other resolution.
fn raw_to_celsius(raw: i16, _resolution: Resolution) -> f32 {
    (raw as f32) / 16.0
}

/// 64-bit ROM address of a 1-Wire device.
///
/// Bits 0-7 contain the family code.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Address(pub u64);

impl Address {
    pub fn family_code(&self) -> u8 {
        (self.0 & 0xff) as u8
    }
}

/// Device entry in a DS18B20 chain.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Device {
    pub id: Address,
}

/// Temperature reading associated with a specific device id.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DeviceTemperature {
    pub id: Address,
    pub temperature: f32,
}

/// All of the data that can be read from the sensor.
#[derive(Debug)]
pub struct SensorData {
    /// Temperature in degrees Celsius. Defaults to 85 on startup.
    pub temperature: f32,
    /// The current resolution configuration.
    pub resolution: Resolution,
    /// If the last recorded temperature is lower than this, the sensor is put in an alarm state.
    pub alarm_temp_low: i8,
    /// If the last recorded temperature is higher than this, the sensor is put in an alarm state.
    pub alarm_temp_high: i8,
}

/// DS18B20 chain connected to a single 1-Wire bus.
/// Temperatures read from one chain: a fixed-capacity buffer of which the
/// first `len` entries are valid. Derefs to a slice of the valid entries.
#[derive(Clone, Copy, Debug)]
pub struct ChainReadings<const N: usize> {
    data: [DeviceTemperature; N],
    len: usize,
}

impl<const N: usize> ChainReadings<N> {
    pub fn as_slice(&self) -> &[DeviceTemperature] {
        &self.data[..self.len]
    }
}

impl<const N: usize> core::ops::Deref for ChainReadings<N> {
    type Target = [DeviceTemperature];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

pub struct Chain<O: OneWireAsync, const N: usize> {
    devices: [Device; N],
    /// Number of devices actually discovered on the bus (`<= N`).
    len: usize,
    onewire: O,
}

impl<O: OneWireAsync, const N: usize> Chain<O, N> {
    /// Initializes the chain by auto-discovering DS18B20 devices on the bus.
    ///
    /// `N` is the chain *capacity*: any number of devices up to `N` is
    /// accepted (including zero) and reported via [`Self::len`]. Discovering
    /// more than `N` devices is an error.
    pub async fn init(mut onewire: O) -> OneWireResult<Self, O::BusError> {
        let mut search =
            OneWireSearchAsync::with_family(&mut onewire, OneWireSearchKind::Normal, FAMILY_CODE);

        let mut devices = [Device { id: Address(0) }; N];
        let mut len = 0usize;
        while let Some(rom) = search.next().await? {
            let Some(device) = devices.get_mut(len) else {
                return Err(OneWireError::InvalidValue(
                    "found more DS18B20 devices than chain capacity",
                ));
            };
            device.id = Address(rom);
            len += 1;
        }

        Ok(Self {
            devices,
            len,
            onewire,
        })
    }

    /// Number of devices discovered on the bus.
    pub fn len(&self) -> usize {
        self.len
    }

    /// `true` when no devices were discovered.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn devices(&self) -> &[Device] {
        &self.devices[..self.len]
    }

    pub fn device_by_index(&self, index: usize) -> OneWireResult<Device, O::BusError> {
        let Some(device) = self.devices[..self.len].get(index).copied() else {
            return Err(OneWireError::InvalidValue("device index out of range"));
        };

        Ok(device)
    }

    pub fn device_by_address(&self, address: Address) -> OneWireResult<Device, O::BusError> {
        let Some(device) = self.devices[..self.len]
            .iter()
            .copied()
            .find(|d| d.id == address)
        else {
            return Err(OneWireError::InvalidValue("device address not in chain"));
        };

        Ok(device)
    }

    pub fn onewire_mut(&mut self) -> &mut O {
        &mut self.onewire
    }

    /// Starts a temperature measurement for one device in the chain.
    pub async fn start_temp_measurement(
        &mut self,
        device: Device,
    ) -> OneWireResult<(), O::BusError> {
        self.onewire.address(Some(device.id.0)).await?;
        self.onewire.write_byte(commands::CONVERT_TEMP).await?;
        Ok(())
    }

    /// Starts a temperature measurement for all devices on this chain simultaneously.
    pub async fn start_simultaneous_temp_measurement(&mut self) -> OneWireResult<(), O::BusError> {
        self.onewire.address(None).await?;
        self.onewire.write_byte(commands::CONVERT_TEMP).await?;
        Ok(())
    }

    pub async fn read_data(&mut self) -> OneWireResult<ChainReadings<N>, O::BusError> {
        self.read_chain_temperatures().await
    }

    async fn read_device_data(&mut self, device: Device) -> OneWireResult<SensorData, O::BusError> {
        let scratchpad = self.read_scratchpad(device).await?;

        let resolution = if let Some(resolution) = Resolution::from_config_register(scratchpad[4]) {
            resolution
        } else {
            return Err(OneWireError::InvalidValue("invalid config register"));
        };

        let raw_temp = i16::from_le_bytes([scratchpad[0], scratchpad[1]]);
        let temperature = raw_to_celsius(raw_temp, resolution);

        Ok(SensorData {
            temperature,
            resolution,
            alarm_temp_high: i8::from_le_bytes([scratchpad[2]]),
            alarm_temp_low: i8::from_le_bytes([scratchpad[3]]),
        })
    }

    pub async fn read_chain_temperatures(
        &mut self,
    ) -> OneWireResult<ChainReadings<N>, O::BusError> {
        let mut readings = ChainReadings {
            data: [DeviceTemperature {
                id: Address(0),
                temperature: 0.0,
            }; N],
            len: self.len,
        };

        let devices = self.devices;
        for (index, device) in devices[..self.len].iter().copied().enumerate() {
            let data = self.read_device_data(device).await?;
            readings.data[index] = DeviceTemperature {
                id: device.id,
                temperature: data.temperature,
            };
        }

        Ok(readings)
    }

    /// Returns `true` if at least one device on the chain is currently alarmed.
    ///
    /// This performs a single alarm-search step and does not enumerate all addresses.
    pub async fn any_device_alarmed(&mut self) -> OneWireResult<bool, O::BusError> {
        let mut search = OneWireSearchAsync::with_family(
            &mut self.onewire,
            OneWireSearchKind::Alarmed,
            FAMILY_CODE,
        );

        Ok(search.next().await?.is_some())
    }

    /// Returns all alarmed devices currently present on this chain.
    ///
    /// The returned array is compacted from index 0 and padded with `None`.
    pub async fn alarmed_devices(&mut self) -> OneWireResult<[Option<Device>; N], O::BusError> {
        let mut alarmed = [None; N];
        let mut next_slot = 0;

        let mut search = OneWireSearchAsync::with_family(
            &mut self.onewire,
            OneWireSearchKind::Alarmed,
            FAMILY_CODE,
        );

        while let Some(rom) = search.next().await? {
            let address = Address(rom);
            if let Some(device) = self.devices.iter().copied().find(|d| d.id == address) {
                if next_slot < N {
                    alarmed[next_slot] = Some(device);
                    next_slot += 1;
                }
            }
        }

        Ok(alarmed)
    }

    pub async fn set_config(
        &mut self,
        device: Device,
        alarm_temp_low: i8,
        alarm_temp_high: i8,
        resolution: Resolution,
    ) -> OneWireResult<(), O::BusError> {
        self.onewire.address(Some(device.id.0)).await?;
        self.onewire.write_byte(commands::WRITE_SCRATCHPAD).await?;
        self.onewire
            .write_byte(alarm_temp_high.to_ne_bytes()[0])
            .await?;
        self.onewire
            .write_byte(alarm_temp_low.to_ne_bytes()[0])
            .await?;
        self.onewire
            .write_byte(resolution.to_config_register())
            .await?;
        Ok(())
    }

    /// Broadcast scratchpad config to all devices on the chain.
    pub async fn simultaneous_set_config(
        &mut self,
        alarm_temp_low: i8,
        alarm_temp_high: i8,
        resolution: Resolution,
    ) -> OneWireResult<(), O::BusError> {
        self.onewire.address(None).await?;
        self.onewire.write_byte(commands::WRITE_SCRATCHPAD).await?;
        self.onewire
            .write_byte(alarm_temp_high.to_ne_bytes()[0])
            .await?;
        self.onewire
            .write_byte(alarm_temp_low.to_ne_bytes()[0])
            .await?;
        self.onewire
            .write_byte(resolution.to_config_register())
            .await?;
        Ok(())
    }

    pub async fn save_to_eeprom(
        &mut self,
        device: Device,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), O::BusError> {
        self.onewire.address(Some(device.id.0)).await?;
        self.onewire.write_byte(commands::COPY_SCRATCHPAD).await?;
        delay.delay_us(10_000).await; // write can take up to 10 ms
        Ok(())
    }

    /// Save config from scratchpad to EEPROM for all devices simultaneously.
    pub async fn simultaneous_save_to_eeprom(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), O::BusError> {
        self.onewire.address(None).await?;
        self.onewire.write_byte(commands::COPY_SCRATCHPAD).await?;
        delay.delay_us(10_000).await; // write can take up to 10 ms
        Ok(())
    }

    pub async fn recall_from_eeprom(
        &mut self,
        device: Device,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), O::BusError> {
        self.onewire.address(Some(device.id.0)).await?;
        self.onewire.write_byte(commands::RECALL_EEPROM).await?;
        self.wait_recall_complete(delay).await
    }

    /// Recall config from EEPROM into scratchpad for all devices simultaneously.
    pub async fn simultaneous_recall_from_eeprom(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), O::BusError> {
        self.onewire.address(None).await?;
        self.onewire.write_byte(commands::RECALL_EEPROM).await?;
        self.wait_recall_complete(delay).await
    }

    async fn read_scratchpad(&mut self, device: Device) -> OneWireResult<[u8; 9], O::BusError> {
        self.onewire.address(Some(device.id.0)).await?;
        self.onewire.write_byte(commands::READ_SCRATCHPAD).await?;

        let mut scratchpad = [0_u8; 9];
        for byte in &mut scratchpad {
            *byte = self.onewire.read_byte().await?;
        }

        if !OneWireCrc::validate(&scratchpad) {
            return Err(OneWireError::InvalidCrc);
        }

        Ok(scratchpad)
    }

    async fn wait_recall_complete(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), O::BusError> {
        // Recall can take up to 10 ms according to DS18B20 datasheet.
        for _ in 0..10 {
            if self.onewire.read_bit().await? {
                return Ok(());
            }
            delay.delay_ms(1).await;
        }

        Err(OneWireError::InvalidValue("recall from EEPROM timed out"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Raw register values from the DS18B20 datasheet's own worked examples
    // ("Temperature Register Format" table), 12-bit resolution throughout.
    #[test]
    fn positive_datasheet_examples_at_12_bit() {
        assert_eq!(raw_to_celsius(0x07D0, Resolution::Bits12), 125.0);
        assert_eq!(raw_to_celsius(0x0191, Resolution::Bits12), 25.0625);
        assert_eq!(raw_to_celsius(0x00A2, Resolution::Bits12), 10.125);
        assert_eq!(raw_to_celsius(0x0008, Resolution::Bits12), 0.5);
        assert_eq!(raw_to_celsius(0x0000, Resolution::Bits12), 0.0);
    }

    // The whole point of the fix: these are negative in the datasheet, and a
    // `u16` read would turn every one of them into several thousand degrees.
    #[test]
    fn negative_datasheet_examples_at_12_bit() {
        assert_eq!(raw_to_celsius(-8_i16, Resolution::Bits12), -0.5); // 0xFFF8
        assert_eq!(raw_to_celsius(-162_i16, Resolution::Bits12), -10.125); // 0xFF5E
        assert_eq!(raw_to_celsius(-401_i16, Resolution::Bits12), -25.0625); // 0xFE6F
        assert_eq!(raw_to_celsius(-880_i16, Resolution::Bits12), -55.0); // 0xFC90
    }

    // The exact case this fix was written for: a real board's raw scratchpad
    // register was 0xFFFF (a benign, near-zero -0.0625 °C reading), and the
    // old unsigned read turned it into a downstream report of 32767
    // centi-Celsius — this is the arithmetic that connects the two.
    #[test]
    fn the_reported_0xffff_case_is_a_benign_near_zero_reading() {
        let raw = i16::from_le_bytes(0xFFFFu16.to_le_bytes());
        let celsius = raw_to_celsius(raw, Resolution::Bits12);
        assert_eq!(celsius, -0.0625);

        // What the old, buggy unsigned read would have produced instead —
        // pinned here so nobody re-introduces it by "simplifying" the cast.
        let buggy_unsigned_celsius = (0xFFFFu16 as f32) / 16.0;
        assert_eq!(buggy_unsigned_celsius, 4095.9375);
        assert_eq!((buggy_unsigned_celsius * 100.0) as i16, i16::MAX);
    }

    #[test]
    fn scale_is_the_same_regardless_of_configured_resolution() {
        // The register's place-value never changes with resolution -- only
        // which low bits are guaranteed meaningful does. A raw count of 200
        // (12.5 C at the one true 1/16 C scale) must decode identically no
        // matter which `Resolution` the config register reports, because the
        // driver has no way to know -- and no reason to assume -- that a
        // lower-resolution device rescales its own register.
        for resolution in [
            Resolution::Bits9,
            Resolution::Bits10,
            Resolution::Bits11,
            Resolution::Bits12,
        ] {
            assert_eq!(raw_to_celsius(200, resolution), 12.5);
        }
    }

    // The live case this fix was written for: raw 408 at "9-bit" was
    // reported by the OLD (per-resolution-divisor) code as 204.0 C, a value
    // absurd enough to trip the plausibility fault -- but it is real,
    // unremarkable ambient (25.5 C) once correctly divided by 16 like every
    // other resolution.
    #[test]
    fn the_live_9_bit_ambient_case_that_used_to_read_as_204_degrees() {
        assert_eq!(raw_to_celsius(408, Resolution::Bits9), 25.5);
        assert_eq!(raw_to_celsius(424, Resolution::Bits9), 26.5);
    }
}
