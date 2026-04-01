#![no_std]

use embedded_hal_async::delay::DelayNs;
use embedded_onewire::{
    OneWireAsync, OneWireCrc, OneWireError, OneWireResult, OneWireSearchAsync, OneWireSearchKind,
};

pub const FAMILY_CODE: u8 = 0x28;

pub mod commands;
mod resolution;

pub use resolution::Resolution;

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
pub struct Chain<O: OneWireAsync, const N: usize> {
    devices: [Device; N],
    onewire: O,
}

impl<O: OneWireAsync, const N: usize> Chain<O, N> {
    /// Initializes the chain by auto-discovering DS18B20 devices on the bus.
    pub async fn init(mut onewire: O) -> OneWireResult<Self, O::BusError> {
        let mut search =
            OneWireSearchAsync::with_family(&mut onewire, OneWireSearchKind::Normal, FAMILY_CODE);

        let mut devices = [Device { id: Address(0) }; N];
        for device in &mut devices {
            let rom = search.next().await?.ok_or(OneWireError::InvalidValue(
                "not enough DS18B20 devices found during discovery",
            ))?;
            device.id = Address(rom);
        }

        if search.next().await?.is_some() {
            return Err(OneWireError::InvalidValue(
                "found more DS18B20 devices than chain capacity",
            ));
        }

        Ok(Self { devices, onewire })
    }

    pub fn devices(&self) -> &[Device; N] {
        &self.devices
    }

    pub fn device_by_index(&self, index: usize) -> OneWireResult<Device, O::BusError> {
        let Some(device) = self.devices.get(index).copied() else {
            return Err(OneWireError::InvalidValue("device index out of range"));
        };

        Ok(device)
    }

    pub fn device_by_address(&self, address: Address) -> OneWireResult<Device, O::BusError> {
        let Some(device) = self.devices.iter().copied().find(|d| d.id == address) else {
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

    pub async fn read_data(&mut self) -> OneWireResult<[DeviceTemperature; N], O::BusError> {
        self.read_chain_temperatures().await
    }

    async fn read_device_data(&mut self, device: Device) -> OneWireResult<SensorData, O::BusError> {
        let scratchpad = self.read_scratchpad(device).await?;

        let resolution = if let Some(resolution) = Resolution::from_config_register(scratchpad[4]) {
            resolution
        } else {
            return Err(OneWireError::InvalidValue("invalid config register"));
        };

        let raw_temp = u16::from_le_bytes([scratchpad[0], scratchpad[1]]);
        let temperature = match resolution {
            Resolution::Bits12 => (raw_temp as f32) / 16.0,
            Resolution::Bits11 => (raw_temp as f32) / 8.0,
            Resolution::Bits10 => (raw_temp as f32) / 4.0,
            Resolution::Bits9 => (raw_temp as f32) / 2.0,
        };

        Ok(SensorData {
            temperature,
            resolution,
            alarm_temp_high: i8::from_le_bytes([scratchpad[2]]),
            alarm_temp_low: i8::from_le_bytes([scratchpad[3]]),
        })
    }

    pub async fn read_chain_temperatures(
        &mut self,
    ) -> OneWireResult<[DeviceTemperature; N], O::BusError> {
        let mut readings = [DeviceTemperature {
            id: Address(0),
            temperature: 0.0,
        }; N];

        let devices = self.devices;
        for (index, device) in devices.iter().copied().enumerate() {
            let data = self.read_device_data(device).await?;
            readings[index] = DeviceTemperature {
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
