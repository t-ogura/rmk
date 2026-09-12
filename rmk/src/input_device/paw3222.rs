//! PAW3222 Low-Power Optical Mouse Sensor Driver
//!
//! Ported from the Zephyr/ZMK driver implementation:
//! <https://github.com/sekigon-gonnoc/zmk-driver-paw3222>, itself derived from
//! Zephyr's `input_paw32xx.c`.
//!
//! The PAW3222 is the sensor used by several trackball split keyboards
//! (Torabo-Tsuki, Cornix-TB, ...). It speaks the same single-wire (SDIO)
//! half-duplex SPI as the PMW3610, so it uses the same [`BitBangSpiBus`], and
//! this driver is structured to mirror [`crate::input_device::pmw3610`].
//!
//! Differences from the PMW3610 worth knowing:
//!
//! - **8-bit deltas, not 12.** `DELTA_X` / `DELTA_Y` are one byte each and are
//!   sign-extended. There is a `DELTA_XY_HI` register, but the Zephyr driver
//!   reads it only to drain it during init and this driver does the same.
//! - **No burst-read register.** Motion is read as `MOTION`, then `DELTA_X` and
//!   `DELTA_Y` back to back inside a single CS assertion, which is what the
//!   Zephyr driver's 4-byte transceive does on the wire.
//! - **Write protection.** `CPI_X` / `CPI_Y` / `OPERATION_MODE` only accept
//!   writes while `WRITE_PROTECT` holds `0x5a`; it must be closed again after.
//! - **CPI granularity is 38.** `cpi / 38` goes in the register, so the usable
//!   range is 608..=4826 CPI.
//!
//! SPI is mode 3 (CPOL=1, CPHA=1), MSB first, up to 2 MHz — which is what
//! [`BitBangSpiBus`] produces: it idles SCK high, moves SDIO while SCK is high
//! and samples on the rising edge.

use embassy_time::{Duration, Timer};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiBus;

pub use crate::driver::bitbang_spi::{BitBangError, BitBangSpiBus};
use crate::input_device::pointing::{InitState, MotionData, PointingDevice, PointingDriver, PointingDriverError};

// ============================================================================
// Registers
// ============================================================================
const PAW3222_PRODUCT_ID1: u8 = 0x00;
const PAW3222_MOTION: u8 = 0x02;
const PAW3222_DELTA_X: u8 = 0x03;
const PAW3222_DELTA_Y: u8 = 0x04;
const PAW3222_OPERATION_MODE: u8 = 0x05;
const PAW3222_CONFIGURATION: u8 = 0x06;
const PAW3222_WRITE_PROTECT: u8 = 0x09;
const PAW3222_CPI_X: u8 = 0x0d;
const PAW3222_CPI_Y: u8 = 0x0e;
const PAW3222_DELTA_XY_HI: u8 = 0x12;
const PAW3222_MOUSE_OPTION: u8 = 0x19;

// ============================================================================
// Constants
// ============================================================================
const PRODUCT_ID_PAW3222: u8 = 0x30;
const SPI_WRITE: u8 = 0x80;
const MOTION_STATUS_MOTION: u8 = 0x80;

const OPERATION_MODE_SLP_ENH: u8 = 1 << 4;
const OPERATION_MODE_SLP2_ENH: u8 = 1 << 3;
const OPERATION_MODE_SLP_MASK: u8 = OPERATION_MODE_SLP_ENH | OPERATION_MODE_SLP2_ENH;

const CONFIGURATION_RESET: u8 = 1 << 7;

const WRITE_PROTECT_ENABLE: u8 = 0x00;
const WRITE_PROTECT_DISABLE: u8 = 0x5a;

const MOUSE_OPTION_INV_X: u8 = 1 << 3;
const MOUSE_OPTION_INV_Y: u8 = 1 << 4;
const MOUSE_OPTION_INV_MASK: u8 = MOUSE_OPTION_INV_X | MOUSE_OPTION_INV_Y;

const PAW3222_DATA_SIZE_BITS: usize = 8;

// Timing constants
/// Delay after the soft reset in `CONFIGURATION` before the part answers again.
const RESET_DELAY_MS: u64 = 2;
/// Product-ID probe: the Zephyr driver retries ten times, 100 ms apart, because
/// the sensor may still be powering up when the MCU gets there.
const PROBE_RETRIES: u8 = 10;
const PROBE_RETRY_DELAY_MS: u64 = 100;

/// NCS falling edge to the first SCK edge.
const T_NCS_SCLK_NS: u64 = 120;
/// Address byte to the first data bit of a read.
const T_SRAD_US: u64 = 2;
/// Last SCK falling edge to NCS rising, for a read.
const T_SCLK_NCS_R_NS: u64 = 120;
/// Last SCK falling edge to NCS rising, for a write.
const T_SCLK_NCS_W_US: u64 = 5;
/// NCS rising edge to the next command.
const T_SWX_US: u64 = 5;

// Resolution constants: the register takes `cpi / RES_STEP`, valid 16..=127.
const RES_STEP: u16 = 38;
const RES_MIN: u16 = 16 * RES_STEP;
const RES_MAX: u16 = 127 * RES_STEP;

/// PAW3222 configuration
#[derive(Clone)]
pub struct Paw3222Config {
    /// CPI resolution (608-4826, step 38). Set to -1 to keep the sensor's own default.
    pub res_cpi: i16,
    /// Invert X at the sensor (`MOUSE_OPTION` bit 3)
    pub invert_x: bool,
    /// Invert Y at the sensor (`MOUSE_OPTION` bit 4)
    pub invert_y: bool,
    /// Force awake mode (disable the sensor's two sleep stages)
    pub force_awake: bool,
}

impl Default for Paw3222Config {
    fn default() -> Self {
        Self {
            res_cpi: -1,
            invert_x: false,
            invert_y: false,
            force_awake: false,
        }
    }
}

/// PAW3222 error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Paw3222Error {
    /// SPI communication error
    Spi,
    /// Invalid product ID detected
    InvalidProductId(u8),
    /// Initialization failed
    InitFailed,
    /// Invalid CPI value
    InvalidCpi,
}

impl From<Paw3222Error> for PointingDriverError {
    fn from(err: Paw3222Error) -> Self {
        match err {
            Paw3222Error::Spi => PointingDriverError::Spi,
            Paw3222Error::InvalidProductId(id) => PointingDriverError::InvalidProductId(id),
            Paw3222Error::InitFailed => PointingDriverError::InitFailed,
            Paw3222Error::InvalidCpi => PointingDriverError::InvalidCpi,
        }
    }
}

/// Sign-extend the sensor's two's-complement delta, whose sign bit is at `bits`.
fn sign_extend(value: u8, bits: usize) -> i16 {
    let sign_bit = 1u16 << bits;
    let value = value as u16;
    if value & sign_bit != 0 {
        (value | !((1u16 << (bits + 1)) - 1)) as i16
    } else {
        value as i16
    }
}

/// PAW3222 driver using embedded-hal SPI traits
pub struct Paw3222<SPI: SpiBus, CS: OutputPin, MOTION: InputPin + Wait> {
    id: u8,
    spi: SPI,
    cs: CS,
    motion_gpio: Option<MOTION>,
    config: Paw3222Config,
}

impl<SPI: SpiBus, CS: OutputPin, MOTION: InputPin + Wait> Paw3222<SPI, CS, MOTION> {
    /// Create a new PAW3222 driver instance
    pub fn new(id: u8, spi: SPI, cs: CS, motion_gpio: Option<MOTION>, config: Paw3222Config) -> Self {
        Self {
            id,
            spi,
            cs,
            motion_gpio,
            config,
        }
    }

    async fn read_reg(&mut self, addr: u8) -> Result<u8, Paw3222Error> {
        let _ = self.cs.set_low();
        Timer::after(Duration::from_nanos(T_NCS_SCLK_NS)).await;

        self.spi.write(&[addr & 0x7f]).await.map_err(|_| Paw3222Error::Spi)?;

        Timer::after(Duration::from_micros(T_SRAD_US)).await;

        let mut value = [0u8];
        self.spi.read(&mut value).await.map_err(|_| Paw3222Error::Spi)?;

        Timer::after(Duration::from_nanos(T_SCLK_NCS_R_NS)).await;
        let _ = self.cs.set_high();

        Timer::after(Duration::from_micros(T_SWX_US)).await;

        Ok(value[0])
    }

    async fn write_reg(&mut self, addr: u8, value: u8) -> Result<(), Paw3222Error> {
        let _ = self.cs.set_low();
        Timer::after(Duration::from_nanos(T_NCS_SCLK_NS)).await;

        self.spi
            .write(&[addr | SPI_WRITE, value])
            .await
            .map_err(|_| Paw3222Error::Spi)?;

        Timer::after(Duration::from_micros(T_SCLK_NCS_W_US)).await;
        let _ = self.cs.set_high();

        Timer::after(Duration::from_micros(T_SWX_US)).await;

        Ok(())
    }

    async fn update_reg(&mut self, addr: u8, mask: u8, value: u8) -> Result<(), Paw3222Error> {
        let val = self.read_reg(addr).await?;
        let val = (val & !mask) | (value & mask);
        self.write_reg(addr, val).await
    }

    /// Read `DELTA_X` then `DELTA_Y` inside one CS assertion.
    ///
    /// The Zephyr driver does this as a single four-byte transceive
    /// (`[DELTA_X, 0xff, DELTA_Y, 0xff]`), keeping NCS low across both
    /// registers; on a half-duplex bus the same wire sequence is an address
    /// write, a data read, another address write and another data read. Reading
    /// the two registers under one assertion is what the datasheet's motion
    /// read describes, so it is kept rather than split into two `read_reg`s.
    async fn read_xy(&mut self) -> Result<(i16, i16), Paw3222Error> {
        let _ = self.cs.set_low();
        Timer::after(Duration::from_nanos(T_NCS_SCLK_NS)).await;

        let mut x = [0u8];
        self.spi
            .write(&[PAW3222_DELTA_X])
            .await
            .map_err(|_| Paw3222Error::Spi)?;
        Timer::after(Duration::from_micros(T_SRAD_US)).await;
        self.spi.read(&mut x).await.map_err(|_| Paw3222Error::Spi)?;

        let mut y = [0u8];
        self.spi
            .write(&[PAW3222_DELTA_Y])
            .await
            .map_err(|_| Paw3222Error::Spi)?;
        Timer::after(Duration::from_micros(T_SRAD_US)).await;
        self.spi.read(&mut y).await.map_err(|_| Paw3222Error::Spi)?;

        Timer::after(Duration::from_nanos(T_SCLK_NCS_R_NS)).await;
        let _ = self.cs.set_high();

        Timer::after(Duration::from_micros(T_SWX_US)).await;

        Ok((
            sign_extend(x[0], PAW3222_DATA_SIZE_BITS - 1),
            sign_extend(y[0], PAW3222_DATA_SIZE_BITS - 1),
        ))
    }

    /// `CPI_X`, `CPI_Y` and `OPERATION_MODE` silently ignore writes unless
    /// `WRITE_PROTECT` holds `0x5a`, so every write to them is bracketed by
    /// these two. The window is closed even when the body failed, so a
    /// transient SPI error cannot leave those registers writable.
    async fn unprotect(&mut self) -> Result<(), Paw3222Error> {
        self.write_reg(PAW3222_WRITE_PROTECT, WRITE_PROTECT_DISABLE).await
    }

    async fn protect(&mut self) -> Result<(), Paw3222Error> {
        self.write_reg(PAW3222_WRITE_PROTECT, WRITE_PROTECT_ENABLE).await
    }

    async fn set_force_awake(&mut self, enable: bool) -> Result<(), Paw3222Error> {
        let val = if enable { 0 } else { OPERATION_MODE_SLP_MASK };
        self.unprotect().await?;
        let result = self
            .update_reg(PAW3222_OPERATION_MODE, OPERATION_MODE_SLP_MASK, val)
            .await;
        self.protect().await?;
        result
    }

    async fn configure(&mut self) -> Result<(), Paw3222Error> {
        // The sensor may still be powering up, so probe rather than assume.
        let mut last = 0u8;
        let mut detected = false;
        for _ in 0..PROBE_RETRIES {
            match self.read_reg(PAW3222_PRODUCT_ID1).await {
                Ok(PRODUCT_ID_PAW3222) => {
                    detected = true;
                    break;
                }
                Ok(other) => last = other,
                Err(_) => last = 0,
            }
            Timer::after(Duration::from_millis(PROBE_RETRY_DELAY_MS)).await;
        }
        if !detected {
            error!("PAW3222 {}: invalid product id: {:#04x}", self.id, last);
            return Err(Paw3222Error::InvalidProductId(last));
        }
        info!("PAW3222 {} detected, product ID: {:#04x}", self.id, PRODUCT_ID_PAW3222);

        self.update_reg(PAW3222_CONFIGURATION, CONFIGURATION_RESET, CONFIGURATION_RESET)
            .await?;
        Timer::after(Duration::from_millis(RESET_DELAY_MS)).await;

        if self.config.res_cpi > 0 {
            self.set_cpi(self.config.res_cpi as u16).await?;
        }

        if self.config.invert_x || self.config.invert_y {
            let mut val = 0u8;
            if self.config.invert_x {
                val |= MOUSE_OPTION_INV_X;
            }
            if self.config.invert_y {
                val |= MOUSE_OPTION_INV_Y;
            }
            self.update_reg(PAW3222_MOUSE_OPTION, MOUSE_OPTION_INV_MASK, val)
                .await?;
        }

        self.set_force_awake(self.config.force_awake).await?;

        // Drain whatever accumulated during reset, so the first reported motion
        // is motion the user actually made.
        for reg in [PAW3222_MOTION, PAW3222_DELTA_X, PAW3222_DELTA_Y, PAW3222_DELTA_XY_HI] {
            self.read_reg(reg).await?;
        }

        info!("PAW3222 {} initialized successfully", self.id);
        Ok(())
    }

    async fn set_cpi(&mut self, cpi: u16) -> Result<(), Paw3222Error> {
        if !(RES_MIN..=RES_MAX).contains(&cpi) {
            error!("PAW3222 {}: res_cpi out of range: {}", self.id, cpi);
            return Err(Paw3222Error::InvalidCpi);
        }
        let val = (cpi / RES_STEP) as u8;
        self.unprotect().await?;
        let result = async {
            self.write_reg(PAW3222_CPI_X, val).await?;
            self.write_reg(PAW3222_CPI_Y, val).await
        }
        .await;
        self.protect().await?;
        result
    }
}

impl<SPI, CS, MOTION> PointingDriver for Paw3222<SPI, CS, MOTION>
where
    SPI: SpiBus,
    CS: OutputPin,
    MOTION: InputPin + Wait,
{
    type MOTION = MOTION;

    async fn init(&mut self) -> Result<(), PointingDriverError> {
        let _ = self.cs.set_high();
        Timer::after(Duration::from_millis(1)).await;

        self.configure().await?;
        Ok(())
    }

    async fn read_motion(&mut self) -> Result<MotionData, PointingDriverError> {
        let status = self.read_reg(PAW3222_MOTION).await?;
        if (status & MOTION_STATUS_MOTION) == 0 {
            return Ok(MotionData::default());
        }

        let (dx, dy) = self.read_xy().await?;
        Ok(MotionData { dx, dy })
    }

    /// Check if motion is pending (motion GPIO is active low)
    fn motion_pending(&mut self) -> bool {
        match &mut self.motion_gpio {
            Some(gpio) => gpio.is_low().unwrap_or(true),
            None => true,
        }
    }

    fn motion_gpio(&mut self) -> Option<&mut MOTION> {
        self.motion_gpio.as_mut()
    }

    /// Set sensor resolution in CPI (608-4826, step 38)
    async fn set_resolution(&mut self, cpi: u16) -> Result<(), PointingDriverError> {
        self.set_cpi(cpi).await?;
        debug!("PAW3222 {}: resolution set to {} CPI", self.id, cpi);
        Ok(())
    }
}

impl<SPI, CS, MOTION> PointingDevice<Paw3222<SPI, CS, MOTION>>
where
    SPI: SpiBus,
    CS: OutputPin,
    MOTION: InputPin + Wait,
{
    /// With a motion pin wired the poll never waits on this timer, but an
    /// unwired sensor still has to be looked at; the Zephyr driver re-checks
    /// every 15 ms while motion continues, and this is the same order.
    const DEFAULT_POLL_INTERVAL_US: u64 = 1000;
    const DEFAULT_REPORT_HZ: u16 = 125;

    /// Create a new PAW3222 device
    pub fn new(id: u8, spi: SPI, cs: CS, motion_gpio: Option<MOTION>, sensor_config: Paw3222Config) -> Self {
        Self::with_poll_interval_and_report_hz(
            id,
            spi,
            cs,
            motion_gpio,
            sensor_config,
            Self::DEFAULT_POLL_INTERVAL_US,
            Self::DEFAULT_REPORT_HZ,
        )
    }

    /// Create a new PAW3222 device with custom report rate (Hz)
    pub fn with_report_hz(
        id: u8,
        spi: SPI,
        cs: CS,
        motion_gpio: Option<MOTION>,
        sensor_config: Paw3222Config,
        report_hz: u16,
    ) -> Self {
        Self::with_poll_interval_and_report_hz(
            id,
            spi,
            cs,
            motion_gpio,
            sensor_config,
            Self::DEFAULT_POLL_INTERVAL_US,
            report_hz,
        )
    }

    /// Create a new PAW3222 device with custom poll interval
    pub fn with_poll_interval(
        id: u8,
        spi: SPI,
        cs: CS,
        motion_gpio: Option<MOTION>,
        sensor_config: Paw3222Config,
        poll_interval_us: u64,
    ) -> Self {
        Self::with_poll_interval_and_report_hz(
            id,
            spi,
            cs,
            motion_gpio,
            sensor_config,
            poll_interval_us,
            Self::DEFAULT_REPORT_HZ,
        )
    }

    /// Create a new PAW3222 device with custom poll interval and report rate
    pub fn with_poll_interval_and_report_hz(
        id: u8,
        spi: SPI,
        cs: CS,
        motion_gpio: Option<MOTION>,
        sensor_config: Paw3222Config,
        poll_interval_us: u64,
        report_hz: u16,
    ) -> Self {
        let report_interval = embassy_time::Duration::from_hz(report_hz as u64);

        // Polling should be more frequent than reporting
        let poll_interval = Duration::from_micros(poll_interval_us).min(report_interval);

        Self {
            id,
            sensor: Paw3222::new(id, spi, cs, motion_gpio, sensor_config),
            init_state: InitState::Pending,
            poll_interval,
            report_interval,
            last_poll: embassy_time::Instant::MIN,
            last_report: embassy_time::Instant::MIN,
            accumulated_x: 0,
            accumulated_y: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 8-bit two's complement, the width the PAW3222 reports deltas in.
    #[test]
    fn sign_extend_8bit() {
        let bits = PAW3222_DATA_SIZE_BITS - 1;
        assert_eq!(sign_extend(0x00, bits), 0);
        assert_eq!(sign_extend(0x01, bits), 1);
        assert_eq!(sign_extend(0x7f, bits), 127);
        assert_eq!(sign_extend(0x80, bits), -128);
        assert_eq!(sign_extend(0xff, bits), -1);
    }

    /// The register takes `cpi / 38` in a field the datasheet limits to
    /// 16..=127, which is where these bounds come from.
    #[test]
    fn cpi_range_matches_register_field() {
        assert_eq!(RES_MIN, 608);
        assert_eq!(RES_MAX, 4826);
    }
}
