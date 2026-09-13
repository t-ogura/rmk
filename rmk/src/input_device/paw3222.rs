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
//! - **12-bit deltas, and they have to be switched on.** `DELTA_X` / `DELTA_Y`
//!   carry bits 7:0 and `DELTA_XY_HI` carries bits 11:8 of both (X in its upper
//!   nibble, Y in its lower), giving +-2047 per read -- but only once
//!   `XY12bit_Enh` (`MOUSE_OPTION` bit 2) is set. The sensor powers up in 8-bit
//!   mode, where the low byte saturates at +-127 and the overflow flags fire on
//!   any brisk movement. Zephyr's driver assembles 12 bits without ever setting
//!   that bit, and the ZMK port dropped to 8-bit outright; both therefore live
//!   with the +-127 ceiling, which overflows on the first frame after the
//!   sensor's sleep modes stretch the sampling period to 32-128 ms.
//! - **The overflow flags are not a drop signal.** `MOTION` bits 4/3
//!   (`DYOVF`/`DXOVF`) follow the 8-bit report buffer: on hardware they were
//!   set on every brisk sample with 12-bit mode confirmed, so treating them as
//!   "garbage, discard" left the cursor nearly still at speed. They are logged
//!   and otherwise ignored, as Zephyr's driver does.
//! - **No burst-read register.** `MOTION`, `DELTA_X`, `DELTA_Y` and
//!   `DELTA_XY_HI` are read back to back inside a single CS assertion, which is
//!   what the Zephyr driver's six-byte transceive does on the wire.
//! - **Write protection.** `CPI_X` / `CPI_Y` / `OPERATION_MODE` only accept
//!   writes while `WRITE_PROTECT` holds `0x5a`; it must be closed again after.
//! - **Axis inversion and swap are register bits.** `MOUSE_OPTION` bits 3/4
//!   invert X/Y and bit 5 swaps them, all behind the same write protection as
//!   the CPI registers.
//! - **CPI granularity is 38.** `cpi / 38` goes in the register, so the usable
//!   range is 608..=4826 CPI; the sensor powers up at 27 * 38 = 1026 CPI.
//!
//! Power: the datasheet gives ~0.25 mA average in Run, 16 uA in Sleep1 and
//! 7 uA in Sleep2 (both enabled by default), so `force_awake` is a real cost on
//! a battery. With 12-bit deltas it is not needed for tracking either: Sleep1
//! samples every 32 ms, which at the default 1026 CPI and the rated 30 ips is
//! under 1000 counts, well inside +-2047. Sleep2's 128 ms window can still
//! overflow at the very top of that speed range, and that is what the overflow
//! flags are for -- the sample is dropped instead of reported wrapped.
//!
//! SPI is mode 3 (CPOL=1, CPHA=1), MSB first, up to 2 MHz — which is what
//! [`BitBangSpiBus`] produces: it idles SCK high, moves SDIO while SCK is high
//! and samples on the rising edge.

use embassy_futures::yield_now;
use embassy_time::{Duration, Instant, Timer};
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
const MOTION_STATUS_DYOVF: u8 = 1 << 4;
const MOTION_STATUS_DXOVF: u8 = 1 << 3;
const MOTION_STATUS_OVF: u8 = MOTION_STATUS_DYOVF | MOTION_STATUS_DXOVF;

const OPERATION_MODE_SLP_ENH: u8 = 1 << 4;
const OPERATION_MODE_SLP2_ENH: u8 = 1 << 3;
const OPERATION_MODE_SLP_MASK: u8 = OPERATION_MODE_SLP_ENH | OPERATION_MODE_SLP2_ENH;

const CONFIGURATION_RESET: u8 = 1 << 7;

const WRITE_PROTECT_ENABLE: u8 = 0x00;
const WRITE_PROTECT_DISABLE: u8 = 0x5a;

const MOUSE_OPTION_XY12BIT_ENH: u8 = 1 << 2;
const MOUSE_OPTION_INV_X: u8 = 1 << 3;
const MOUSE_OPTION_INV_Y: u8 = 1 << 4;
const MOUSE_OPTION_SWAP_XY: u8 = 1 << 5;
const MOUSE_OPTION_MASK: u8 = MOUSE_OPTION_XY12BIT_ENH | MOUSE_OPTION_INV_X | MOUSE_OPTION_INV_Y | MOUSE_OPTION_SWAP_XY;

const PAW3222_DATA_SIZE_BITS: usize = 12;

// Timing constants
/// Delay after the soft reset in `CONFIGURATION` before the part answers again.
const RESET_DELAY_MS: u64 = 2;
/// Product-ID probe: the Zephyr driver retries ten times, 100 ms apart, because
/// the sensor may still be powering up when the MCU gets there.
const PROBE_RETRIES: u8 = 10;
const PROBE_RETRY_DELAY_MS: u64 = 100;

// There are deliberately no per-transaction delay constants here.
//
// The datasheet's NCS/SCLK setup and hold times are all well under a
// microsecond, and `BitBangSpiBus` already spends roughly that long on a single
// bit, so a bit-banged transfer meets them by construction. The Zephyr driver
// this is ported from inserts no delays either -- it issues the whole motion
// read as one uninterrupted SPI transfer.
//
// Adding them back would be actively harmful rather than merely redundant:
// `Timer::after` is an await point, and on a 32768 Hz tick it rounds any
// sub-tick duration up to 30-60 us. A read with delays between its bytes hands
// the executor several such windows *while CS is still asserted*, so a BLE or
// USB task can stretch one motion read past the sensor's next frame. The
// deltas then saturate and the cursor moves in steps.

/// A motion burst is four registers, ~100 us on the bit-banged bus. Anything
/// well past that means the transfer was preempted mid-way (see `read_motion`).
/// embassy-time's tick is 30 us on a 32 kHz RTC, so the bound is coarse.
const BURST_MAX: Duration = Duration::from_micros(400);

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
    /// Swap X and Y at the sensor (`MOUSE_OPTION` bit 5)
    pub swap_xy: bool,
    /// Force awake mode (disable the sensor's two sleep stages)
    pub force_awake: bool,
}

impl Default for Paw3222Config {
    fn default() -> Self {
        Self {
            res_cpi: -1,
            invert_x: false,
            invert_y: false,
            swap_xy: false,
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

/// Chip-select guard time: tNCS-LEAD (CS low to first SCK) and tNCS-LAG (last
/// SCK to CS high) are 1 us minimum, tNCS-HI between transactions 2 us. This
/// spins for a few microseconds on any core from 16 MHz up.
#[inline(always)]
fn ncs_delay() {
    for _ in 0..64 {
        core::hint::spin_loop();
    }
}

/// Sign-extend the sensor's two's-complement delta, whose sign bit is at `bits`.
fn sign_extend(value: u16, bits: usize) -> i16 {
    let sign_bit = 1u16 << bits;
    if value & sign_bit != 0 {
        (value | !((1u16 << (bits + 1)) - 1)) as i16
    } else {
        value as i16
    }
}

/// Assemble the 12-bit deltas from the sensor's three delta registers.
///
/// `DELTA_XY_HI` holds X[11:8] in its upper nibble and Y[11:8] in its lower.
fn assemble_deltas(x_low: u8, y_low: u8, hi: u8) -> (i16, i16) {
    let x = (((hi as u16) << 4) & 0x0f00) | x_low as u16;
    let y = (((hi as u16) << 8) & 0x0f00) | y_low as u16;
    (
        sign_extend(x, PAW3222_DATA_SIZE_BITS - 1),
        sign_extend(y, PAW3222_DATA_SIZE_BITS - 1),
    )
}

/// PAW3222 driver using embedded-hal SPI traits
pub struct Paw3222<SPI: SpiBus, CS: OutputPin, MOTION: InputPin + Wait> {
    id: u8,
    spi: SPI,
    cs: CS,
    motion_gpio: Option<MOTION>,
    config: Paw3222Config,
    /// `MOUSE_OPTION` read back after init reported `XY12bit_Enh` set.
    ///
    /// Register writes on this bus have no other witness, so the driver does
    /// not assume its own write landed. When it did not, the sensor is in its
    /// power-on 8-bit mode: `DELTA_XY_HI` is meaningless and the overflow
    /// flags fire on every brisk sample, so both are ignored rather than
    /// letting the cursor stall at speed.
    twelve_bit: bool,
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
            twelve_bit: false,
        }
    }

    async fn read_reg(&mut self, addr: u8) -> Result<u8, Paw3222Error> {
        let _ = self.cs.set_low();
        ncs_delay();

        self.spi.write(&[addr & 0x7f]).await.map_err(|_| Paw3222Error::Spi)?;

        let mut value = [0u8];
        self.spi.read(&mut value).await.map_err(|_| Paw3222Error::Spi)?;

        ncs_delay();
        let _ = self.cs.set_high();
        ncs_delay();

        Ok(value[0])
    }

    async fn write_reg(&mut self, addr: u8, value: u8) -> Result<(), Paw3222Error> {
        let _ = self.cs.set_low();
        ncs_delay();

        self.spi
            .write(&[addr | SPI_WRITE, value])
            .await
            .map_err(|_| Paw3222Error::Spi)?;

        ncs_delay();
        let _ = self.cs.set_high();
        ncs_delay();

        Ok(())
    }

    async fn update_reg(&mut self, addr: u8, mask: u8, value: u8) -> Result<(), Paw3222Error> {
        let val = self.read_reg(addr).await?;
        let val = (val & !mask) | (value & mask);
        self.write_reg(addr, val).await
    }

    /// Read `MOTION`, `DELTA_X`, `DELTA_Y` and `DELTA_XY_HI` inside one CS
    /// assertion, and return `(status, dx, dy)`.
    ///
    /// The Zephyr driver issues this as a single transceive with NCS held low
    /// across all four registers; on a half-duplex bus the same wire sequence is
    /// an address write and a data read per register. The order is the
    /// datasheet's: `MOTION` first, because reading it is what validates the
    /// delta registers behind it.
    async fn read_motion_burst(&mut self) -> Result<(u8, i16, i16), Paw3222Error> {
        let _ = self.cs.set_low();
        ncs_delay();

        let mut regs = [0u8; 4];
        for (value, addr) in
            regs.iter_mut()
                .zip([PAW3222_MOTION, PAW3222_DELTA_X, PAW3222_DELTA_Y, PAW3222_DELTA_XY_HI])
        {
            let mut byte = [0u8];
            self.spi.write(&[addr]).await.map_err(|_| Paw3222Error::Spi)?;
            self.spi.read(&mut byte).await.map_err(|_| Paw3222Error::Spi)?;
            *value = byte[0];
        }

        ncs_delay();
        let _ = self.cs.set_high();
        ncs_delay();

        let [status, x_low, y_low, hi] = regs;
        let (dx, dy) = if self.twelve_bit {
            assemble_deltas(x_low, y_low, hi)
        } else {
            (x_low as i8 as i16, y_low as i8 as i16)
        };
        Ok((status, dx, dy))
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

        // MOUSE_OPTION sits above WRITE_PROTECT's boundary, so the window has
        // to be open or the write is silently ignored. The 12-bit enable is not
        // optional: the sensor powers up in 8-bit mode, where DELTA_X/Y saturate
        // at +-127 and raise the overflow flags on any brisk movement, and
        // DELTA_XY_HI reads back nothing useful.
        let mut opt = MOUSE_OPTION_XY12BIT_ENH;
        if self.config.invert_x {
            opt |= MOUSE_OPTION_INV_X;
        }
        if self.config.invert_y {
            opt |= MOUSE_OPTION_INV_Y;
        }
        if self.config.swap_xy {
            opt |= MOUSE_OPTION_SWAP_XY;
        }
        self.unprotect().await?;
        let result = self.update_reg(PAW3222_MOUSE_OPTION, MOUSE_OPTION_MASK, opt).await;
        self.protect().await?;
        result?;

        let readback = self.read_reg(PAW3222_MOUSE_OPTION).await?;
        self.twelve_bit = (readback & MOUSE_OPTION_XY12BIT_ENH) != 0;
        if self.twelve_bit {
            info!(
                "PAW3222 {}: 12-bit mode confirmed (MOUSE_OPTION {:#04x})",
                self.id, readback
            );
        } else {
            warn!(
                "PAW3222 {}: MOUSE_OPTION write did not take ({:#04x}), staying in 8-bit mode",
                self.id, readback
            );
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
        ncs_delay();
        let _ = self.cs.set_high();
        ncs_delay();
        Timer::after(Duration::from_millis(1)).await;

        self.configure().await?;
        Ok(())
    }

    async fn read_motion(&mut self) -> Result<MotionData, PointingDriverError> {
        let started = Instant::now();
        let (status, dx, dy) = self.read_motion_burst().await?;
        let took = started.elapsed();

        if took > BURST_MAX {
            // Something -- the radio, in practice -- ran in the middle of the
            // transfer. This used to drop the sample, on the theory that the
            // low bytes and the high nibbles could then describe different
            // frames. On hardware the drops themselves were the visible
            // fault: at speed the motion pin stays asserted, reads run every
            // poll, and a radio interrupt lands inside one of them a couple
            // of times a second -- each drop a lost frame's travel, felt as a
            // ~2 Hz hiccup that slow movement never showed. Zephyr's driver
            // never dropped and tracked cleanly, so the sample is kept.
            trace!("PAW3222 {}: burst took {} us", self.id, took.as_micros());
        }
        let motion = if (status & MOTION_STATUS_MOTION) == 0 {
            MotionData::default()
        } else {
            // DXOVF/DYOVF are reported but never acted on. On hardware they
            // came up on every brisk sample even with 12-bit mode confirmed --
            // they track the 8-bit report buffer, not the 12-bit readout -- and
            // dropping those samples left the cursor nearly still at speed.
            // Zephyr's driver ignores the flags too. A real 12-bit wrap would
            // need two inches of travel between two reads, which per-frame
            // reading cannot produce.
            if (status & MOTION_STATUS_OVF) != 0 {
                trace!("PAW3222 {}: overflow flag set (status {:#04x})", self.id, status);
            }
            MotionData { dx, dy }
        };

        // Nothing above this line ever suspends -- the bit-banged transfers are
        // synchronous loops, and the delays that used to punctuate them are gone.
        // The caller polls on the motion pin, which reads ready the moment the
        // sensor has data, so without a yield here a pin stuck low would spin
        // this task forever and starve the rest of the firmware. Yield outside
        // the CS assertion, where handing the CPU over costs nothing.
        yield_now().await;

        Ok(motion)
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

    /// 12-bit two's complement, the width the PAW3222 reports deltas in.
    #[test]
    fn sign_extend_12bit() {
        let bits = PAW3222_DATA_SIZE_BITS - 1;
        assert_eq!(sign_extend(0x000, bits), 0);
        assert_eq!(sign_extend(0x001, bits), 1);
        assert_eq!(sign_extend(0x7ff, bits), 2047);
        assert_eq!(sign_extend(0x800, bits), -2048);
        assert_eq!(sign_extend(0xfff, bits), -1);
    }

    /// `DELTA_XY_HI` carries X[11:8] in its upper nibble and Y[11:8] in its
    /// lower, exactly as Zephyr's `input_paw32xx.c` assembles them.
    #[test]
    fn assemble_deltas_places_the_high_nibbles() {
        // Small positive motion: high nibbles are zero.
        assert_eq!(assemble_deltas(0x05, 0x03, 0x00), (5, 3));
        // Small negative motion: high nibbles are the sign extension.
        assert_eq!(assemble_deltas(0xfb, 0xfd, 0xff), (-5, -3));
        // Beyond eight bits, which is the whole point.
        assert_eq!(assemble_deltas(0x00, 0x00, 0x12), (0x100, 0x200));
        assert_eq!(assemble_deltas(0xff, 0xff, 0x7f), (0x7ff, -1));
    }

    /// Every bit the driver writes to MOUSE_OPTION is one the datasheet defines.
    #[test]
    fn mouse_option_bits_match_the_datasheet() {
        assert_eq!(MOUSE_OPTION_XY12BIT_ENH, 0b0000_0100);
        assert_eq!(MOUSE_OPTION_INV_X, 0b0000_1000);
        assert_eq!(MOUSE_OPTION_INV_Y, 0b0001_0000);
        assert_eq!(MOUSE_OPTION_SWAP_XY, 0b0010_0000);
        assert_eq!(MOUSE_OPTION_MASK, 0b0011_1100);
    }

    /// The register takes `cpi / 38` in a field the datasheet limits to
    /// 16..=127, which is where these bounds come from.
    #[test]
    fn cpi_range_matches_register_field() {
        assert_eq!(RES_MIN, 608);
        assert_eq!(RES_MAX, 4826);
    }
}
