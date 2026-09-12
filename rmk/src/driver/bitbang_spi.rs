//! Bit-banging SPI Bus Implementation
//!
//! This module provides a software SPI implementation using GPIO bit-banging.
//! It's designed for sensors that use a single bidirectional data line (half-duplex SPI).

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::ErrorType;
use embedded_hal_async::spi::SpiBus;

use crate::driver::flex_pin::FlexPin;

/// Error type for bit-banging SPI
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitBangError {
    /// Generic SPI error
    Bus,
}

impl embedded_hal::spi::Error for BitBangError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

/// Bit-banging SPI bus for half-duplex communication
///
/// This implements the `embedded_hal::spi::SpiBus` trait using GPIO bit-banging.
/// It's designed for sensors like PMW3610 that use a single bidirectional data line.
///
/// # Type Parameters
/// - `SCK`: SPI clock pin (output)
/// - `SDIO`: Bidirectional data pin (implements `FlexPin`)
pub struct BitBangSpiBus<SCK, SDIO>
where
    SCK: OutputPin,
    SDIO: FlexPin,
{
    sck: SCK,
    sdio: SDIO,
}

impl<SCK, SDIO> BitBangSpiBus<SCK, SDIO>
where
    SCK: OutputPin,
    SDIO: FlexPin,
{
    /// Create a new bit-banging SPI bus
    pub fn new(mut sck: SCK, sdio: SDIO) -> Self {
        let _ = sck.set_high();
        Self { sck, sdio }
    }

    /// One SCK phase: 12 spins is 0.5-1 us on a 64 MHz Cortex-M4 (about a
    /// quarter of that on a 125 MHz RP2040), above the PixArt parts' 250 ns
    /// hold time and under their 2 MHz clock maximum.
    #[inline(always)]
    fn spi_delay() {
        for _ in 0..12 {
            core::hint::spin_loop();
        }
    }

    fn write_byte(&mut self, byte: u8) {
        self.sdio.set_as_output();

        for i in (0..8).rev() {
            if (byte >> i) & 1 == 1 {
                let _ = self.sdio.set_high();
            } else {
                let _ = self.sdio.set_low();
            }
            Self::spi_delay();

            let _ = self.sck.set_low();
            Self::spi_delay();

            let _ = self.sck.set_high();
            Self::spi_delay();
        }
    }

    fn read_byte(&mut self) -> u8 {
        self.sdio.set_as_input();

        let mut byte = 0u8;

        for i in (0..8).rev() {
            let _ = self.sck.set_low();
            Self::spi_delay();

            // The sensor drives SDIO after the falling edge and guarantees it
            // only briefly past the rising one, so sample at the end of the
            // low phase, right before the rising edge -- not a delay after it.
            if self.sdio.is_high().unwrap_or(false) {
                byte |= 1 << i;
            }

            let _ = self.sck.set_high();
            Self::spi_delay();
        }

        byte
    }
}

impl<SCK, SDIO> ErrorType for BitBangSpiBus<SCK, SDIO>
where
    SCK: OutputPin,
    SDIO: FlexPin,
{
    type Error = BitBangError;
}

impl<SCK, SDIO> SpiBus for BitBangSpiBus<SCK, SDIO>
where
    SCK: OutputPin,
    SDIO: FlexPin,
{
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words.iter_mut() {
            *word = self.read_byte();
        }
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &word in words {
            self.write_byte(word);
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.write(write).await?;
        self.read(read).await?;
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words.iter_mut() {
            self.write_byte(*word);
            *word = self.read_byte();
        }
        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
