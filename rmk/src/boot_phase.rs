//! Where boot is, for firmware that wants to show it.
//!
//! Everything before the task join runs sequentially in `main`, so a board
//! that is slow to come up gives no sign of *which* step is slow. RMK stamps
//! [`BOOT_PHASE`] as it passes the expensive steps; a board can read it from a
//! task spawned early (spawned tasks run whenever `main` awaits) and put it on
//! an LED. The values are ordered and only ever increase.

use core::sync::atomic::{AtomicU8, Ordering};

/// Not yet at storage initialisation: chip, radio and USB setup.
pub const CHIP_INIT: u8 = 0;
/// Entered storage initialisation.
pub const STORAGE_ENTER: u8 = 1;
/// The stored config was checked against this build.
pub const STORAGE_CHECKED: u8 = 2;
/// Storage initialised (or the layout synced) and returned.
pub const STORAGE_DONE: u8 = 3;
/// The keymap and behavior config were read back from storage.
pub const KEYMAP_LOADED: u8 = 4;
/// Building the USB transport (keymap, matrix and host service are done).
pub const USB_READY: u8 = 5;
/// Building the BLE transport; the join follows.
pub const BLE_READY: u8 = 6;

/// The most recent phase reached; see the constants above.
pub static BOOT_PHASE: AtomicU8 = AtomicU8::new(CHIP_INIT);

pub(crate) fn stamp(phase: u8) {
    BOOT_PHASE.store(phase, Ordering::Release);
}

/// Reload a watchdog left running by the previous boot.
///
/// An nRF52 watchdog is not stopped by a reset-button or software reset, so
/// after one it keeps counting down through the next boot's initialisation,
/// which nothing feeds until the tasks are up. Storage initialisation on a
/// fresh build is a few hundred MPSL-scheduled flash writes -- long enough
/// for the dog to bite mid-way, over and over, each attempt getting a little
/// further. Reloading from the write path keeps the window open while that
/// work is in progress. On a watchdog that is not running the write is
/// ignored; on other chips this is a no-op.
pub(crate) fn pet_stale_watchdog() {
    #[cfg(feature = "_nrf_ble")]
    embassy_nrf::pac::WDT
        .rr(0)
        .write(|w| w.set_rr(embassy_nrf::pac::wdt::vals::Rr::Reload));
}
