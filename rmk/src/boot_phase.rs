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
