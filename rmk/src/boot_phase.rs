//! A timestamped list of the milestones boot passes, for builds that cannot
//! watch it happen.
//!
//! Nothing a keyboard logs before its USB transport exists reaches a host, and
//! on a board with no debug probe that is the whole of boot. So each milestone
//! records the time it was reached, and [`log_timeline`] prints the list once
//! there is somewhere to print it.
//!
//! Stamps are free (a store into `.bss`) and the list is 32 entries, so this
//! stays compiled in; only the caller of `log_timeline` decides whether it is
//! ever read.

use core::cell::Cell;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_sync::blocking_mutex::Mutex;

/// Chip, radio and USB driver are up; the first initializer has run.
pub const CHIP_INIT: u8 = 0;
/// Entering storage initialisation.
pub const STORAGE_ENTER: u8 = 1;
/// The stored config has been checked against this build.
pub const STORAGE_CHECKED: u8 = 2;
/// Storage is initialised, including any layout sync.
pub const STORAGE_DONE: u8 = 3;
/// Keymap and behavior have been read back.
pub const KEYMAP_LOADED: u8 = 4;
/// The USB transport is being built.
pub const USB_READY: u8 = 5;
/// The BLE transport is being built; the task join follows.
pub const BLE_READY: u8 = 6;

/// The furthest milestone reached, for anything that wants it live.
pub static BOOT_PHASE: AtomicU8 = AtomicU8::new(CHIP_INIT);

/// Milestones kept. Boot passes seven; the rest are for callers above RMK.
const ENTRIES: usize = 32;

struct Timeline {
    len: usize,
    entries: [(u8, u32); ENTRIES],
}

static TIMELINE: Mutex<crate::RawMutex, Cell<Timeline>> = Mutex::new(Cell::new(Timeline {
    len: 0,
    entries: [(0, 0); ENTRIES],
}));

/// Record that `phase` has been reached, with the time on `embassy_time`'s
/// clock. Values above [`BLE_READY`] are the caller's to define.
pub fn stamp(phase: u8) {
    BOOT_PHASE.store(phase, Ordering::Release);
    let now = embassy_time::Instant::now().as_millis() as u32;
    TIMELINE.lock(|cell| {
        let mut t = cell.replace(Timeline {
            len: 0,
            entries: [(0, 0); ENTRIES],
        });
        if t.len < ENTRIES {
            t.entries[t.len] = (phase, now);
            t.len += 1;
        }
        cell.set(t);
    });
}

/// Print the milestones and the gaps between them.
///
/// Call it from somewhere that runs after the transports exist -- a processor's
/// poll, say -- and repeat it: a log buffer holds little, and a serial port can
/// be opened at any time.
pub fn log_timeline() {
    let (len, entries) = TIMELINE.lock(|cell| {
        let t = cell.replace(Timeline {
            len: 0,
            entries: [(0, 0); ENTRIES],
        });
        let copy = (t.len, t.entries);
        cell.set(t);
        copy
    });
    let mut previous = 0u32;
    for &(phase, ms) in entries.iter().take(len) {
        info!(
            "boot: phase {} at {} ms (+{} ms)",
            phase,
            ms,
            ms.saturating_sub(previous)
        );
        previous = ms;
    }
}
