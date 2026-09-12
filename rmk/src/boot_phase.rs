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

/// When each phase was reached, in milliseconds since the timer started.
///
/// Kept in RAM that a reset does not clear (`.uninit`; power-on does), with a
/// boot counter, so a board that restarts during boot still shows the
/// attempts that came before the one that finally got its USB up. Boot-time
/// log lines themselves are lost -- the USB logger only exists once the tasks
/// run -- which is why this is recorded and logged later by [`log_timeline`].
const TIMELINE_LEN: usize = 48;
const TIMELINE_MAGIC: u32 = 0x424F_4F54; // "BOOT"

#[repr(C)]
struct Timeline {
    magic: u32,
    /// Boots seen since power-on (the current one included).
    boots: u32,
    len: u32,
    /// `(boot, phase, ms)`; `boot` counts from 1.
    entries: [(u32, u8, u32); TIMELINE_LEN],
}

#[cfg_attr(
    all(target_arch = "arm", target_os = "none"),
    unsafe(link_section = ".uninit.rmk_boot_timeline")
)]
static mut TIMELINE: core::mem::MaybeUninit<Timeline> = core::mem::MaybeUninit::uninit();
static TIMELINE_LOCK: embassy_sync::blocking_mutex::Mutex<crate::RawMutex, ()> =
    embassy_sync::blocking_mutex::Mutex::new(());
/// Cleared at every boot (it lives in `.bss`), unlike `TIMELINE`.
static THIS_BOOT_STARTED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// Run `f` on the timeline, initialising it first when this is a new boot or
/// the memory holds nothing recognisable.
fn with_timeline<R>(f: impl FnOnce(&mut Timeline) -> R) -> R {
    TIMELINE_LOCK.lock(|_| {
        // SAFETY: every access goes through this lock, and the static is only
        // ever read after it has been initialised below.
        let t = unsafe { &mut *core::ptr::addr_of_mut!(TIMELINE) };
        if !THIS_BOOT_STARTED.swap(true, Ordering::AcqRel) {
            // SAFETY: reading possibly-uninitialised memory as plain integers;
            // any garbage fails the magic check and is overwritten.
            let valid = unsafe { (*t.as_ptr()).magic } == TIMELINE_MAGIC
                && unsafe { (*t.as_ptr()).len } as usize <= TIMELINE_LEN;
            if valid {
                let t = unsafe { t.assume_init_mut() };
                t.boots = t.boots.wrapping_add(1);
            } else {
                t.write(Timeline {
                    magic: TIMELINE_MAGIC,
                    boots: 1,
                    len: 0,
                    entries: [(0, 0, 0); TIMELINE_LEN],
                });
            }
        }
        f(unsafe { t.assume_init_mut() })
    })
}

/// Record that `phase` has been reached now. Firmware may call this for its
/// own milestones too; values above [`BLE_READY`] are the firmware's to define.
pub fn stamp(phase: u8) {
    BOOT_PHASE.store(phase, Ordering::Release);
    let ms = embassy_time::Instant::now().as_millis() as u32;
    with_timeline(|t| {
        let i = t.len as usize;
        if i < TIMELINE_LEN {
            t.entries[i] = (t.boots, phase, ms);
            t.len += 1;
        } else {
            // Full: drop the oldest so the latest boots stay visible.
            t.entries.copy_within(1.., 0);
            t.entries[TIMELINE_LEN - 1] = (t.boots, phase, ms);
        }
    });
}

/// Log the recorded timeline: one line per phase, tagged with its boot
/// number, with the gap from the previous phase of the same boot. Cheap
/// enough to repeat, which a USB logger needs: it buffers only a little and
/// the host may open the port at any time.
pub fn log_timeline() {
    let (boots, len, entries) = with_timeline(|t| (t.boots, t.len as usize, t.entries));
    info!("boot: this is boot #{} since power-on", boots);
    let mut prev: Option<(u32, u32)> = None;
    for &(boot, phase, ms) in entries[..len].iter() {
        let gap = match prev {
            Some((b, p)) if b == boot => ms.saturating_sub(p),
            _ => ms,
        };
        info!("boot #{}: phase {} at {} ms (+{} ms)", boot, phase, ms, gap);
        prev = Some((boot, ms));
    }
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
