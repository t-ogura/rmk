//! Passive scanning, shared by the split central and the dongle.

use bt_hci::cmd::le::LeSetScanParams;
use bt_hci::controller::ControllerCmdSync;
use embassy_time::{Duration, Timer};
use trouble_host::prelude::*;

/// Split central: the radio is shared with the host link and the peripheral
/// connections, so it can spare only 30ms of every 100ms.
#[cfg(feature = "split")]
pub(crate) const SPLIT_CENTRAL_SCAN_WINDOW: Duration = Duration::from_millis(30);

/// Dongle: USB-powered with a single link, so favor fast discovery.
#[cfg(feature = "dongle")]
pub(crate) const DONGLE_SCAN_WINDOW: Duration = Duration::from_millis(60);

/// The split central's scan while it sleeps: 30ms of every 600ms (5%).
///
/// A sleeping central used to stop looking for its halves altogether, so a
/// half that woke up after the central went idle could not rejoin until
/// something on the central itself (a pointing device, a host event) woke it
/// — on a central without keys of its own, possibly never. This keeps a
/// reconnect window open at a twentieth of the awake radio time: a half
/// advertising every 50ms is caught in about 3s on average, well inside the
/// 10s it spends advertising directed at the central after a key press.
#[cfg(feature = "split")]
pub(crate) fn sleep_scan_config() -> ScanConfig<'static> {
    ScanConfig {
        active: false,
        interval: Duration::from_millis(600),
        window: Duration::from_millis(30),
        ..Default::default()
    }
}

/// A passive scan listening for `window` out of every 100ms.
pub(crate) fn scan_config(window: Duration) -> ScanConfig<'static> {
    ScanConfig {
        active: false,
        interval: Duration::from_millis(100),
        window,
        ..Default::default()
    }
}

/// Start a scan, retrying while the controller refuses one — it does until a
/// previous connect's initiator has stopped.
///
/// End the session with [`ScanSession::stop`], which waits for the controller to
/// confirm. Dropping it only signals the cancel, and the controller refuses an
/// initiator until the runner has issued the stop.
pub(crate) async fn start_scan<'a, C: Controller + ControllerCmdSync<LeSetScanParams>>(
    stack: &'a Stack<'_, C, DefaultPacketPool>,
    window: Duration,
    filter_accept_list: &[Address],
) -> ScanSession<'a, false> {
    let config = ScanConfig {
        filter_accept_list,
        ..scan_config(window)
    };
    loop {
        let mut central = stack.central();
        match Scanner::new(&mut central).scan(&config).await {
            Ok(session) => return session,
            Err(_) => Timer::after_millis(500).await,
        }
    }
}
