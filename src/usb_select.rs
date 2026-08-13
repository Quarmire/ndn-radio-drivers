//! Selecting **one** USB dongle when several identical ones share a product id, plus a guard
//! against claiming the device that currently carries a live kernel link.
//!
//! Both Realtek backends need this: a node can have two `0bda:a81a` (RTL8812EU) dongles, one
//! running the kernel Wi-Fi mesh and one spare for a named-data-radio face. `open()` /
//! `open_pid()` claim the *first* enumerated match — which is whichever the kernel bound first —
//! so a plain open can grab the live link and drop the node off its mesh. This module lets a
//! caller pin a specific radio by enumeration index or, robustly, by USB bus:port topology.
//!
//! ## The selector is USB-specific by design, but the config surface is not
//!
//! A radio is just a backend, and "which physical device" is a backend concern. Higher layers pass
//! a **backend-agnostic device string** (`RadioDeviceConfig.address`, or `NDN_RADIO_DEV`); this USB
//! backend interprets it via [`DeviceSelect::parse`] — `"1-1.4"` is a bus:port, `"#<n>"` an index. A
//! serial backend would read the same field as a device path. So adding another backend needs no
//! change here and no USB vocabulary in the shared config.
//!
//! ## Coexistence with a kernel-driven sibling
//!
//! Claiming one dongle via libusb (`set_auto_detach_kernel_driver(true)` + `claim_interface`) detaches
//! the kernel driver from **that device only** — the sibling `0bda:a81a` stays bound to `rtl88x2eu`
//! and keeps carrying the kernel Wi-Fi mesh. So a node can run its fleet mesh on one dongle and a
//! named-radio monitor face on the other at the same time; that is the supported configuration.
//!
//! The failure it guards against is claiming the *wrong* one. [`check_live_link`] looks up the target
//! device's kernel netdev `operstate` in sysfs before claiming: if it is `up` it **warns** (and, with
//! `NDN_GUARD_LIVE_LINK=1`, **refuses**), so an automated bring-up never silently drops the live mesh —
//! it tells the operator to pin the spare with `address`/`NDN_USB_ADDR`. Warn-by-default keeps the
//! single-radio case (where the intended radio may legitimately be up) working without opt-out.
//!
//! Note on release: libusb re-attaches the kernel driver when the handle drops, but the netdev may
//! come back `operstate=down`/unconfigured and need a `ip link set … up` (or a NetworkManager cycle)
//! to rejoin the cell. Claiming is not a transparent, self-reversing borrow of the kernel's radio —
//! dedicate the spare rather than time-sharing the mesh radio.

use rusb::{Context, Device, UsbContext};

use crate::FaceError;

fn usb_err(e: rusb::Error) -> FaceError {
    FaceError::Io(std::io::Error::other(e))
}

/// How to pick among several dongles sharing one product id.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum DeviceSelect {
    /// The first matching device in enumeration order — the historical behaviour.
    #[default]
    First,
    /// The Nth match (0-based) in enumeration order. Simple, but **not stable** across reboots or
    /// hotplug — libusb enumeration order is not guaranteed. Prefer [`Addr`](DeviceSelect::Addr).
    Index(usize),
    /// A specific USB topological address `"<bus>-<port>[.<port>…]"` (e.g. `"1-1.4"`), matching the
    /// Linux sysfs device path. Stable across reboots as long as the dongle stays in the same
    /// physical port — the right selector for pinning "the spare dongle" in a config.
    Addr(String),
}

impl DeviceSelect {
    /// Parse a backend-agnostic device string (as carried by a config's `address` field) into a USB
    /// selector: `"#<n>"` is an enumeration index; anything else non-empty is a USB bus:port address
    /// (`"1-1.4"`). Empty / whitespace ⇒ [`First`](DeviceSelect::First). This is the seam that keeps
    /// USB an implementation detail of *this* backend — the config layer only knows "a device string".
    pub fn parse(s: &str) -> Self {
        let s = s.trim();
        if s.is_empty() {
            return DeviceSelect::First;
        }
        if let Some(idx) = s.strip_prefix('#') {
            if let Ok(i) = idx.trim().parse() {
                return DeviceSelect::Index(i);
            }
        }
        DeviceSelect::Addr(s.to_string())
    }

    /// Build a selector from the environment: `NDN_RADIO_DEV` (a device string parsed by
    /// [`parse`](Self::parse)) wins, then the USB-specific `NDN_USB_ADDR` / `NDN_USB_INDEX` kept for
    /// back-compat. Nothing set ⇒ [`First`](DeviceSelect::First).
    pub fn from_env() -> Self {
        if let Some(d) = std::env::var_os("NDN_RADIO_DEV") {
            return DeviceSelect::parse(&d.to_string_lossy());
        }
        if let Some(a) = std::env::var_os("NDN_USB_ADDR") {
            return DeviceSelect::Addr(a.to_string_lossy().into_owned());
        }
        if let Some(i) = std::env::var("NDN_USB_INDEX").ok().and_then(|s| s.trim().parse().ok()) {
            return DeviceSelect::Index(i);
        }
        DeviceSelect::First
    }

    /// Human-readable form for error messages.
    pub fn describe(&self) -> String {
        match self {
            DeviceSelect::First => "first match".into(),
            DeviceSelect::Index(i) => format!("index {i}"),
            DeviceSelect::Addr(a) => format!("usb-addr {a}"),
        }
    }

    /// Does a candidate at enumeration `index` with USB address `addr` satisfy this selector? The
    /// pure core of [`select_device`], factored out so it is testable without hardware.
    fn matches_at(&self, index: usize, addr: &str) -> bool {
        match self {
            DeviceSelect::First => true,
            DeviceSelect::Index(i) => index == *i,
            DeviceSelect::Addr(a) => addr == a,
        }
    }
}

/// The USB topological address of `device` as `"<bus>-<port>[.<port>…]"` — the Linux sysfs path
/// convention (e.g. `1-1.4`). Root-hub devices with no port path render as `"<bus>-0"`.
pub fn usb_addr(device: &Device<Context>) -> String {
    let bus = device.bus_number();
    match device.port_numbers() {
        Ok(ports) if !ports.is_empty() => {
            let path = ports.iter().map(u8::to_string).collect::<Vec<_>>().join(".");
            format!("{bus}-{path}")
        }
        _ => format!("{bus}-0"),
    }
}

/// Walk the USB device list and return the one matching product id `pids.contains(pid)` selected by
/// `sel`. `label` names the chip family for logs/errors (e.g. `"RTL8822E"`). Runs the live-link
/// guard ([`check_live_link`]) on the chosen device before returning it.
pub fn select_device(
    pids: &[u16],
    vendor_id: u16,
    sel: &DeviceSelect,
    label: &str,
) -> Result<Device<Context>, FaceError> {
    let context = Context::new().map_err(usb_err)?;
    let mut seen = 0usize;
    let mut candidates = 0usize;
    for device in context.devices().map_err(usb_err)?.iter() {
        let desc = device.device_descriptor().map_err(usb_err)?;
        if desc.vendor_id() != vendor_id || !pids.contains(&desc.product_id()) {
            continue;
        }
        let addr = usb_addr(&device);
        let this = seen;
        candidates += 1;
        tracing::info!(
            target: "named_radio",
            chip = label, candidate = this, usb_addr = %addr,
            pid = format_args!("0x{:04x}", desc.product_id()), want = %sel.describe(),
            "USB candidate",
        );
        let hit = sel.matches_at(this, &addr);
        seen += 1;
        if hit {
            check_live_link(&device, label)?;
            return Ok(device);
        }
    }
    Err(FaceError::Io(std::io::Error::new(
        std::io::ErrorKind::NotFound,
        format!(
            "no {label} (vid {vendor_id:#06x}) at {} ({candidates} candidate(s) present)",
            sel.describe()
        ),
    )))
}

/// **Live-link guard.** If the device about to be claimed currently backs a kernel netdev whose
/// `operstate` is `up`, claiming it (which detaches the kernel driver) would drop that link. This
/// **always warns**; with `NDN_GUARD_LIVE_LINK=1` set it **refuses** (returns an error) instead, so
/// an automated fleet bring-up can hard-guard the active mesh radio. Linux-only; a no-op elsewhere.
pub fn check_live_link(device: &Device<Context>, label: &str) -> Result<(), FaceError> {
    #[cfg(target_os = "linux")]
    if let Some((iface, state)) = live_netdev(device) {
        if state == "up" {
            let addr = usb_addr(device);
            if std::env::var_os("NDN_GUARD_LIVE_LINK").is_some() {
                return Err(FaceError::Io(std::io::Error::new(
                    std::io::ErrorKind::Other,
                    format!(
                        "{label} at {addr} currently carries an UP kernel netdev ({iface}); refusing \
                         to claim it (NDN_GUARD_LIVE_LINK is set). Pick the spare via NDN_USB_ADDR / \
                         usb-addr, or bring {iface} down first."
                    ),
                )));
            }
            tracing::warn!(
                target: "named_radio",
                chip = label, usb_addr = %addr, iface = %iface,
                "claiming a dongle whose kernel netdev is UP — the kernel link on {iface} will drop; \
                 set NDN_USB_ADDR/usb-addr to pin the spare, or NDN_GUARD_LIVE_LINK=1 to refuse",
            );
        }
    }
    let _ = (device, label);
    Ok(())
}

/// The `(iface, operstate)` of the kernel netdev backed by this USB device, if any. The netdev lives
/// under the USB **interface** sysfs dir (`<bus>-<port>:<cfg>.<intf>/net/<iface>`), so scan the
/// device tree for an entry whose name is the device address plus a `:` interface suffix.
#[cfg(target_os = "linux")]
fn live_netdev(device: &Device<Context>) -> Option<(String, String)> {
    let prefix = format!("{}:", usb_addr(device));
    let base = std::path::Path::new("/sys/bus/usb/devices");
    for entry in std::fs::read_dir(base).ok()?.flatten() {
        if !entry.file_name().to_string_lossy().starts_with(&prefix) {
            continue;
        }
        let Ok(nets) = std::fs::read_dir(entry.path().join("net")) else {
            continue;
        };
        for net in nets.flatten() {
            let iface = net.file_name().to_string_lossy().into_owned();
            let state = std::fs::read_to_string(net.path().join("operstate"))
                .map(|s| s.trim().to_string())
                .unwrap_or_default();
            return Some((iface, state));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_maps_index_and_addr_forms() {
        assert_eq!(DeviceSelect::parse(""), DeviceSelect::First);
        assert_eq!(DeviceSelect::parse("  "), DeviceSelect::First);
        assert_eq!(DeviceSelect::parse("#0"), DeviceSelect::Index(0));
        assert_eq!(DeviceSelect::parse("#3"), DeviceSelect::Index(3));
        assert_eq!(DeviceSelect::parse("1-1.4"), DeviceSelect::Addr("1-1.4".into()));
        // A "#" with a non-numeric tail is not an index — treat the whole thing as an address
        // rather than silently selecting the first device.
        assert_eq!(DeviceSelect::parse("#x"), DeviceSelect::Addr("#x".into()));
    }

    #[test]
    fn matches_at_selects_the_intended_candidate() {
        // First accepts whatever it sees first.
        assert!(DeviceSelect::First.matches_at(0, "1-1.1"));
        // Index picks exactly the Nth candidate.
        let by_index = DeviceSelect::Index(1);
        assert!(!by_index.matches_at(0, "1-1.1"));
        assert!(by_index.matches_at(1, "1-1.4"));
        assert!(!by_index.matches_at(2, "1-1.5"));
        // Addr picks by USB topology regardless of enumeration order — the stable selector.
        let by_addr = DeviceSelect::Addr("1-1.4".into());
        assert!(!by_addr.matches_at(0, "1-1.1"));
        assert!(by_addr.matches_at(9, "1-1.4"));
    }
}
