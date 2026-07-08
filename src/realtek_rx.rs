//! Shared Realtek RX-descriptor decode — the RSSI / MCS / hardware-timestamp extraction common
//! to the USB Realtek backends ([`libusb_rtl8733b`](crate::libusb_rtl8733b),
//! [`libusb_rtl88xx`](crate::libusb_rtl88xx)), so each backend does not re-derive it.
//!
//! Kept here (not in the generic `ndn-frame-io` layer) because the pwdb / DESC_RATE / RXTSFL
//! formats are Realtek-specific — this is the anti-redundancy seam for that chip family. Each
//! backend still owns its descriptor *framing* (offsets, drvinfo size, C2H split); it delegates
//! the *field interpretation* here.

use ndn_frame_io::{ClockDomainId, LatchPoint, LinkStamp};

/// Jaguar-series phystatus path power `pwdb` (a byte the drvinfo/phystatus block reports) to
/// RSSI in dBm: `rx_pwr_dbm = pwdb - 110` (per `phydm_phystatus.c`), clamped to a sane range.
/// RSSI is non-positive for a real receive.
pub fn rssi_dbm(pwdb: u8) -> i8 {
    (i16::from(pwdb) - 110).clamp(-110, 0) as i8
}

/// Realtek RX HwRate (DESC_RATE code) to an 802.11n/ac MCS index. HT MCS0-15
/// (`DESC_RATEMCS0 = 0x0c`), VHT (`DESC_RATEVHTSS1MCS0 = 0x2c`); legacy CCK/OFDM carry no MCS.
pub fn mcs_from_desc_rate(rate: u8) -> Option<u8> {
    if (0x0c..=0x1b).contains(&rate) {
        Some(rate - 0x0c) // HT MCS0-15
    } else if rate >= 0x2c {
        Some((rate - 0x2c) % 10) // VHT MCSx within a stream group
    } else {
        None // legacy CCK / OFDM
    }
}

/// Build a per-frame RX hardware [`LinkStamp`] from a free-run RXTSFL (microseconds) at the
/// MAC-done latch — the always-on per-frame clock ([`RadioClockKind::FreeRunRxStamp`]).
///
/// [`RadioClockKind::FreeRunRxStamp`]: ndn_frame_io::RadioClockKind::FreeRunRxStamp
pub fn rx_stamp(rxtsfl: u32, domain: ClockDomainId) -> LinkStamp {
    LinkStamp::new(
        u64::from(rxtsfl),
        domain,
        LatchPoint::MacDone.precision_floor_ns(),
        LatchPoint::MacDone,
    )
}

// CSI (channel state information): ASSESSED — not available on these Realtek parts. The vendor
// phydm's only CSI is compressed 802.11 *beamforming* feedback (angles for TxBF/MU-MIMO, set up
// in phydm_direct_bf.c via BB 0x72c / 0x19b8[6]); it is computed on-chip and never handed to the
// host as a per-subcarrier H-matrix, and it is N/A on the 1x1 8733b (beamforming needs >=2
// chains). So these backends report `CsiSupport::None`. A host-visible per-subcarrier estimate
// would need a CSI-tool NIC (Atheros/Intel) or an SDR (`CsiSupport::PerSubcarrier`); the coarse
// per-path RSSI/CFO/EVM in the phystatus is the most a Realtek part could offer (`Coarse`), and
// is not decoded here yet. If a future backend gains CSI, it decodes into a shared type here.

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rssi_matches_hw_validated_range() {
        // pwdb 44..37 -> -66..-73 dBm, the values measured on real 8731bu ambient RX.
        assert_eq!(rssi_dbm(44), -66);
        assert_eq!(rssi_dbm(37), -73);
        assert_eq!(rssi_dbm(0), -110); // noise floor, clamped
        assert_eq!(rssi_dbm(255), 0); // absurd high -> clamped non-positive
    }

    #[test]
    fn mcs_maps_ht_vht_and_legacy() {
        assert_eq!(mcs_from_desc_rate(0x04), None); // OFDM 6M (legacy)
        assert_eq!(mcs_from_desc_rate(0x0c), Some(0)); // HT MCS0
        assert_eq!(mcs_from_desc_rate(0x13), Some(7)); // HT MCS7
        assert_eq!(mcs_from_desc_rate(0x2c), Some(0)); // VHT-1SS MCS0
        assert_eq!(mcs_from_desc_rate(0x03), None); // CCK 11M
    }
}
