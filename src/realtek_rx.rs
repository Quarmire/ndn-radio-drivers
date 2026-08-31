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

/// **Jaguar1 (`phy_status_rpt_8812`) per-path RSSI**: `(gain_trsw[i] & 0x7F) - 110`.
///
/// ★ Two things distinguish this from [`rssi_dbm`], and both were wrong on the RTL8812AU:
///
/// 1. **The byte.** In `struct phy_status_rpt_8812` byte 0 is `gain_trsw[0]` (**path A**) and
///    byte 1 is `gain_trsw[1]` (**path B**). The 8812AU decode read byte 1 — the second chain —
///    and called it "path-A pwdb".
/// 2. **The TRSW mask.** The top bit of that byte is the T/R switch state, not gain. Unmasked, a
///    switched path reads >= 128, `(128 + gain) - 110` comes out POSITIVE, and the `[-110, 0]`
///    clamp turns it into a perfectly plausible **0 dBm**. A silent, sane-looking wrong answer.
///
/// The `- 110` constant is **correct for Jaguar1** — retracting an earlier suspicion in this tree
/// that it was inherited from the 8733BU. It is confirmed independently by the 802.11k table
/// (`11k_dbm{-92..-50}` against `11k_gain_idx{18..60}` is exactly `idx - 110` at all 11 points)
/// and by `IGI_2_DBM(igi) = igi - 110`. One offset unifies RSSI, IGI and EDCCA on this generation.
///
/// Kept separate from [`rssi_dbm`] rather than replacing it: the jgr2/jgr3 backends read a genuine
/// `pwdb` byte whose interpretation is right for their generation.
pub fn jaguar1_path_rssi_dbm(gain_trsw: u8) -> i8 {
    (i16::from(gain_trsw & 0x7f) - 110).clamp(-110, 0) as i8
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
    /// ☠ The failure this decode existed to fix produced a **plausible** answer, not an obvious
    /// one: with TRSW set the raw byte is >= 128, `(128 + gain) - 110` is positive, and the
    /// `[-110, 0]` clamp reports a clean **0 dBm** — a strong-signal reading indistinguishable
    /// from a real one. That is why it survived: nothing looked broken.
    #[test]
    fn trsw_unmasked_silently_reports_zero_dbm() {
        let gain = 30u8; // a genuine ~-80 dBm receive
        assert_eq!(jaguar1_path_rssi_dbm(gain), -80);
        // Same sample with the T/R switch bit set — the mask is what saves it.
        let with_trsw = gain | 0x80;
        assert_eq!(
            jaguar1_path_rssi_dbm(with_trsw),
            -80,
            "TRSW must be masked off"
        );
        // What the old path did: no mask, so it clamps to a believable 0 dBm.
        assert_eq!(rssi_dbm(with_trsw), 0, "documents the old wrong behaviour");
    }

    /// The `-110` offset is correct for Jaguar1 — the 802.11k gain-index table anchors it at
    /// eleven points (`11k_gain_idx{18..60}` -> `11k_dbm{-92..-50}`), i.e. exactly `idx - 110`.
    #[test]
    fn the_minus_110_offset_matches_the_802_11k_table() {
        for (idx, dbm) in [(18u8, -92i8), (22, -88), (30, -80), (42, -68), (60, -50)] {
            assert_eq!(jaguar1_path_rssi_dbm(idx), dbm, "gain index {idx}");
        }
    }

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
