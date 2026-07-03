//! Userspace USB Wi-Fi monitor-mode driver backends over the `ndn-radio-hal` contract.
//!
//! Split out of `ndn-face-monitor-wifi` so drivers have a dedicated home; each
//! backend implements `FrameIo` + `WifiRadio` against the HAL and does no NDN
//! forwarding.

// Re-export the contract surface the backend modules reference as `crate::…`
// (they were written as modules of ndn-face-monitor-wifi, which re-exported these).
pub use ndn_frame_io::{
    frame, radiotap, BROADCAST, CapturedFrame, DEFAULT_SRC, FaceError, FaceId, FrameFormat,
    FrameIo, InjectFrame, MAX_RELIABLE_MCS, McsDescriptor, McsPolicy, Reach, Reliability,
    TxIntent, WifiRadio, mcs_for_rssi, mcs_phy_rate_bps, name_group_mac, name_group_uni,
};

mod libusb_rtl88xx;
pub use libusb_rtl88xx::{
    CHIP_ID_8822E, ChannelBw, FwVersion, LibUsbRtl88xxBackend, REALTEK_VID, REG_SYS_CFG,
    RTL88XX_PIDS, RfPath,
};
mod rtl8821c;
pub use rtl8821c::{RTL8821CU_PIDS, Rtl8821cuBackend};
mod mt7612;
pub use mt7612::{MT7612U_PIDS, Mt7612uBackend};
mod rtl8812au;
pub use rtl8812au::{ChipInfo, IqkResult, RTL8812AU_PIDS, Rtl8812auBackend};
// RTL8731BU / RTL8733BU (halmac_87xx, 1x1 11ac) — ground-up port, M1 (open +
// reg-I/O + chip-version). REALTEK_VID is already re-exported above.
mod libusb_rtl8733b;
pub use libusb_rtl8733b::{ChipVersion, FW_NIC_8733B, FwHeader, RTL8733B_PIDS, Rtl8733buBackend};

// The control-plane `RadioKnobs` impls for the driver backends. These live with
// the driver types (the trait is from `ndn-radio-hal`, the types are declared
// here) — the orphan rule requires the impl travel with the local type. The
// data-plane `FrameIo`/`WifiRadio` impls live in each backend module.
mod radio_knobs {
    use ndn_radio_hal::{Bandwidth, RadioKnobs};
    use ndn_transport::FaceError;

    impl RadioKnobs for crate::LibUsbRtl88xxBackend {
        fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
            let cbw = match bw {
                Bandwidth::Bw20 => crate::ChannelBw::Bw20,
                Bandwidth::Bw40 => crate::ChannelBw::Bw40,
                Bandwidth::Bw80 => crate::ChannelBw::Bw80,
                Bandwidth::Nb10 => crate::ChannelBw::Nb10,
                Bandwidth::Nb5 => crate::ChannelBw::Nb5,
            };
            crate::LibUsbRtl88xxBackend::set_channel(self, channel, cbw)
        }
        fn set_tx_power(&self, idx: u32) -> Result<(), FaceError> {
            crate::LibUsbRtl88xxBackend::set_tx_power(self, idx)
        }
        fn set_tx_csd(&self, on: bool) -> Result<(), FaceError> {
            crate::LibUsbRtl88xxBackend::set_tx_csd(self, on)
        }
        fn set_edcca_ignore(&self, on: bool) -> Result<(), FaceError> {
            crate::LibUsbRtl88xxBackend::set_edcca_ignore(self, on)
        }
    }

    impl RadioKnobs for crate::Mt7612uBackend {
        fn set_channel(&self, channel: u8, bw: Bandwidth) -> Result<(), FaceError> {
            // Only channel 6 / 20 MHz has been captured + replayed so far. Other
            // channels need the per-channel RF program captured the same way
            // (see docs/RADIO_SUBSYSTEM.md "Adding a channel"). This is the
            // "capability added incrementally" boundary made explicit.
            if channel == 6 && bw == Bandwidth::Bw20 {
                crate::Mt7612uBackend::set_channel_ch6(self)
            } else {
                Err(FaceError::Io(std::io::Error::other(format!(
                    "mt7612u: only ch6/20MHz tuned so far (requested ch{channel}/{bw:?})"
                ))))
            }
        }
        // set_tx_power / set_tx_csd / set_edcca_ignore: default no-ops until the
        // mt76x2 power-table / TXOP-CTRL / ED-CCA registers are ported.
    }
}
