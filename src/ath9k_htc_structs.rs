#![allow(dead_code)]
//! On-wire HTC header layouts for the AR9271, transcribed from the mainline
//! Linux ath9k driver at kernel tag **v6.12.33**
//! (`drivers/net/wireless/ath/ath9k/htc.h`, and `common.h` where noted).
//!
//! These headers are parsed from big-endian byte slices coming off USB, so the
//! multi-byte fields are kept as raw byte arrays (`__beN` = N big-endian bytes)
//! rather than native integers. That gives `#[repr(C)]` a padding-free layout
//! whose `size_of` equals the exact on-wire size — matching the C `__packed`
//! structs — and makes the compile-time size asserts meaningful. A byte-offset
//! table is given for each so a hand-rolled parser can index without depending
//! on struct field access.

/// `struct tx_frame_hdr` — TX frame header prepended to outbound frames.
/// SOURCE: htc.h:73-83 (fetched, VERBATIM). C is `__packed`.
///
/// Field order / offsets (on-wire, big-endian for __be32):
/// ```text
/// off size field       C type
///   0    1  data_type  u8
///   1    1  node_idx   u8
///   2    1  vif_idx    u8
///   3    1  tidno      u8
///   4    4  flags      __be32   (ATH9K_HTC_TX_*)
///   8    1  key_type   u8
///   9    1  keyix      u8
///  10    1  cookie     u8
///  11    1  pad        u8
/// total: 12 bytes
/// ```
/// VERIFIED: 12 bytes — matches the spec's claim of 12 B.
#[repr(C)]
pub struct TxFrameHdr {
    pub data_type: u8,
    pub node_idx: u8,
    pub vif_idx: u8,
    pub tidno: u8,
    pub flags: [u8; 4], // __be32 ATH9K_HTC_TX_*
    pub key_type: u8,
    pub keyix: u8,
    pub cookie: u8,
    pub pad: u8,
}
const _: () = assert!(core::mem::size_of::<TxFrameHdr>() == 12);

/// `struct ath_htc_rx_status` — RX status header prepended to inbound frames.
///
/// SOURCE NOTE: the fetched `htc.h` only *references* this struct (htc.h:277);
/// its definition lives in ath9k `common.h`, which was NOT among the fetched
/// files. However, the fetched htc.h DOES define the on-wire size directly:
///     `#define HTC_RX_FRAME_HEADER_SIZE 40`  (htc.h:271)
/// so the 40-byte total below is SOURCE-BACKED even though the field breakdown
/// is reconstructed from ath9k common.h (canonical, deterministic under
/// `__packed`). VERIFY the field names/order against common.h before trusting.
///
/// Field order / offsets (on-wire, big-endian for __beN):
/// ```text
/// off size field           C type
///   0    8  rs_tstamp       __be64
///   8    2  rs_datalen      __be16
///  10    1  rs_status       u8
///  11    1  rs_phyerr       u8
///  12    1  rs_rssi         s8
///  13    3  rs_rssi_ctl[3]  s8[3]
///  16    3  rs_rssi_ext[3]  s8[3]
///  19    1  rs_keyix        u8
///  20    1  rs_rate         u8
///  21    1  rs_antenna      u8
///  22    1  rs_more         u8
///  23    1  rs_isaggr       u8
///  24    1  rs_moreaggr     u8
///  25    1  rs_num_delims   u8
///  26    1  rs_flags        u8
///  27    1  rs_dummy        u8
///  28    4  evm0            __be32
///  32    4  evm1            __be32
///  36    4  evm2            __be32
/// total: 40 bytes
/// ```
/// VERIFIED: 40 bytes — equals htc.h HTC_RX_FRAME_HEADER_SIZE and the spec's 40 B.
#[repr(C)]
pub struct AthHtcRxStatus {
    pub rs_tstamp: [u8; 8], // __be64
    pub rs_datalen: [u8; 2], // __be16
    pub rs_status: u8,
    pub rs_phyerr: u8,
    pub rs_rssi: i8,
    pub rs_rssi_ctl: [i8; 3],
    pub rs_rssi_ext: [i8; 3],
    pub rs_keyix: u8,
    pub rs_rate: u8,
    pub rs_antenna: u8,
    pub rs_more: u8,
    pub rs_isaggr: u8,
    pub rs_moreaggr: u8,
    pub rs_num_delims: u8,
    pub rs_flags: u8,
    pub rs_dummy: u8,
    pub evm0: [u8; 4], // __be32
    pub evm1: [u8; 4], // __be32
    pub evm2: [u8; 4], // __be32
}
const _: () = assert!(core::mem::size_of::<AthHtcRxStatus>() == 40);

/// On-wire RX status header size, transcribed directly from htc.h:271.
pub const HTC_RX_FRAME_HEADER_SIZE: usize = 40;
