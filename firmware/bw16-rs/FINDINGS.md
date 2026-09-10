# BW16 (RTL8720DN) radio-knob findings

What the Rust firmware can and cannot control, established by measurement: the BW16 injects, an
ESP32-C5 running `firmware/esp32c5-ndn` witnesses, and its per-frame PHY metadata (RSSI, noise, rate
code, PHY format) reports what actually reached the air. Register-level claims are additionally
checked by reading the register back out of the hardware.

| Knob | Mechanism | Result |
|------|-----------|--------|
| Channel + band | `wifi_set_channel` | ✅ 2.4 GHz and 5 GHz, both directions (ch6 / ch36 / ch149) |
| TX rate | `update_mgnt_tx_rate(padapter, MGN)` | ✅ **all 17 rates, 1 Mb/s → HT MCS7** |
| TX power | phydm TXAGC via `config_phydm_write_txagc_8721d` | ✅ **0.274 dB/step, R² = 0.982, 31 dB span** |
| RX RSSI | promisc callback `userdata`, byte 31 | ✅ per-frame dBm |
| RX rate / MCS | promisc callback `userdata`, byte 32 | ✅ per-frame, legacy and HT |
| RX noise floor | — | ❌ not per-frame on this chip (see below) |
| Bandwidth 40 MHz | `wext_set_bw40_enable` | ⚠️ API succeeds; effect on inject unverified |

## Three earlier conclusions were wrong. All three are retracted.

**1. "The rate is not reachable through the management-TX path; it needs a data-path rewrite."**
Wrong. It is reachable, and the fix is one exported symbol.

The reasoning was sound and the experiment was not. `wifi_tx_raw_frame` does inject via the
management queue, and 802.11 management frames *are* conventionally legacy-rate — so probing
`pkt_attrib` for a rate field and finding nothing looked like confirmation. But
`update_mgntframe_attrib` never writes a rate into `pkt_attrib` at all: it memsets the struct and
stores a handful of unrelated constants. The rate is decided later, at descriptor fill.
`rtl8721d_update_txdesc` reads `padapter[0x855]`, converts it with `MRateToHwRate`, writes the result
into the descriptor's DATA_RATE field, and sets the use-fixed-rate bit so hardware rate adaptation
cannot override it. `update_mgnt_tx_rate(padapter, mgn_rate)` — a linkable global symbol, called by
the driver itself — is the setter for that byte.

Two things kept the original probe from finding this. The rate field it swept for is not in
`pkt_attrib`; and even if it had been, the sweep covered offsets 0–56 while `pkt_attrib.rate` sits at
+99. A later sweep of 0–63 targeting a *legacy* 54 Mb/s (detectable, unlike an HT rate that also needs
`ht_en` and `raid` set consistently) likewise found nothing — correctly, because the field is not there.

Also retracted from that probe: "offsets ≥34 went silent = TX break". TX never broke. That was the
RTL8812EU witness wedging; with the C5 witness every offset in 0–63 transmitted normally.

**2. "TX power is a driver stub."** Half right, and the half that was wrong mattered. The *iwpriv
path* is a stub — `txpower` is registered under the private-ioctl **GET** family and its handler
parses the argument and echoes it without writing a register. Sweeping index 0–63 through it moved a
witness's RSSI by 0.6 dB, which is noise, and that was read as "this chip's power is not
controllable". It is: phydm's TXAGC registers (one byte per rate, path A) are directly writable via
`config_phydm_write_txagc_8721d`, reachable through a pointer walk from the exported `rltk_wlan_info`.

**3. "This SDK has no `rtw_rx_info_t`, so per-frame RSSI is unavailable."** Wrong twice over. The
struct is in the shipped header. And the RSSI never needed it: the promiscuous callback's third
argument — which the shim discarded as `(void)ud;` — is an `ieee80211_frame_info_t*` whose byte 31 is
a signed dBm, exactly as the SDK's own `promisc_callback_all` reads it. The blob also writes one byte
*past* the declared struct, at offset 32: `HwRateToMRate(pattrib->data_rate)`, the per-frame PHY rate
as an MGN code. That undeclared byte is where the RX rate and MCS come from.

The common thread: each conclusion came from reasoning about the mechanism rather than reading the
binary that implements it, and each was confirmed by an experiment too blunt to distinguish "the knob
does nothing" from "I am turning the wrong knob".

## Verifying a power change without going on air

`wext_get_tx_power(WLAN0_NAME, idx[20])` is compiled and returns the live TXAGC indices — `[0..4]`
CCK 1/2/5.5/11, `[4..12]` OFDM 6…54, `[12..20]` HT MCS0…7. Exposed as `T_READPOWER`/`T_POWERIDX` and
`SerialRadioBackend::read_txpower`. Use it before trusting any RSSI: it separates "the register
moved" from "the air changed", which a witness alone cannot do. (The SDK's own `wifi_get_txpower` is
both `#if 0`-disabled and buggy — it sscanf's into an 11-byte buffer. Do not use it.)

The DM watchdog reprograms TXAGC from its own tables, so a direct write is stomped unless power
tracking is off; `c_set_txagc` calls `halrf_set_pwr_track(dm, 0)` first. `T_TXPOWER 0xFF` restores the
driver's computed values via `rtl8721d_set_tx_power_level`.

The pointer walk (`rltk_wlan_info` → dev → priv → padapter → pHalData+0x20C8 → dm+0x23C8) is checked
before use: `*(void**)dm` must equal `padapter`, because phydm's own accessors start by dereferencing
`dm` and passing the result to a padapter-taking function. If that check fails, the offsets are wrong
for the SDK build and the firmware writes nothing rather than corrupting memory.

## What this chip genuinely cannot do

* **Per-frame noise floor.** The phy-status parser the 8721D uses never fills the noise or bandwidth
  fields the promiscuous path could carry. Noise is measurable out-of-band via
  `odm_inband_noise_monitor_n`, but it is not a per-frame quantity here, so `T_RX_TS` reports 0 and
  the host does not synthesise an SNR from it.
* **VHT / HE.** The part is 1x1 HT20; the driver's own rate table tops out at HT MCS7 / 65 Mb/s. The
  capability deliberately does not advertise `he_cap`, so cognition never escalates to the HE reach
  levers (ER-SU, DCM) that the C5 can actually emit.
* **A hardware arbitrary-instant scheduled TX.** No P2P-NoA or quiet-time engine is compiled into
  this blob. Scheduled TX is a bounded busy-wait on the SDK µs ticker, which is why its *submission*
  error is a constant 10 µs while its on-air jitter is still whatever CSMA adds.
* **BLE 5 extended advertising, and LE Coded PHY.** Not present in the **controller**, which is what
  binds. Read straight from the silicon: the stack caches the controller's answer to `LE Read Local
  Supported Features` in `gap_local_features` (also reachable as
  `le_get_gap_param(GAP_PARAM_LOCAL_FEATURES)`), and on this part it reads **`3d 01 00 00 00 00 00
  00`** — LE 2M PHY set, LE Coded PHY clear, LE Extended Advertising clear, LE Periodic Advertising
  clear. So advertising is legacy-only: 31 bytes, 27 usable under the `0x4E44` manufacturer envelope,
  against the C5's ~245.

  The host stack was *also* built without extended advertising (`btgap.a` compiled with
  `F_BT_LE_5_0_AE_ADV_SUPPORT = 0`; it exports no ext-adv symbols, and its 31-byte gate is a literal
  `cmp r1, #31` inside `le_adv_set_param`). That one **is** bypassable — `gap_vendor_cmd_req(opcode,
  len, params)` forwards any opcode to the controller verbatim, with no whitelist, OGF check or length
  check on the TX path — so the two explanations had to be told apart rather than assumed.

  They were, three ways, and all three agree the **controller** is the binding limit:

  1. The cached feature mask has bit 12 clear (above).
  2. The full raw-HCI sequence — `LE Set Extended Advertising Parameters` (0x2036, 25 bytes),
     `…Data` (0x2037) and `…Enable` (0x2039) — was issued before any legacy advertising had run, with
     a 64-byte AD. Every command queued successfully and **nothing appeared on air**, watched by a
     host running CoreBluetooth, which was concurrently reporting ~196 other manufacturer adverts.
  3. The latch oracle: a controller that accepts extended advertising rejects the legacy advertising
     commands until reset. Legacy advertising delivered **12/12 both before and after** the attempt —
     so the controller never entered extended mode, i.e. it rejected those commands.

  The controller's own error code cannot be read back, which is why the third check matters: the host's
  Command Complete router forwards only OGF 0x3F (`0xFCxx`+) to the application, and drops OGF 0x08
  entirely for opcodes above 0x2031 — so an ext-adv rejection is invisible from firmware.

  Note the LE 2M PHY bit is no consolation either: on BLE 5 the advertising PHY is selectable only
  through extended advertising, so a legacy advertiser is 1M by specification. Same for LE Coded —
  which this controller lacks anyway, so there is no long-range BLE reach lever on this part.

## The clock

Arduino's `micros()` on this core is `tick*1000 − SysTick_current/200` and **steps backwards** by up
to ~1 ms at a tick boundary. A wrap-extension that treats any decrease as a 32-bit wrap therefore adds
2³² µs on almost every call — which is what made the occupancy report fire ~500×/s instead of 5. The
firmware uses `us_ticker_read()` (which carries its own monotonicity correction) and additionally
counts a decrease as a wrap only when it exceeds half the range, holding the previous value otherwise.
