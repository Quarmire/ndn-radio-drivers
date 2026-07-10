# waveshare-lora-rs

Open **Rust** firmware for the Waveshare USB-TO-LoRa dongle (**GD32F103C8** + **SX1262** + CH343
USB-UART), replacing the closed factory firmware. The stock firmware wraps every LoRa frame in a
proprietary NETID/ADDR header, so it can only talk to its own kind; this firmware speaks **plain
standard LoRa**, so the dongle interoperates with any SX127x/SX126x peer (verified bidirectionally
against a Heltec WiFi-LoRa-32 SX1276 node) and is driven by the host as a clean serial modem.

## Hardware map

| Function | Pin | Notes |
|----------|-----|-------|
| Host UART | USART1 PA9 (TX) / PA10 (RX) | 115200 8N1 → CH343 → `/dev/ttyACM*` (Linux) / `/dev/cu.usbmodem*` (macOS) |
| SX1262 SPI | SPI2 — NSS PB12, SCK PB13, MISO PB14, MOSI PB15 | NSS driven by hand (BUSY handshake) |
| SX1262 RESET | PA4 | |
| SX1262 BUSY | PB1 | |
| SX1262 DIO1 | PB0 | RxDone/TxDone (polled) |
| RF switch | PB4 | **HIGH = RX, LOW = TX**; PB4 is JNTRST → JTAG disabled (SWD kept) |
| TCXO | DIO3, 1.7 V | non-`-B` variant; recalibrated after enable |
| LEDs | PA6 (RXD) / PA7 (TXD) | |

Regulator is **LDO** (not DC-DC); PA is an SX1262 (+22 dBm: paDutyCycle 0x04, hpMax 0x07).

## Host ⇄ firmware serial protocol

Binary framing: `7E A5 | type | len | payload[len] | xor-crc` (crc = XOR of type, len, payload).
The parser resyncs on `7E A5` and validates the crc, so a dropped byte costs at most one frame.

Host → firmware:

| Type | Name | Payload |
|------|------|---------|
| 0x01 | TX | LoRa frame bytes to transmit |
| 0x02 | SET_FREQ | u32 big-endian Hz |
| 0x03 | SET_MOD | `[sf, bw_code, cr_code]` (bw 0x04=125k/0x05=250k/0x06=500k; cr 0x01..0x04 = 4/5..4/8) |
| 0x04 | SET_PWR | `[i8 dBm]` (≤ +22) |
| 0x05 | SET_SYNC | `[sx127x sync byte]` (0x12 private / 0x34 public) |
| 0x06 | GET_INFO | *(empty)* |

Firmware → host:

| Type | Name | Payload |
|------|------|---------|
| 0x81 | RX | `[rssi i16 BE, snr i16 BE, LoRa bytes]` |
| 0x82 | TXDONE | `[ok]` |
| 0x83 | INFO | `[status, sync(2), errors(2), freq(4), sf, bw, cr, pwr]` |
| 0x84 | LOG | ascii |

Defaults: 915 MHz (US) / SF7 / BW125 / CR4-5 / sync 0x12 (→ SX126x reg 0x1424) / preamble 8 /
explicit header / CRC on — matched to the Heltec node. A reference host codec is in the effort's
`/tmp/lora_proto.py`.

## Build & flash

```sh
cargo build --release
arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/waveshare-lora-rs firmware.bin
```

Flash over **ST-Link + openocd** (the chip ships RDP read-protected; the first `stm32f1x unlock 0`
mass-erases the stock firmware). No nRST is broken out, so openocd only attaches in the brief window
right after the ST-Link USB re-enumerates — build first, replug, then flash immediately:

```sh
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "init; reset halt; stm32f1x unlock 0; reset halt; \
      flash write_image erase firmware.bin 0x08000000; verify_image firmware.bin 0x08000000; \
      reset run; exit"
```

JTAG is disabled in software but **SWD is preserved**, so the dongle stays reflashable.
