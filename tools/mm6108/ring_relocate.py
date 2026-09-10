#!/usr/bin/env python3
"""Move the MM6108 in-chip timestamp ring OUT of the live MPE program buffer.

☠ THE HAZARD THIS FIXES. The four-site timestamp ring was placed at 0x8020AA80, which is 0x5C
bytes INSIDE the MAC's data-TX MPE program buffer (descriptor at 0x8020B95C reads
start=0x8020AA24 end=0x8020B834, i.e. the ring sat wholly inside a 3600-byte live allocation,
about 28 bytes above the program cursor's measured high-water mark of 0x8020AA64). Every frame
that built a slightly longer program would have silently corrupted the instrument, the program,
or both.

WHERE IT GOES, AND WHY THAT CELL. Measured, not reasoned:
  * the mapped MAC RAM window ends between 0x8020C900 and 0x80210000 (do NOT bisect that by
    probing -- see the warning below);
  * 0x8020CE00..0x80210000 was painted with zeros and then subjected to 10984 TX frames of
    1400 B from this node plus a second node transmitting and morse_cli traffic. Afterwards
    ONLY 0x8020FF00..0x80210000 (the top 256 B) was non-zero: that is the MAC core's stack,
    growing down from the top of RAM. 12288 bytes below it stayed byte-for-byte zero.
  * so the instrument moves to the BOTTOM of that proven-free region, as far from the stack as
    the region allows.

  old                              new                       margin
  ctrl   0x8020A0D8  (live .bss)   0x8020D0D8                2212 B above the MPE buffer's end
  counts 0x8020A0DC..F4            0x8020D0DC..F4
  ring   0x8020AA80  (IN the MPE   0x8020DA80..0x8020E27F    7296 B below the measured stack
                      program buf)                            floor 0x8020FF00

THE PATCH IS 9 BYTES. Both blocks are addressed as `lui a5, <page>` plus a 12-bit displacement,
and the new layout is the old one shifted by exactly +0x3000, so relocating is a one-nibble edit
to each of 9 `lui` immediates -- 5x 0x8020a->0x8020d (ctrl/counters) and 4x 0x8020b->0x8020e
(ring base, which the store reaches as page-0x580). No store displacement is re-encoded, so
there is no way to get an S-type immediate wrong.

⚠⚠ NEVER READ AN UNMAPPED CHIP ADDRESS THROUGH /dev/morse_io. One 64-byte read at 0x80210000
made morse_spi_cmd53_read fail -71 and then EVERY later SPI access returned 0xffffffff --
morse_cli went with it. It looks exactly like dead hardware. `echo spi0.0 >
/sys/bus/spi/drivers/morse_spi/{unbind,bind}` recovers it (and reloads the firmware), but the
unbind DESTROYS mon0 and RESETS the channel to 904.5 MHz / 1 MHz.

⚠ The new cells are ABOVE .bss, so the firmware does NOT zero them at boot -- they hold
power-on noise. Zero them from the host after every flash (`zero.pl 0x8020D000 5120`) or the
first counter read is garbage.

Usage:  ring_relocate.py <in.bin> <out.bin>
"""
import sys, zlib, hashlib

# file offsets of the `lui` words inside .mac_imem (vaddr 0x00120000 -> file 0x16c60)
CTRL = [0x1729C, 0x172E8, 0x1732C, 0x173BC, 0x17408]   # lui a5, 0x8020a
RING = [0x172C0, 0x1730C, 0x173E0, 0x1742C]            # lui a5, 0x8020b
OLD_C, NEW_C = b"\xb7\xa7\x20\x80", b"\xb7\xd7\x20\x80"
OLD_R, NEW_R = b"\xb7\xb7\x20\x80", b"\xb7\xe7\x20\x80"
TRAMPOLINE = range(0x1729C, 0x17434)

def main(src, dst):
    orig = open(src, "rb").read()
    d = bytearray(orig)
    for off, old, new in [(o, OLD_C, NEW_C) for o in CTRL] + [(o, OLD_R, NEW_R) for o in RING]:
        if bytes(d[off:off + 4]) != old:
            raise SystemExit(f"refusing: 0x{off:x} is {bytes(d[off:off+4]).hex()}, "
                             f"expected {old.hex()} -- wrong image or already relocated")
        d[off:off + 4] = new
    diff = [i for i in range(len(orig)) if orig[i] != d[i]]
    if len(diff) != 9 or not all(i in TRAMPOLINE for i in diff):
        raise SystemExit(f"refusing: {len(diff)} bytes changed, or outside the trampoline")
    open(dst, "wb").write(bytes(d))
    print(f"{len(diff)} bytes changed, all inside .mac_imem 0x{TRAMPOLINE.start:x}..0x{TRAMPOLINE.stop:x}")
    print(f"size {len(d)}   crc32 0x{zlib.crc32(bytes(d)) & 0xffffffff:08x}   "
          f"md5 {hashlib.md5(bytes(d)).hexdigest()}")
    print("the driver logs that crc32 on load -- match it, it is the proof the chip got THIS image")

if __name__ == "__main__":
    main(*sys.argv[1:3])
