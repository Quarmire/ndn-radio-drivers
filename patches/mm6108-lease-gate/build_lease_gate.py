#!/usr/bin/env python3
"""
MM6108 NAMED-AIRTIME-LEASE GATE -- firmware patch builder (SOURCE ONLY).

Builds a patched mm6108.bin from a stock image you supply.  We never ship a
patched binary (Morse Micro Binary Distribution Licence): this script + the stock
image is the distributable form.

TARGET IMAGE (the one mds-o5p-2 actually runs):
    len 459124   crc32 0x51d355b9   ELF32 LE RV32IMC
    .mac_imem  vaddr 0x00120000  file 0x016c54  size 146468
    container_off(v) = 0x16c54 + v - 0x120000     (only PT_LOAD bytes reach the chip)
The script REFUSES to run against any other image: every address below was
derived from this build and offsets are build-specific.

WHY THIS GATE, IN ONE PARAGRAPH
    The named airtime lease is slot-rate-limited by  slot >= frame_cost + guard.
    Measured at 908 MHz / 8 MHz / MCS7 / CW=0: frame_cost 710 us (312 us PPDU +
    375 us host->air fixed + 23 us residual backoff) and the host guard must be
    1000 us to contain a host->driver->bus->MAC tail that p99.9 = 1810 us on air
    while the host itself places to 235 us.  So the guard is 58.5% of the slot and
    the PPDU is 18.2%.  No host knob shrinks that tail (SCHED_FIFO is refused,
    CW=0 leaves p99/p99.9 untouched).  Releasing the frame from INSIDE the chip,
    on the chip's own microsecond clock, is the only mechanism that shrinks it.

MECHANISM
    Enforcement point:  the firmware's own TX-admission test.
        0x0012578c  tx_admit(frame):
        0x00125796    auipc s1,0x800db ; addi s1,s1,-0xe6   -> s1 = 0x802006B0
        0x0012579e    lbu   a5,0x4e(s1)     ; = 0x802006FE, MORSE_CMD_PARAM_ID_TX_BLOCK
        0x001257a2    c.mv  s0,a0
        0x001257a4    c.bnez a5, +0xc       ; a5 != 0  ==> refuse
      The refusal path returns -0x8000 and the frame is HELD, not dropped: measured
      on air (chip TX Total +200 / +0 / +200 across block-off/on/off; 293 rx / 0 rx
      / drain burst).  That is the firmware's OWN refuse-and-defer machinery, and
      re-using it is why this gate needs no wait, no lock and no doorbell.
    We displace the single 4-byte `lbu` with a `j` to a trampoline that returns
    a5 = 1 when the chip clock is outside our slot, and otherwise performs the
    original load so the host's tx_block keeps working unchanged.

WHAT WE DELIBERATELY DO NOT DO
    * No busy-wait at the TX doorbell.  0x001433b8 (`c.sw a5,8(a4)`, GO bit of
      0x100a9008) sits in a leaf whose siblings spin unbounded on 0x100a900c bit0,
      and 0x001434e8 tail-jumps into the cross-core spinlock at 0x00120dd4
      (`amoswap.w.aq` on 0x100a7010, also taken by the uPHY core, no watchdog).
      Stalling there is an unrecoverable two-core hang.
    * No hijack of 0x100aa100/104.  The hardware one-shot is shared with the
      firmware's own deadline queue.  (Stage 4 below arms it only via 0x001213d8.)
    * No RAW/RPS engine.  It works, but it needs an AP beacon and an AID; this
      lease is association-free by construction and must stay that way.

MAXIMUM SAFE BLOCK
    A refused frame ages out at the TX lifetime, default 0x00080000 = 524288 us
    (written at 0x00134476 `lui a4,0x80`), counted by tx_lifetime_expired (u16 at
    0x80204FE6).  Our closed interval is at most PERIOD - (OPEN_HI - OPEN_LO);
    at the measured geometry that is ~1.4 ms, a 370x margin.  The builder refuses
    any configuration whose closed interval exceeds 200000 us.

REGISTER DISCIPLINE
    Only a0/a1/a5 may be touched (t0/t1 are caller-saved by the ABI and STILL fatal
    on this image -- measured).  At 0x0012579e a0 is live (consumed at 0x001257a2)
    and s1 is live (our base), so the stubs use a5 freely and save/restore a1 on
    the stack.  16-byte stack adjust keeps the ABI alignment invariant for any
    interrupt that lands mid-stub.
"""
import argparse, struct, sys, zlib

STOCK_LEN, STOCK_CRC = 459124, 0x51d355b9
MAC_FILE, MAC_VADDR  = 0x016c54, 0x00120000
def coff(v): return MAC_FILE + v - MAC_VADDR

# ---------------------------------------------------------------- addresses
SITE_GATE   = 0x0012579E   # lbu a5,0x4e(s1)     bytes 83 c7 e4 04
RET_GATE    = 0x001257A2   # c.mv s0,a0
SITE_PARAM  = 0x00132ED0   # snez a5,a5          bytes b3 37 f0 00
PARAM_LEG   = 0x00132ED4   # auipc a4,0x800ce ; sb a5,-0x7d6(a4)   (legacy tx_block store)
PARAM_DONE  = 0x00132EDC   # c.li a0,0 ; c.jr ra
RUN1, RUN1_LEN = 0x0012063C, 196   # nop run: 49 x `nop`, after `mret` at 0x00120638
RUN2, RUN2_LEN = 0x0012048A, 118   # zero run, previously exercised to 84 B
DATA        = 0x001206F0   # 16 B config: +0 RHO  +4 PERIOD  +8 OPEN_LO  +12 OPEN_HI
DATA_HI, DATA_LO = 0x120, 0x6F0
TEL         = 0x80209B6C   # cb+0x1c, RAW control block spare word -- host-readable via
                           # /dev/morse_io (.mac_dmem reads are safe; .mac_imem reads WEDGE)
TEL_HI, TEL_LO = 0x8020A, -0x494
MTIME_HI, MTIME_LO = 0x200C, -8    # CLINT mtime lo @ 0x0200BFF8, 1 tick == 1 us (measured)
TXBLOCK_OFF = 0x4E                 # 0x802006FE relative to s1 = 0x802006B0

ORIG = {SITE_GATE:  bytes.fromhex('83c7e404'),
        SITE_PARAM: bytes.fromhex('b337f000')}

# ---------------------------------------------------------------- encoder
RG = dict(zero=0, sp=2, s1=9, a0=10, a1=11, a5=15)
def _i(op,f3,rd,rs1,imm): return (imm&0xfff)<<20|RG[rs1]<<15|f3<<12|RG[rd]<<7|op
def _s(op,f3,rs1,rs2,imm):
    i=imm&0xfff; return (i>>5)<<25|RG[rs2]<<20|RG[rs1]<<15|f3<<12|(i&0x1f)<<7|op
def _r(op,f3,f7,rd,a,b): return f7<<25|RG[b]<<20|RG[a]<<15|f3<<12|RG[rd]<<7|op
def _b(op,f3,a,b,imm):
    i=imm&0x1fff
    return ((i>>12)&1)<<31|((i>>5)&0x3f)<<25|RG[b]<<20|RG[a]<<15|f3<<12|((i>>1)&0xf)<<8|((i>>11)&1)<<7|op
def _u(op,rd,i): return (i&0xfffff)<<12|RG[rd]<<7|op
def _j(op,rd,imm):
    i=imm&0x1fffff
    return ((i>>20)&1)<<31|((i>>1)&0x3ff)<<21|((i>>11)&1)<<20|((i>>12)&0xff)<<12|RG[rd]<<7|op
LUI  =lambda rd,i:_u(0x37,rd,i);            ADDI =lambda rd,s,i:_i(0x13,0,rd,s,i)
SLLI =lambda rd,s,i:_i(0x13,1,rd,s,i);      SRLI =lambda rd,s,i:_i(0x13,5,rd,s,i)
LW   =lambda rd,s,i:_i(0x03,2,rd,s,i);      LBU  =lambda rd,s,i:_i(0x03,4,rd,s,i)
SW   =lambda b,v,i:_s(0x23,2,b,v,i)
SUB  =lambda rd,a,b:_r(0x33,0,0x20,rd,a,b); ADD =lambda rd,a,b:_r(0x33,0,0,rd,a,b)
REMU =lambda rd,a,b:_r(0x33,7,1,rd,a,b);    SNEZ=lambda rd,s:_r(0x33,3,0,rd,'zero',s)
BEQ  =lambda a,b,o:_b(0x63,0,a,b,o);        BLTU=lambda a,b,o:_b(0x63,6,a,b,o)
BGEU =lambda a,b,o:_b(0x63,7,a,b,o);        J   =lambda o:_j(0x6f,'zero',o)

def link(base, prog):
    labels, pc = {}, base
    for lab,_ in prog:
        if lab: labels[lab] = pc
        pc += 4
    out, pc = [], base
    for _,fn in prog: out.append(fn(pc, labels)); pc += 4
    return b"".join(struct.pack('<I', w) for w in out)

# ---------------------------------------------------------------- stage 1: OBSERVE ONLY
# Changes nothing the radio does.  Answers three questions in one image:
#   (1) does the hook fire 1:1 with TX admissions?  compare against total_tx_packets
#       (0x80204FC0) -- the firmware's own per-frame counter.
#   (2) is cb+0x1c genuinely a cell the firmware never writes?  it must read 0 before
#       the patch and must then carry ONLY our values.
#   (3) is .mac_imem data-writable from the MAC core?  (decides where config lives)
# Read-back at 0x80209B6C discriminates all three:
#   0x00000000 constant -> hook never fired, or the cell is unreadable
#   0x5A710000 constant -> hook fires, imem store did NOT take   (use the dmem fallback)
#   0x5A710001,2,3...   -> hook fires AND imem is writable       (primary layout)
# Also THE decisive measurement for the whole design: with the host holding
# `morse_cli tx_block 1`, this counter still advances on every admission ATTEMPT, so
# its rate is the firmware's re-drive cadence.  The gate can only be as sharp as that
# cadence; if it is slower than ~100 us, stage 4 (timer kick) is mandatory.
IMEM_SEED = 0x5A710000
def prog_observe():
    P=[]; a=lambda l,f: P.append((l,f))
    a('OBS', lambda pc,L: ADDI('sp','sp',-16))
    a(None,  lambda pc,L: SW('sp','a1',0))
    a(None,  lambda pc,L: LUI('a1',DATA_HI))
    a(None,  lambda pc,L: LW('a5','a1',DATA_LO))          # imem counter
    a(None,  lambda pc,L: ADDI('a5','a5',1))
    a(None,  lambda pc,L: SW('a1','a5',DATA_LO))          # store into .mac_imem
    a(None,  lambda pc,L: LW('a5','a1',DATA_LO))          # read it straight back
    a(None,  lambda pc,L: LUI('a1',TEL_HI))
    a(None,  lambda pc,L: SW('a1','a5',TEL_LO))           # publish -> cb+0x1c
    a(None,  lambda pc,L: LW('a1','sp',0))
    a(None,  lambda pc,L: ADDI('sp','sp',16))
    a(None,  lambda pc,L: LBU('a5','s1',TXBLOCK_OFF))     # the displaced instruction
    a(None,  lambda pc,L: J(RET_GATE-pc))
    return P

# ---------------------------------------------------------------- stage 3: THE GATE
# open  <=>  OPEN_LO <= ((mtime_lo - RHO) mod PERIOD) < OPEN_HI
# PERIOD == 0 is the OFF switch and the shipped default, so the actuating image is
# byte-for-byte inert until the host writes PERIOD.  OPEN_HI == 0 also fails open.
# NOTE: mtime_lo is 32 bits and wraps every 4295 s.  Choose PERIOD a power of two
# (1024/2048/4096/8192 us) and the wrap is exact and invisible; otherwise the grid
# phase steps by (2^32 mod PERIOD) once every 71.6 minutes.
def prog_gate():
    P=[]; a=lambda l,f: P.append((l,f))
    a('GATE',lambda pc,L: ADDI('sp','sp',-16))
    a(None,  lambda pc,L: SW('sp','a1',0))
    a(None,  lambda pc,L: LUI('a1',MTIME_HI))
    a(None,  lambda pc,L: LW('a5','a1',MTIME_LO))                 # a5 = chip us clock
    a(None,  lambda pc,L: LUI('a1',DATA_HI))
    a(None,  lambda pc,L: LW('a1','a1',DATA_LO+0))                # RHO
    a(None,  lambda pc,L: SUB('a5','a5','a1'))
    a(None,  lambda pc,L: LUI('a1',DATA_HI))
    a(None,  lambda pc,L: LW('a1','a1',DATA_LO+4))                # PERIOD (0 = gate off)
    a(None,  lambda pc,L: BEQ('a1','zero',L['OPEN']-pc))
    a(None,  lambda pc,L: REMU('a5','a5','a1'))                   # phase in [0,PERIOD)
    a(None,  lambda pc,L: LUI('a1',DATA_HI))
    a(None,  lambda pc,L: LW('a1','a1',DATA_LO+8))                # OPEN_LO
    a(None,  lambda pc,L: BLTU('a5','a1',L['SHUT']-pc))
    a(None,  lambda pc,L: LUI('a1',DATA_HI))
    a(None,  lambda pc,L: LW('a1','a1',DATA_LO+12))               # OPEN_HI (0 = fail open)
    a(None,  lambda pc,L: BEQ('a1','zero',L['OPEN']-pc))
    a(None,  lambda pc,L: BGEU('a5','a1',L['SHUT']-pc))
    a('OPEN',lambda pc,L: LW('a1','sp',0))
    a(None,  lambda pc,L: ADDI('sp','sp',16))
    a(None,  lambda pc,L: LBU('a5','s1',TXBLOCK_OFF))             # host tx_block still honoured
    a(None,  lambda pc,L: J(RET_GATE-pc))
    a('SHUT',lambda pc,L: LUI('a1',TEL_HI))                       # refusal counter -> cb+0x1c
    a(None,  lambda pc,L: LW('a5','a1',TEL_LO))
    a(None,  lambda pc,L: ADDI('a5','a5',1))
    a(None,  lambda pc,L: SW('a1','a5',TEL_LO))
    a(None,  lambda pc,L: LW('a1','sp',0))
    a(None,  lambda pc,L: ADDI('sp','sp',16))
    a(None,  lambda pc,L: ADDI('a5','zero',1))                    # refuse: firmware defers
    a(None,  lambda pc,L: J(RET_GATE-pc))
    return P

# ---------------------------------------------------------------- stage 2: CONFIG CHANNEL
# Repoint the set-handler of MORSE_CMD_PARAM_ID_TX_BLOCK (=6, table 0x80200638 entry
# {id,set=0x00132eb4,get=0x00132ee4}).  The driver does NOT veto param 6 (command.c:1671
# switches on other ids only, so it falls to default: -> morse_cmd_tx()), which is why
# this channel is association-free: no AP, no AID, no beacon.
#   value & 0xF0000000 == 0            -> legacy tx_block (snez ; sb 0x802006FE)
#   idx = value >> 28 in 1..4          -> DATA[idx-1] = value & 0x0FFFFFFF
#     1 RHO   2 PERIOD   3 OPEN_LO   4 OPEN_HI
# 28-bit payload = 268 s of microseconds, far beyond any slot geometry.
# Host side: raise max_val in patches/morse_cli-tx_block-param.patch to 0xFFFFFFFF, else
# morse_cli rejects the word before it ever reaches the chip.
# WRITE ORDER: RHO, OPEN_LO, OPEN_HI, then PERIOD last (PERIOD is the enable).
# OFF SWITCH: `morse_cli tx_block 0x20000000` (PERIOD = 0).  One command, one word.
def prog_param():
    P=[]; a=lambda l,f: P.append((l,f))
    a('PAR', lambda pc,L: SRLI('a0','a5',28))
    a(None,  lambda pc,L: BEQ('a0','zero',L['LEG']-pc))
    a(None,  lambda pc,L: ADDI('a0','a0',-1))
    a(None,  lambda pc,L: ADDI('a1','zero',4))
    a(None,  lambda pc,L: BGEU('a0','a1',L['DONE']-pc))    # idx > 4 -> ignore, bounds the store
    a(None,  lambda pc,L: SLLI('a5','a5',4))
    a(None,  lambda pc,L: SRLI('a5','a5',4))
    a(None,  lambda pc,L: SLLI('a0','a0',2))
    a(None,  lambda pc,L: LUI('a1',DATA_HI))
    a(None,  lambda pc,L: ADDI('a1','a1',DATA_LO))
    a(None,  lambda pc,L: ADD('a1','a1','a0'))
    a(None,  lambda pc,L: SW('a1','a5',0))
    a('DONE',lambda pc,L: J(PARAM_DONE-pc))
    a('LEG', lambda pc,L: SNEZ('a5','a5'))                 # the displaced instruction
    a(None,  lambda pc,L: J(PARAM_LEG-pc))
    return P

STAGES = {
    # name        site patches                       trampolines
    'observe':  ([(SITE_GATE, RUN1)],                 [(RUN1, prog_observe, RUN1_LEN)]),
    'config':   ([(SITE_PARAM, RUN2)],                [(RUN2, prog_param,  RUN2_LEN)]),
    'gate':     ([(SITE_GATE, RUN1), (SITE_PARAM, RUN2)],
                 [(RUN1, prog_gate, RUN1_LEN), (RUN2, prog_param, RUN2_LEN)]),
}

def build(src, stage, seed_imem):
    img = bytearray(src)
    if len(img) != STOCK_LEN or zlib.crc32(bytes(img)) != STOCK_CRC:
        sys.exit("refusing: input is not the stock image (len %d crc32 0x%08x); every "
                 "offset here is build-specific -- re-derive before patching another build"
                 % (len(img), zlib.crc32(bytes(img))))
    sites, tramps = STAGES[stage]
    touched = []
    for base, prog, limit in tramps:
        blob = link(base, prog())
        if len(blob) > limit:
            sys.exit("trampoline %08x is %d B, run is %d B" % (base, len(blob), limit))
        off = coff(base)
        assert img[off:off+len(blob)] in (b'\x13\x00\x00\x00'*(len(blob)//4), bytes(len(blob))), \
               "trampoline landing zone at %08x is not the expected nop/zero run" % base
        img[off:off+len(blob)] = blob
        touched.append((base, len(blob), off))
    if seed_imem is not None:
        off = coff(DATA)
        img[off:off+16] = struct.pack('<4I', 0, 0, 0, 0)
        if stage == 'observe':
            img[off:off+4] = struct.pack('<I', seed_imem)
        touched.append((DATA, 16, off))
    for site, tgt in sites:
        off = coff(site)
        if bytes(img[off:off+4]) != ORIG[site]:
            sys.exit("site %08x does not hold the expected original bytes %s"
                     % (site, ORIG[site].hex()))
        img[off:off+4] = struct.pack('<I', J(tgt-site))
        touched.append((site, 4, off))
    return bytes(img), touched

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('stock'); ap.add_argument('out')
    ap.add_argument('--stage', choices=list(STAGES), required=True)
    ap.add_argument('--no-seed', action='store_true')
    a = ap.parse_args()
    img, touched = build(open(a.stock,'rb').read(), a.stage,
                         None if a.no_seed else IMEM_SEED)
    open(a.out,'wb').write(img)
    print("stage %s -> %s" % (a.stage, a.out))
    print("crc32 0x%08x  (must match the driver's `Loaded firmware ... crc32` line)"
          % zlib.crc32(img))
    for v,n,o in touched:
        print("  vaddr %08x  %3d B  container_off 0x%05X" % (v,n,o))
    print("revert: reload the stock image; the chip is RAM-loaded, unbind/bind undoes everything")

if __name__ == '__main__':
    main()
