#!/usr/bin/env python3
"""Do two nodes running name-derived airtime leases actually transmit at different times?

    python3 lease_separation.py <settle_seconds> capA.pcap capB.pcap

Each capture is one node's own AR9271 in monitor mode, so capA holds B's frames and vice versa.
Every frame carries a TimeToken whose (ref_idx, ref_common) is the SENDER's hardware send timestamp
for an earlier frame, expressed in the sender's merged common time -- so once the clocks converge,
both nodes' transmit events are on one timeline and can be compared directly. No observer, no host
clock, and no third radio is involved.

⚠ Do NOT measure this as a histogram modulo the lease period. The slot rotates by one every epoch
(`slot = (base + epoch) mod N`), so modulo one period a perfectly working lease is uniform BY
CONSTRUCTION -- measured, and it cost a round of false "98% overlap" conclusions on firmware that was
gating correctly at 25% duty the whole time. Nearest-neighbour separation needs no modulus at all,
which is why it is what this script computes.

Pair it with a permutation null (slide one node's timeline by a random offset): a real separation
has to beat an arbitrary alignment, and that is the only thing that rules out reading structure into
two unrelated time bases.
"""
import struct,sys,bisect,statistics
def load(p):
    f=open(p,'rb'); f.read(24); out=[]
    while True:
        h=f.read(16)
        if len(h)<16: break
        ts,tus,incl,orig=struct.unpack('<IIII',h); d=f.read(incl)
        if len(d)<incl: break
        if len(d)<8: continue
        rlen=struct.unpack('<H',d[2:4])[0]
        b=d[rlen:] if rlen<=len(d) else b''
        if len(b)<40 or b[24:28]!=b'NDTT': continue
        my,ri,rc=struct.unpack('>III',b[28:40])
        out.append((b[10:16].hex(),ri,rc,ts+tus/1e6))
    return out
skip=float(sys.argv[1])
tx={}
for cap in sys.argv[2:]:
    rows=load(cap)
    if not rows: continue
    t0=rows[0][3]
    for a2,ri,rc,h in rows:
        if h-t0<skip or not ri: continue
        tx.setdefault(a2,{})[ri]=rc
ks=sorted(tx)
assert len(ks)==2, ks
# Both senders' events are in COMMON time, so they are directly comparable once the clocks merge.
# Unwrap around the 32-bit modulus by anchoring on the first event.
def series(d):
    v=sorted(d.values()); base=v[len(v)//2]
    out=[]
    for x in v:
        y=x-base
        if y> 1<<31: y-=1<<32
        if y<-(1<<31): y+=1<<32
        out.append(y)
    return sorted(out), base
A,baseA=series(tx[ks[0]]); B,baseB=series(tx[ks[1]])
shift=baseB-baseA
B=[b+shift for b in B]
print(f"{ks[0]}: {len(A)} sends   {ks[1]}: {len(B)} sends   (common time, settled window)")
gaps=[]
for a in A:
    i=bisect.bisect_left(B,a)
    cand=[abs(a-B[j]) for j in (i-1,i) if 0<=j<len(B)]
    if cand: gaps.append(min(cand))
gaps.sort()
n=len(gaps)
print(f"\nnearest opposite-node send, for each of {n} sends:")
for q in (0.01,0.1,0.25,0.5,0.75,0.9):
    print(f"   p{int(q*100):02d}  {gaps[int(q*n)]:>9,} us")
for thr in (100,500,1000,4096,8192):
    print(f"   within {thr:>5} us of an opposite send: {100*sum(1 for g in gaps if g<thr)/n:5.1f}%")
