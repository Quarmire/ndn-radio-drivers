#!/usr/bin/env python3
"""Stage breakdown for one lease arm: host scheduling, host->chip transport, in-chip, and air.

Inputs
  ring.log  from /tmp/poll3.pl  (P host_ns0 host_ns1 ctrl mtime  +  2048 B ring hex)
  tx.csv    from halow_lease --csv          (seq,target_us,presend_us,postsend_us,ok,...)
  rx.csv    from halow_lease rx --csv       (seq,target_us,presend_us,rx_wall_us,tsft,...)

Two rules this obeys
  * ring entries are reassembled by the GLOBAL write counter, not by hoping the dump is in order,
    so ring loss is detected rather than silently interpolated;
  * the on-air instant is fitted against the RECEIVER's OWN wall clock (tsft ~ rx_wall_us), never
    against the sender's intended targets -- that fit's intercept would absorb the schedule and
    the run would confirm whatever it was handed.
"""
import sys, csv, math
from collections import Counter

def pct(v, q):
    if not v: return float('nan')
    s=sorted(v); k=(len(s)-1)*q; f=math.floor(k); c=math.ceil(k)
    return s[f] if f==c else s[f]+(s[c]-s[f])*(k-f)

def stats(v):
    if not v: return dict(n=0)
    m=sum(v)/len(v)
    sd=math.sqrt(sum((x-m)**2 for x in v)/len(v))
    return dict(n=len(v), mean=m, sd=sd, p50=pct(v,.5), p90=pct(v,.9),
                p99=pct(v,.99), p999=pct(v,.999), mx=max(v), mn=min(v))

def line(tag, s, unit="us"):
    if not s.get('n'): return f"  {tag:<28} n=0"
    return (f"  {tag:<28} n={s['n']:<6} sd={s['sd']:8.1f}  p50={s['p50']:9.1f} p90={s['p90']:9.1f} "
            f"p99={s['p99']:9.1f} p99.9={s['p999']:9.1f} max={s['mx']:10.1f}")

def read_ring(path):
    """-> (entries, loss_events).  entries = list of (global_index, mtime29, tag), in index order."""
    polls=[]
    with open(path) as f:
        while True:
            h=f.readline()
            if not h: break
            if not h.startswith('P '): continue
            _,h0,h1,ctrl,mt=h.split()
            hx=f.readline().strip()
            raw=bytes.fromhex(hx)
            words=[int.from_bytes(raw[i:i+4],'little') for i in range(0,len(raw),4)]
            polls.append((int(h0),int(h1),int(ctrl),int(mt),words))
    seen={}
    loss=[]
    prev_ctrl=None
    N=len(polls[0][4])
    # ☠ THE TEAR. ctrl is read BEFORE the 4x512 B ring dump, and the firmware keeps writing during
    # it, so the OLDEST indices we would otherwise claim, [ctrl-512, ctrl-512+written_during_dump),
    # have already been recycled and hold a word from exactly one ring wrap ago. Left uncorrected
    # that injected ~one-wrap (0.88 s) outliers into the in-chip stages -- a 875 ms "in-chip"
    # interval that is obviously the INSTRUMENT and not the radio. Drop a generous 64-entry
    # skirt and keep the FIRST sighting of each index; at a 0.2 s poll against a 0.88 s wrap every
    # index is still seen ~4 times, so nothing is lost.
    SKIRT=64
    for h0,h1,ctrl,mt,words in polls:
        if prev_ctrl is not None and ctrl-prev_ctrl > N-SKIRT:
            loss.append((prev_ctrl,ctrl))
        prev_ctrl=ctrl
        lo=max(0,ctrl-N+SKIRT)
        for k in range(lo,ctrl):
            if k not in seen:
                seen[k]=words[k % N]
    ent=[]
    for k in sorted(seen):
        w=seen[k]
        ent.append((k, w>>3, w&7))
    # second, independent guard: the shared clock is monotonic, so an entry whose mtime goes
    # backwards is a stale word however it got here. Report the count -- silence would hide it.
    clean=[]; back=0
    for e in ent:
        if clean and e[1] < clean[-1][1] and (clean[-1][1]-e[1]) < (1<<28):
            back+=1; continue
        clean.append(e)
    if back: print(f"  ☠ dropped {back} non-monotonic (stale) ring entries")
    return clean, loss, polls

def unwrap(vals, bits=29):
    """29-bit mtime -> monotonic."""
    out=[]; base=0; prev=None
    for v in vals:
        if prev is not None and v < prev - (1<<(bits-1)): base += 1<<bits
        out.append(v+base); prev=v
    return out

def quads(ent):
    """Consecutive tag runs 0,1,2,3 = one TX frame. Anything else (the RX-driven doorbell fires
    SUB/GO/IRQ without a CNT) breaks the pattern and is dropped rather than mis-paired."""
    out=[]; i=0; dropped=0
    tags=[e[2] for e in ent]
    while i+3 < len(ent):
        if tags[i:i+4]==[0,1,2,3]:
            out.append((ent[i][1],ent[i+1][1],ent[i+2][1],ent[i+3][1])); i+=4
        else:
            dropped+=1; i+=1
    return out, dropped

def fit(x,y):
    n=len(x); mx=sum(x)/n; my=sum(y)/n
    sxx=sum((a-mx)**2 for a in x); sxy=sum((a-mx)*(b-my) for a,b in zip(x,y))
    b=sxy/sxx; a=my-b*mx
    return a,b

def main(ring_path, tx_path, rx_path, label):
    ent, loss, polls = read_ring(ring_path)
    print(f"\n═══ {label} ═══")
    print(f"  ring: {len(ent)} entries, {len(polls)} polls, loss events={len(loss)}  "
          f"tags={dict(Counter(e[2] for e in ent))}")
    q,dropped = quads(ent)
    print(f"  quadruples (SUB,GO,IRQ,CNT) = {len(q)}   pattern-breaks dropped = {dropped}")

    tx=list(csv.DictReader(open(tx_path)))
    tx=[r for r in tx if r['ok']=='1']
    print(f"  tx.csv rows(ok) = {len(tx)}")

    # ---- stage 1: host scheduling, entirely on one host clock ----
    s1=[int(r['presend_us'])-int(r['target_us']) for r in tx]
    s_send=[int(r['postsend_us'])-int(r['presend_us']) for r in tx]

    # ---- pair chip quadruples to frames ----
    # ☠ NOT by tail-aligning the two lists. One dropped pattern-break in the middle shifts every
    # frame before it by one, and the symptom is a "SUB -> air" residual in the milliseconds while
    # every in-chip stage reads deterministic -- i.e. the pairing, not the radio. Match each frame
    # to its NEAREST chip submit under a 2-parameter clock map, monotonically, and refit.
    SUBall=unwrap([a for a,_,_,_ in q]); GOall=unwrap([b for _,b,_,_ in q])
    IRQall=unwrap([c for _,_,c,_ in q]); CNTall=unwrap([d for _,_,_,d in q])
    presall=[int(r['presend_us']) for r in tx]
    def align(a,b,win):
        pairs=[]; j=0
        for i,pv in enumerate(presall):
            if j >= len(SUBall): break
            pred=a+b*pv
            while j+1 < len(SUBall) and abs(SUBall[j+1]-pred) < abs(SUBall[j]-pred): j+=1
            if abs(SUBall[j]-pred) <= win:
                pairs.append((i,j)); j+=1
        return pairs
    n0=min(len(SUBall),len(presall))
    b0=(SUBall[-1]-SUBall[0])/(presall[-1]-presall[0])
    a0=SUBall[0]-b0*presall[0]
    pairs=align(a0,b0,20000)
    if len(pairs) > 10:
        a0,b0 = fit([presall[i] for i,_ in pairs],[SUBall[j] for _,j in pairs])
        pairs=align(a0,b0,8000)
    print(f"  aligned {len(pairs)}/{len(presall)} frames to chip submits "
          f"({len(SUBall)} quadruples available)")
    txs=[tx[i] for i,_ in pairs]
    SUB=[SUBall[j] for _,j in pairs]; GO=[GOall[j] for _,j in pairs]
    IRQ=[IRQall[j] for _,j in pairs]; CNT=[CNTall[j] for _,j in pairs]
    s1=[int(r['presend_us'])-int(r['target_us']) for r in txs]
    s_send=[int(r['postsend_us'])-int(r['presend_us']) for r in txs]

    # in-chip stages are one clock, exact
    s3=[g-s for s,g in zip(SUB,GO)]
    s4=[i-g for g,i in zip(GO,IRQ)]
    s5=[c-i for i,c in zip(IRQ,CNT)]

    # ---- stage 2: presend -> SUB, cross clock.  Fit chip mtime on the MEASURED presend
    # (not on the intended target: a fit against an intention is what makes a schedule confirm
    # itself). A 2-parameter affine cannot absorb per-frame jitter.
    pres=[int(r['presend_us']) for r in txs]
    a,b = fit(pres, SUB)
    s2=[s-(a+b*p) for s,p in zip(SUB,pres)]
    s2=[x-pct(s2,.5) for x in s2]     # centre on p50; only the spread is meaningful cross-clock
    print(f"  chip mtime rate vs host CLOCK_REALTIME: {b:.9f} ticks/us")

    print("  ── stages ──")
    print(line("1 target -> presend", stats(s1)))
    print(line("2 presend -> SUB  (fitted)", stats(s2)))
    print(line("3 SUB -> GO", stats([float(x) for x in s3])))
    print(line("4 GO -> IRQ", stats([float(x) for x in s4])))
    print(line("5 IRQ -> CNT", stats([float(x) for x in s5])))
    print(line("  (sendto syscall)", stats([float(x) for x in s_send])))

    # ---- the air ----
    rx=list(csv.DictReader(open(rx_path)))
    rx=[r for r in rx if r['tsft']]
    if rx:
        run=Counter(r['run'] for r in rx).most_common(1)[0][0]
        rx=[r for r in rx if r['run']==run]
        w=[int(r['rx_wall_us']) for r in rx]; t=[int(r['tsft']) for r in rx]
        ga,gb=fit(w,t)                       # RECEIVER's own two clocks. No schedule input.
        W=[(x-ga)/gb for x in t]             # tsft expressed in the receiver's own wall us
        tgt=[int(r['target_us']) for r in rx]
        e=[wi-ti for wi,ti in zip(W,tgt)]
        med=pct(e,.5)
        e=[x-med for x in e]                 # one CONSTANT removed (the inter-host clock offset)
        print(f"  rx: {len(rx)} stamped frames, tsft rate {gb:.9f} ticks/rx-wall-us, "
              f"const offset removed = {med:.0f} us")
        print(line("★ ON AIR: target -> air", stats(e)))
        # air relative to the chip's own submit, frame by frame
        seq2air={int(r['seq']):wi for r,wi in zip(rx,W)}
        pairs=[]
        for r,su in zip(txs,SUB):
            s=int(r['seq'])
            if s in seq2air: pairs.append((seq2air[s], su, int(r['target_us']), int(r['presend_us'])))
        if pairs:
            A=[p[0] for p in pairs]; S=[float(p[1]) for p in pairs]
            fa,fb=fit(S,A); dr=[x-(fa+fb*y) for x,y in zip(A,S)]
            dr=[x-pct(dr,.5) for x in dr]
            print(line("★ SUB -> air (residual)", stats(dr)))
    print()

if __name__=="__main__":
    main(*sys.argv[1:])
