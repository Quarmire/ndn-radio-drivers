#!/usr/bin/env python3
"""On-air placement error for one arm, from rx.csv alone — no ring, no chip instrument.

The point of this tool is that it does not touch the SPI bus. The full stage breakdown needs the
ring, the ring poll needs the bus, and the bus is the thing under measurement; so the number that
sets the guard has to be obtainable WITHOUT it. It is: the receiver's radiotap TSFT stamps every
frame, and the frame carries its own intended target.

The fit is the receiver's own two clocks (tsft ~ its CLOCK_REALTIME). The sender's schedule enters
only through ONE removed constant (the median), which cannot create or hide jitter.
"""
import sys, csv, math
from collections import Counter, defaultdict

def pct(v,q):
    s=sorted(v); k=(len(s)-1)*q; f=math.floor(k); c=math.ceil(k)
    return s[f] if f==c else s[f]+(s[c]-s[f])*(k-f)

def fit(x,y):
    n=len(x); mx=sum(x)/n; my=sum(y)/n
    sxx=sum((a-mx)**2 for a in x); sxy=sum((a-mx)*(b-my) for a,b in zip(x,y))
    b=sxy/sxx; return my-b*mx, b

def go(path,label,tx_path=None):
    rows=[r for r in csv.DictReader(open(path)) if r['tsft']]
    run=Counter(r['run'] for r in rows).most_common(1)[0][0]
    rows=[r for r in rows if r['run']==run]
    rows.sort(key=lambda r:int(r['seq']))
    w=[int(r['rx_wall_us']) for r in rows]; t=[int(r['tsft']) for r in rows]
    a,b=fit(w,t)
    W=[(x-a)/b for x in t]
    tgt=[int(r['target_us']) for r in rows]
    e=[x-y for x,y in zip(W,tgt)]
    med=pct(e,.5); e=[x-med for x in e]
    m=sum(e)/len(e); sd=math.sqrt(sum((x-m)**2 for x in e)/len(e))
    print(f"\n═══ {label} ═══")
    sent=None
    if tx_path:
        sent=sum(1 for r in csv.DictReader(open(tx_path)) if r['ok']=='1')
    print(f"  n={len(rows)} stamped" + (f" of {sent} sent  ({100*len(rows)/sent:.2f}% delivered)" if sent else "")
          + f"   tsft rate {b:.9f} ticks/rx-wall-us")
    print(f"  ★ target -> air   sd={sd:8.1f}  p50={pct(e,.5):8.1f} p90={pct(e,.9):8.1f} "
          f"p99={pct(e,.99):9.1f} p99.9={pct(e,.999):9.1f} max={max(e):9.1f}")
    for thr in (100,200,500,1000,2000):
        k=sum(1 for x in e if abs(x)>thr)
        print(f"     |error| > {thr:5d} us : {k:6d}  ({100*k/len(e):6.3f}%)")
    # per-second buckets of the tail: an aggregate hides whether the tail is spread or clustered
    t0=min(tgt)
    buck=defaultdict(int); tot=defaultdict(int)
    for ti,ei in zip(tgt,e):
        s=(ti-t0)//1_000_000
        tot[s]+=1
        if abs(ei)>200: buck[s]+=1
    ks=sorted(tot)
    print("  tail(>200us) per second:", " ".join(str(buck.get(s,0)) for s in ks))
    # spacing between tail events
    ev=[ti for ti,ei in zip(tgt,e) if abs(ei)>200]
    if len(ev)>3:
        gaps=[(ev[i+1]-ev[i])/1000.0 for i in range(len(ev)-1)]
        print(f"  tail events n={len(ev)}  gap ms: p10={pct(gaps,.1):.1f} p50={pct(gaps,.5):.1f} "
              f"p90={pct(gaps,.9):.1f}  mean={sum(gaps)/len(gaps):.1f}")
    return e

if __name__=="__main__":
    go(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv)>3 else None)
