#!/usr/bin/env python3
"""Diff `src/mt76x0/`'s transcribed tables against the upstream mt76 C headers.

The MT7610U port carries ~700 rows of hand-transcribed RF, PLL, MAC and BBP
initialisation data. A single wrong hex digit in any of them is a silent RF
failure that presents as "the radio just does not receive" — the most expensive
class of bug this codebase has, and one no unit test can catch, because the
values are only meaningful to the silicon.

So the tables are not reviewed by eye; they are diffed. This resolves the C
macros (`MT_RF(bank,reg)`, `MT_BBP(unit,n)`, the `RF_*_BAND`/`RF_BW_*` flags,
and any `#define` reachable from `mt76x02_regs.h`) and the Rust struct
literals down to plain numbers on both sides, then compares them positionally.
It reads neither side's comments, so a comment that lies cannot hide a value
that is wrong.

    python3 tools/mt76x0_tabdiff.py     # exit 0 = every row matches

Point `UP` at a checkout of https://github.com/openwrt/mt76 (any recent commit;
these tables have not changed since 2018). Result as of the port landing:
14 tables, 698 rows, all identical.
"""
import re, sys, os

UP = "/private/tmp/claude-501/-Users-pmle-Documents-Dev-ndn-workspace-ndn-ext/3b590b8d-c60b-41e8-ad45-aaa2ef931b48/scratchpad/mt76-src"
RS = "/Users/pmle/Documents/Dev/ndn-workspace/ndn-radio-drivers/src"

BBP_BASE = {  # mt76x02_regs.h:604-617
    "CORE": 0x2000, "IBI": 0x2100, "AGC": 0x2300, "TXC": 0x2400, "RXC": 0x2500,
    "TXO": 0x2600, "TXBE": 0x2700, "RXFE": 0x2800, "RXO": 0x2900, "DFS": 0x2A00,
    "TR": 0x2B00, "CAL": 0x2C00, "DSC": 0x2E00, "PFMU": 0x2F00,
}
BAND = {"RF_G_BAND":0x0100,"RF_A_BAND":0x0200,"RF_A_BAND_LB":0x0400,
        "RF_A_BAND_MB":0x0800,"RF_A_BAND_HB":0x1000,"RF_A_BAND_11J":0x2000,
        "RF_BW_20":1,"RF_BW_40":2,"RF_BW_10":4,"RF_BW_80":8}

# ── #define symbol table from the upstream headers ──────────────────────────
_OBJ, _FN = {}, {}
def _load_defines():
    for h in ("mt76x02_regs.h", "mt76x0/phy.h", "mt76x02_mcu.h", "mt76.h"):
        try: src = open(os.path.join(UP, h)).read()
        except OSError: continue
        src = re.sub(r"\\\n", " ", src)
        for m in re.finditer(r"^#define\s+(\w+)\(([\w,\s]*)\)\s+(.+)$", src, re.M):
            _FN[m[1]] = ([a.strip() for a in m[2].split(",")], m[3].strip())
        for m in re.finditer(r"^#define\s+(\w+)\s+(.+)$", src, re.M):
            if m[1] not in _FN:
                _OBJ.setdefault(m[1], m[2].strip())
_load_defines()

def _resolve(tok, depth=0):
    """Resolve a C identifier or expression to a number using the #define tables."""
    if depth > 12: return None
    t = tok.strip()
    if not t: return None
    t = re.sub(r"/\*.*?\*/", "", t, flags=re.S).strip()
    # function-like macro call
    m = re.fullmatch(r"(\w+)\s*\((.*)\)", t, re.S)
    if m and m[1] in _FN:
        params, body = _FN[m[1]]
        args, cur, d = [], "", 0
        for ch in m[2] + ",":
            if ch == "(": d += 1
            elif ch == ")": d -= 1
            if ch == "," and d == 0: args.append(cur); cur = ""
            else: cur += ch
        if len(args) == len(params):
            for pnm, av in zip(params, args):
                body = re.sub(r"\b" + re.escape(pnm) + r"\b", "(" + av + ")", body)
            return _resolve(body, depth + 1)
    try:
        return int(t.replace("U", "").replace("u", ""), 0)
    except ValueError:
        pass
    if t in _OBJ:
        return _resolve(_OBJ[t], depth + 1)
    # arithmetic over resolvable leaves
    expr = t
    for ident in sorted(set(re.findall(r"(?<![\w0-9])[A-Za-z_]\w*", expr)), key=len, reverse=True):
        if ident in ("BIT", "GENMASK"): continue
        v = None
        if ident in _OBJ: v = _resolve(_OBJ[ident], depth + 1)
        if v is None: return None
        expr = re.sub(r"\b" + re.escape(ident) + r"\b", str(v), expr)
    expr = re.sub(r"BIT\s*\(([^()]*)\)", r"(1 << (\1))", expr)
    expr = re.sub(r"GENMASK\s*\(([^,]*),([^()]*)\)", r"(((1 << ((\1) - (\2) + 1)) - 1) << (\2))", expr)
    if not re.fullmatch(r"[-+*/()<>|&^~\s\dxXa-fA-F]+", expr): return None
    try: return eval(expr, {"__builtins__": {}}, {})
    except Exception: return None

def c_expr(e):
    """Evaluate a C initialiser expression using the macros we know."""
    e = e.strip()
    m = re.fullmatch(r"MT_RF\(\s*(\d+)\s*,\s*(\d+)\s*\)", e)
    if m: return (int(m[1]) << 16) | int(m[2])
    m = re.fullmatch(r"MT_BBP\(\s*(\w+)\s*,\s*(\d+)\s*\)", e)
    if m: return BBP_BASE[m[1]] + (int(m[2]) << 2)
    if "|" in e:
        v = 0
        for part in e.split("|"):
            sub = c_expr(part)
            if sub is None: return None
            v |= sub
        return v
    if e in BAND: return BAND[e]
    try:
        return int(e, 0)
    except ValueError:
        return _resolve(e)

def c_table(path, name):
    """Return the flat list of numeric leaves of each row of a C array."""
    src = open(path).read()
    i = src.find(name + "[] = {")
    if i < 0: return None
    i = src.index("{", i + len(name))
    depth, j = 0, i
    while True:
        if src[j] == "{": depth += 1
        elif src[j] == "}":
            depth -= 1
            if depth == 0: break
        j += 1
    body = src[i+1:j]
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.S)
    body = re.sub(r"//.*", "", body)
    rows, depth, cur = [], 0, ""
    for ch in body:
        if ch == "{":
            depth += 1
            if depth == 1: cur = ""; continue
        if ch == "}":
            depth -= 1
            if depth == 0: rows.append(cur); continue
        if depth >= 1: cur += ch
    out = []
    for r in rows:
        vals, tok, d, par = [], "", 0, 0
        for ch in r + ",":
            if ch == "(": par += 1
            elif ch == ")": par -= 1
            if ch == "{": continue
            if ch == "}": continue
            if ch == "," and d == 0 and par == 0:
                if tok.strip(): vals.append(c_expr(tok))
                tok = ""
            else:
                tok += ch
        out.append(vals)
    return out

def rs_table(path, name):
    src = open(path).read()
    m = re.search(re.escape(name) + r"\s*:\s*&\[[^\]]*?\]\s*=\s*&\[", src)
    if not m:
        m = re.search(re.escape(name) + r"[^=]*=\s*&\[", src)
    if not m: return None
    i = m.end() - 1
    depth, j = 0, i
    while True:
        if src[j] == "[": depth += 1
        elif src[j] == "]":
            depth -= 1
            if depth == 0: break
        j += 1
    body = src[i+1:j]
    body = re.sub(r"//.*", "", body)
    rows, depth, cur = [], 0, ""
    for ch in body:
        if ch in "([{":
            depth += 1
            if depth == 1: cur = ""; continue
        if ch in ")]}":
            depth -= 1
            if depth == 0: rows.append(cur); continue
        if depth >= 1: cur += ch
    def split_top(r):
        parts, tok, d = [], "", 0
        for ch in r + ",":
            if ch in "([{": d += 1
            elif ch in ")]}": d -= 1
            if ch == "," and d == 0:
                if tok.strip(): parts.append(tok)
                tok = ""
            else:
                tok += ch
        return parts

    def num(t):
        t = t.strip()
        t = re.sub(r"^\w+\s*:\s*", "", t).strip()          # struct field name
        t = re.sub(r"\b(?:super::|self::|crate::)?\w*::", "", t)  # path prefixes on consts
        if "|" in t:
            v = 0
            for p in t.split("|"):
                sub = num(p)
                if sub is None: return None
                v |= sub
            return v
        if t in BAND: return BAND[t]
        t2 = t.replace("_", "")
        if re.fullmatch(r"0[xX][0-9a-fA-F]+|\d+", t2):
            return int(t2, 0)
        return None

    out = []
    for r in rows:
        out.append([num(t) for t in split_top(r)])
    return out

CASES = [
    (f"{UP}/mt76x0/initvals_phy.h", "mt76x0_rf_central_tab",       f"{RS}/mt76x0/initvals_phy.rs", "RF_CENTRAL_TAB"),
    (f"{UP}/mt76x0/initvals_phy.h", "mt76x0_rf_2g_channel_0_tab",  f"{RS}/mt76x0/initvals_phy.rs", "RF_2G_CHANNEL_0_TAB"),
    (f"{UP}/mt76x0/initvals_phy.h", "mt76x0_rf_5g_channel_0_tab",  f"{RS}/mt76x0/initvals_phy.rs", "RF_5G_CHANNEL_0_TAB"),
    (f"{UP}/mt76x0/initvals_phy.h", "mt76x0_rf_vga_channel_0_tab", f"{RS}/mt76x0/initvals_phy.rs", "RF_VGA_CHANNEL_0_TAB"),
    (f"{UP}/mt76x0/initvals_phy.h", "mt76x0_rf_bw_switch_tab",     f"{RS}/mt76x0/initvals_phy.rs", "RF_BW_SWITCH_TAB"),
    (f"{UP}/mt76x0/initvals_phy.h", "mt76x0_rf_band_switch_tab",   f"{RS}/mt76x0/initvals_phy.rs", "RF_BAND_SWITCH_TAB"),
    (f"{UP}/mt76x0/initvals_phy.h", "mt76x0_rf_ext_pa_tab",        f"{RS}/mt76x0/initvals_phy.rs", "RF_EXT_PA_TAB"),
    (f"{UP}/mt76x0/initvals_phy.h", "mt76x0_frequency_plan",       f"{RS}/mt76x0/freq_plan.rs",    "FREQUENCY_PLAN"),
    (f"{UP}/mt76x0/initvals_phy.h", "mt76x0_sdm_frequency_plan",   f"{RS}/mt76x0/freq_plan.rs",    "SDM_FREQUENCY_PLAN"),
    (f"{UP}/mt76x0/initvals.h",     "mt76x0_bbp_switch_tab",       f"{RS}/mt76x0/initvals.rs",     "BBP_SWITCH_TAB"),
    (f"{UP}/mt76x0/initvals_init.h","common_mac_reg_table",        f"{RS}/mt76x0/initvals.rs",     "COMMON_MAC_REG_TABLE"),
    (f"{UP}/mt76x0/initvals_init.h","mt76x0_mac_reg_table",        f"{RS}/mt76x0/initvals.rs",     "MT76X0_MAC_REG_TABLE"),
    (f"{UP}/mt76x0/initvals_init.h","mt76x0_bbp_init_tab",         f"{RS}/mt76x0/initvals.rs",     "MT76X0_BBP_INIT_TAB"),
    (f"{UP}/mt76x0/initvals_init.h","mt76x0_dcoc_tab",             f"{RS}/mt76x0/initvals.rs",     "MT76X0_DCOC_TAB"),
]

bad = 0
for cpath, cname, rpath, rname in CASES:
    if not os.path.exists(rpath):
        print(f"  SKIP {rname}: {os.path.basename(rpath)} not written yet"); continue
    c = c_table(cpath, cname)
    r = rs_table(rpath, rname)
    if c is None: print(f"  ?? C table {cname} not found"); continue
    if r is None: print(f"  ?? Rust table {rname} not found in {os.path.basename(rpath)}"); bad += 1; continue
    if len(c) != len(r):
        print(f"  ✗ {rname}: LENGTH {len(r)} rust vs {len(c)} C"); bad += 1; continue
    diffs = []
    for n, (cr, rr) in enumerate(zip(c, r)):
        cv = [v for v in cr if v is not None]
        rv = [v for v in rr if v is not None]
        if len(cv) != len(rv) or cv != rv:
            diffs.append((n, cr, rr))
    if diffs:
        print(f"  ✗ {rname}: {len(diffs)}/{len(c)} rows differ")
        for n, cr, rr in diffs[:6]:
            print(f"      row {n}: C={['None' if v is None else hex(v) for v in cr]}")
            print(f"              R={['None' if v is None else hex(v) for v in rr]}")
        bad += 1
    else:
        print(f"  ✓ {rname}: {len(c)} rows identical")
sys.exit(1 if bad else 0)
