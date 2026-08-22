#!/usr/bin/env python3
"""env_census.py — the NDN_* environment-surface gate (companion to crates/ndn-env, #81).

ndn-env exists because a misspelled or unregistered NDN_* variable fails silently: the
run looks configured and is not. The registry (crates/ndn-env/src/lib.rs) catches that
at *runtime* for names it knows about — this census catches the other half at *CI time*:
a newly introduced `env::var("NDN_…")` read that nobody registered would otherwise be
invisible to `ndn_env::snapshot()` forever.

What it does:
  1. scans the sibling repos for distinct NDN_* names passed to `env::var` / `env::var_os`
     in .rs files (`set_var`/`remove_var` do not count — those are writes, mostly tests);
  2. parses the registered names out of the ndn-env classification table;
  3. reads the committed baseline (crates/ndn-env/unregistered-baseline.txt) of known
     historical debt — names read today but not yet classified;
  4. exits nonzero listing any name that is neither registered nor baselined.

So the gate is a ratchet: existing debt is frozen in the baseline, and every NEW name
must be registered in ndn-env (the right fix) or deliberately baselined in the same
commit that introduces it (the visible-debt fix). Shrink the baseline over time;
never let it grow silently.

Usage:
  python3 tools/env_census.py [WORKSPACE_ROOT] [--write-baseline]

WORKSPACE_ROOT is the directory holding the repo checkouts as siblings
(ndn-rs/, ndn-ext/, …). Defaults to the parent of this repo, which is the local
workspace layout; CI passes ${{ github.workspace }}. Siblings that are not
checked out are skipped — CI checks out fewer repos than a local workspace has,
so a baseline entry whose only reader is an unchecked-out repo is NOT an error.

--write-baseline regenerates the baseline from the current diff (used to seed it,
and to prune it after registering names). Review the diff before committing.
"""

import re
import sys
from pathlib import Path

# The repos that read NDN_* variables. Scanned when present under the root, skipped
# (with a note) when not — CI only checks out the siblings its build needs.
SIBLINGS = ("ndn-rs", "ndn-ext", "ndn-fwd", "ndn-radio-drivers", "ndn-sim")

# Directory names never scanned: build output (a shared `target-shared/` sits at the
# workspace root; per-repo `target/` can appear anywhere) and VCS/hidden dirs.
SKIP_DIRS = {"target", "target-shared"}

# A read: `env::var("NDN_X")` / `env::var_os("NDN_X")` (also bare `var(` after a
# `use std::env::var`). The lookbehind rejects `set_var(` / `remove_var(` — writes,
# used by tests to plant values (e.g. ndn-env's own NDN_SCHED_CLAM typo test).
READ_RE = re.compile(r'(?<![A-Za-z0-9_])var(?:_os)?\s*\(\s*"(NDN_[A-Za-z0-9_]+)"')

# A registration: a `cfg("NDN_X", …)` / `dbg_("NDN_X", …)` entry in ndn-env's KNOWN
# table. Names in comments (e.g. the NDN_SCHED_GROUP_DEPTH deletion note) don't match.
REG_RE = re.compile(r'(?<![A-Za-z0-9_])(?:cfg|dbg_)\(\s*"(NDN_[A-Za-z0-9_]+)"')

REPO_ROOT = Path(__file__).resolve().parent.parent  # <ndn-radio-drivers>
REGISTRY = REPO_ROOT / "crates" / "ndn-env" / "src" / "lib.rs"
BASELINE = REPO_ROOT / "crates" / "ndn-env" / "unregistered-baseline.txt"

BASELINE_HEADER = """\
# NDN_* names read somewhere in the workspace (env::var / env::var_os in .rs files)
# but NOT registered in the ndn-env classification table (crates/ndn-env/src/lib.rs).
#
# This is FROZEN DEBT, enforced by tools/env_census.py in CI: a new NDN_* read must
# either be registered in ndn-env (preferred — it becomes visible to snapshot()/
# describe() run headers) or added here in the same commit, deliberately. Shrink this
# file by registering names and regenerating it (tools/env_census.py --write-baseline);
# never grow it silently.
"""


def scan_reads(root: Path):
    """name -> sorted list of files (relative to root) that read it."""
    reads: dict[str, set[str]] = {}
    scanned = []
    for sib in SIBLINGS:
        repo = root / sib
        if not repo.is_dir():
            continue
        scanned.append(sib)
        for path in repo.rglob("*.rs"):
            parts = path.relative_to(root).parts
            if any(p in SKIP_DIRS or p.startswith(".") for p in parts[:-1]):
                continue
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError as e:
                print(f"warning: unreadable {path}: {e}", file=sys.stderr)
                continue
            for name in READ_RE.findall(text):
                reads.setdefault(name, set()).add(str(path.relative_to(root)))
    return reads, scanned


def parse_registered() -> set[str]:
    names = set(REG_RE.findall(REGISTRY.read_text(encoding="utf-8")))
    if not names:
        sys.exit(f"error: parsed 0 registered names from {REGISTRY} — the table regex no "
                 "longer matches its source; fix env_census.py before trusting this gate")
    return names


def parse_baseline() -> set[str]:
    if not BASELINE.exists():
        return set()
    names = set()
    for line in BASELINE.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            names.add(line)
    return names


def main() -> int:
    args = [a for a in sys.argv[1:] if a != "--write-baseline"]
    write_baseline = "--write-baseline" in sys.argv[1:]
    root = Path(args[0]).resolve() if args else REPO_ROOT.parent
    if not root.is_dir():
        sys.exit(f"error: workspace root {root} is not a directory")

    reads, scanned = scan_reads(root)
    if not scanned:
        sys.exit(f"error: no sibling repos {SIBLINGS} found under {root}")
    registered = parse_registered()
    unregistered = {n: files for n, files in reads.items() if n not in registered}

    if write_baseline:
        body = "".join(f"{n}\n" for n in sorted(unregistered))
        BASELINE.write_text(BASELINE_HEADER + body, encoding="utf-8")
        print(f"wrote {BASELINE.relative_to(REPO_ROOT)}: {len(unregistered)} unregistered name(s)")
        return 0

    baseline = parse_baseline()
    new = {n: files for n, files in unregistered.items() if n not in baseline}

    print(f"env census: scanned {', '.join(scanned)} under {root}")
    print(f"  {len(reads)} distinct NDN_* names read; {len(registered)} registered in ndn-env; "
          f"{len(baseline)} baselined; {len(new)} unaccounted for")

    # Non-fatal hygiene notes. A baselined name that is no longer *seen* is not an error:
    # its reader may simply not be checked out here (CI scans fewer repos than a local
    # workspace). A baselined name that is now *registered* is pure staleness — prune it.
    stale = sorted(baseline & registered)
    if stale:
        print(f"  note: {len(stale)} baselined name(s) are now registered — prune them "
              f"(--write-baseline): {', '.join(stale)}")
    unseen = sorted(baseline - registered - set(reads))
    if unseen:
        print(f"  note: {len(unseen)} baselined name(s) not read in the scanned checkouts "
              f"(stale, or their reader is not checked out here): {', '.join(unseen)}")

    if new:
        print(f"\nFAIL: {len(new)} NDN_* name(s) read but neither registered in "
              f"crates/ndn-env/src/lib.rs nor baselined in "
              f"crates/ndn-env/unregistered-baseline.txt:", file=sys.stderr)
        for name in sorted(new):
            files = sorted(new[name])
            shown = ", ".join(files[:3]) + (f", … +{len(files) - 3} more" if len(files) > 3 else "")
            print(f"  {name}  ({shown})", file=sys.stderr)
        print("\nRegister the name (with an honest Config/DebugBisect classification) in "
              "ndn-env's KNOWN table, or baseline it deliberately.", file=sys.stderr)
        return 1
    print("  OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
