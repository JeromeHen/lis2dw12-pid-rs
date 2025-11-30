#!/usr/bin/env python3
"""
Check that each lis2dw12_ C function in lis2dw12-pid/lis2dw12_reg.h has a matching
Rust counterpart in `src/` or `tests/`.

This check is fuzzy: for C names with digits (e.g. `4d`, `6d`) it maps them to
`fourd`/`sixd` when looking in Rust sources because the Rust port uses word
names for these symbols.

Exit code: 0 if all functions have at least one matching Rust occurrence, 1
otherwise and prints the missing functions.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
H = ROOT / 'lis2dw12-pid' / 'lis2dw12_reg.h'
SRCDIRS = [ROOT / 'src', ROOT / 'tests']

if not H.exists():
    print('ERROR: lis2dw12_reg.h not found at', H)
    sys.exit(1)

txt = H.read_text(encoding='utf8')

# match function prototypes returning int32_t and starting with lis2dw12_
fn_re = re.compile(r'\bint32_t\s+(lis2dw12_[a-z0-9_]+)\s*\(')
names = sorted(set(fn_re.findall(txt)))


def rust_candidates(cname):
    # strip prefix
    s = cname[len('lis2dw12_'):]
    cands = {s}
    # Replace '4d' -> 'fourd', '6d' -> 'sixd'
    s2 = s.replace('4d', 'fourd').replace('6d', 'sixd')
    cands.add(s2)
    # Some names may map to who_am_i vs device_id_get etc; keep both
    return cands


def search_sources(token):
    # Search for token in src/ and tests/ files
    for d in SRCDIRS:
        for path in d.rglob('*.rs'):
            if token in path.read_text(encoding='utf8'):
                return True
    return False


missing = []
for n in names:
    cands = rust_candidates(n)
    found = False
    for cand in cands:
        if search_sources(cand):
            found = True
            break
    if not found:
        missing.append(n)

if missing:
    print('\nParity check FAILED — the following C functions have no obvious Rust counterpart:')
    for m in missing:
        print(' -', m)
    print('\nNotes: the check performs a name substring match in `src/` and `tests/` and')
    print('applies a small digit-to-word mapping (4d->fourd, 6d->sixd). If a function is')
    print('implemented under a different name or intentionally absent, update this script')
    print('or provide an allowlist in the CI workflow.')
    sys.exit(1)

print('Parity check OK — all C functions in lis2dw12_reg.h have matching Rust tokens (fuzzy).')
sys.exit(0)
