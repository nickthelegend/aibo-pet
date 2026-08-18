"""audit_printable.py — will each part physically print on a Bambu A1 mini?

Reports only. Nothing here changes geometry: if a part does not fit, that is
information, not something to silently "fix" by shrinking it.

A1 mini usable build volume is 180 x 180 x 180. Parts are checked as they sit
in their print orientation, and also rotated 45 degrees about Z, because a
part too wide for the axes can still fit on the diagonal (up to 254 across).
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly
import params as P
import partlib as pl

BED = P.PLATE
SKIRT = 3.0          # brim/skirt allowance per side


def main():
    print(f"Bambu A1 mini: {BED:.0f} x {BED:.0f} x {BED:.0f} mm "
          f"(allowing {SKIRT} mm per side for skirt)\n")
    usable = BED - 2 * SKIRT
    rows, bad = [], []
    for name, mesh in assembly.print_items():
        b = mesh.bounds()
        w, d, h = b[3] - b[0], b[4] - b[1], b[5] - b[2]
        diag = mesh.copy().rotate_z(45.0).bounds()
        dw, dd = diag[3] - diag[0], diag[4] - diag[1]
        flat = w <= usable and d <= usable
        rot = dw <= usable and dd <= usable
        ok = (flat or rot) and h <= BED
        how = "as-is" if flat else ("rotate 45" if rot else "-")
        rows.append((name, w, d, h, ok, how))
        if not ok:
            bad.append((name, w, d, h))
    print(f"{'part':16s} {'X':>7s} {'Y':>7s} {'Z':>7s}   {'fits':>5s}  how")
    print("-" * 58)
    for n, w, d, h, ok, how in rows:
        print(f"{n:16s} {w:7.1f} {d:7.1f} {h:7.1f}   "
              f"{'YES' if ok else 'NO':>5s}  {how}")
    print("-" * 58)
    tallest = max(rows, key=lambda r: r[3])
    widest = max(rows, key=lambda r: max(r[1], r[2]))
    print(f"tallest: {tallest[0]} at {tallest[3]:.0f} mm "
          f"({100*tallest[3]/BED:.0f}% of Z)")
    print(f"widest:  {widest[0]} at {max(widest[1], widest[2]):.0f} mm "
          f"({100*max(widest[1], widest[2])/usable:.0f}% of usable X/Y)")
    if bad:
        print("\nDOES NOT FIT:")
        for n, w, d, h in bad:
            print(f"  {n}  {w:.1f} x {d:.1f} x {h:.1f}")
    else:
        print("\nEvery part fits the A1 mini. Print them one at a time; the "
              "plates in\nexports/plate-*.stl are groupings, not a requirement.")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
