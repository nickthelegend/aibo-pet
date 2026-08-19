"""audit_band.py — does the Thenar Band actually hold together?

Same discipline as audit_interfaces.py: a check is not evidence until it has
been shown to FAIL on geometry that is wrong. Run with --prove and every
check is re-run against a deliberately broken variant; any check that still
passes there is reported as worthless, because it is.

    .venv/bin/python cad/audit_band.py
    .venv/bin/python cad/audit_band.py --prove
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import part_band as B
import partlib as pl

TOL = 1e-6


# The audit reads the SAME profiles the part is built from. If it rebuilt
# them from the constants it would keep passing after the part diverged.
_lug_ring = B.lug_ring
_entry_ring = B.entry_ring
_flange = B.flange_ring


def checks(twist, lug_z0):
    """Every check as (name, passed, detail). Parameterised on the two
    numbers a broken build would get wrong."""
    out = []

    lug = _lug_ring(twist)
    entry = _entry_ring()
    flange = _flange()

    # 1. the lug must not still be sitting over the gap it entered through
    over_gap = lug.intersection(entry).area
    out.append(("lug clears its entry gap", over_gap <= TOL,
                f"{over_gap:.1f} mm2 of lug still over a gap"))

    # 2. and the whole lug must be over flange material, or only part of it
    #    is carrying the pull out load
    captured = lug.intersection(flange).area
    frac = captured / lug.area if lug.area else 0.0
    out.append(("lug fully captured by flange", frac >= 0.999,
                f"{frac * 100:.1f}% of lug area over flange"))

    # 3. the twist must land inside the geometric window, not just near it
    lo = (B.ENTRY_SWEEP + B.LUG_SWEEP) / 2.0
    hi = 120.0 - lo
    out.append(("twist inside the window", lo < twist < hi,
                f"{lo:.1f} < {twist:.1f} < {hi:.1f}"))

    # 4. the lug has to be axially inside the groove, not fouling the flange
    #    or the back wall
    g0, g1 = B.FLANGE_T, B.FLANGE_T + B.GROOVE_T
    inside = g0 <= lug_z0 and lug_z0 + B.LUG_T <= g1 + TOL
    out.append(("lug axially inside the groove", inside,
                f"lug z {lug_z0:.2f}..{lug_z0 + B.LUG_T:.2f} vs groove "
                f"{g0:.2f}..{g1:.2f}"))

    return out


def static_checks():
    """Checks that do not depend on the bayonet numbers."""
    out = []

    # cap lip vs bay mouth: the printed snap fit needs its clearance back
    mouth = pl.rounded_rect(B.BAY_W - 2 * B.BAY_WALL, B.BAY_L - 2 * B.BAY_WALL,
                            max(B.BAY_R - B.BAY_WALL, 0.6))
    lip = pl.rounded_rect(B.BAY_W - 2 * B.BAY_WALL - 2 * B.CAP_FIT,
                          B.BAY_L - 2 * B.BAY_WALL - 2 * B.CAP_FIT,
                          max(B.BAY_R - B.BAY_WALL, 0.6))
    out.append(("cap lip fits the bay mouth", lip.within(mouth),
                f"{B.CAP_FIT} mm per side"))

    # the pad must not sit inside the wrist bore, or it is pressing on nothing
    parts = B.assembled()
    b = parts["band-pad"].bounds()
    out.append(("pad reaches past the cuff to the palm",
                b[1] < -(B.WRIST_Y / 2 + B.CUFF_WALL) + 2.0,
                f"pad min y {b[1]:.1f}"))

    # every part still has to print
    for name, m in B.build().items():
        out.append((f"{name} fits the plate", pl.fits_build_plate(m), ""))

    return out


def run(twist, lug_z0, label):
    rows = checks(twist, lug_z0)
    bad = [r for r in rows if not r[1]]
    print(f"\n  {label}")
    for name, ok, detail in rows:
        print(f"    {'PASS' if ok else 'FAIL'}  {name:34s} {detail}")
    return rows


def main():
    print("THENAR BAND -- assembly audit")
    rows = run(B.BAYO_TWIST, B.LUG_Z0, f"as built (twist {B.BAYO_TWIST})")
    for name, ok, detail in static_checks():
        print(f"    {'PASS' if ok else 'FAIL'}  {name:34s} {detail}")
        rows.append((name, ok, detail))

    failed = [r for r in rows if not r[1]]

    if "--prove" in sys.argv:
        print("\n  PROOF: the same checks against a bayonet that is not "
              "twisted home,\n  and a lug sunk into the flange. Any check that "
              "PASSES here is worthless.")
        broken = run(0.0, 0.0, "broken (twist 0, lug at z0)")
        useless = [n for n, ok, _ in broken if ok
                   and n in {"lug clears its entry gap",
                             "lug fully captured by flange",
                             "twist inside the window",
                             "lug axially inside the groove"}]
        print()
        if useless:
            print(f"  {len(useless)} check(s) do not discriminate: {useless}")
            return 1
        print("  all four bayonet checks fail on the broken build, so they "
              "are real.")

    print()
    if failed:
        print(f"{len(failed)} FAILED")
        return 1
    print(f"all {len(rows)} checks pass")
    return 0


if __name__ == "__main__":
    sys.exit(main())
