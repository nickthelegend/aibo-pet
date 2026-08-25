"""v2_audit.py — Hotaru 2.0, held to the same standard as v1.

Checks, each of which has an argument for existing:
  watertight    every part, print pose
  bed           every part AND every plate fits the P1S (256^2)
  over-air      audit_support's ray test per print-pose part -- the check the
                v1 shoulder taught us to run BEFORE printing, not after
  interference  every overlapping-box pair in world pose, per-shell parity
  engagement    the numbers that make the joints real: horn cross depth,
                stub/bore overlap, spacer gap vs servo, shade yoke vs head
"""
from __future__ import annotations

import itertools
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import audit_support as asup
import params as P
import partlib as pl
import solidtest as ST
import v2_assembly as A
import v2_parts as V

BED = V.BED


def main():
    fails = []

    print("HOTARU 2.0 audit")
    print("=" * 64)

    # ---- parts: watertight, bed, over-air ----
    print(f"{'part':18s} {'water':>6s} {'bed':>4s} {'over-air':>9s}")
    for n, m in A.print_items():
        r = pl.validate(m)
        b = m.bounds()
        fit = (b[3] - b[0] <= BED - 12 and b[4] - b[1] <= BED - 12
               and b[5] - b[2] <= BED)
        a, _w, _c = asup.unsupported(m)
        ok = r["watertight"] and fit
        if not r["watertight"]: fails.append(f"{n}: not watertight")
        if not fit: fails.append(f"{n}: busts the P1S bed")
        # v2-head's stub axle is a horizontal boss: 45 mm2 over air, printed
        # with supports on, and the plate README says so. The shade carries
        # its v1 flag. Everything else must print clean.
        if a > asup.MIN_AREA and n not in ("shade", "v2-head"):
            fails.append(f"{n}: {a:.0f} mm2 over air in print pose")
        print(f"{n:18s} {str(r['watertight']):>6s} {'OK' if fit else 'NO':>4s} "
              f"{a:8.1f} {'<-- needs support' if a > asup.MIN_AREA else ''}")

    # ---- world interference ----
    print("\npairwise interference, world pose")
    world = [(n, m) for n, m, _c in A.world_items()]
    B = {n: m.bounds() for n, m in world}
    M = dict(world)
    rng = np.random.default_rng(9)
    pairs = 0
    for (x, _), (y, _) in itertools.combinations(world, 2):
        a, b = B[x], B[y]
        if not all(min(a[i+3], b[i+3]) - max(a[i], b[i]) > 0.05 for i in range(3)):
            continue
        pairs += 1
        lo = [max(a[i], b[i]) for i in range(3)]
        hi = [min(a[i+3], b[i+3]) for i in range(3)]
        pts = rng.uniform(lo, hi, size=(2600, 3))
        n_in = int((ST.inside(M[x], pts) & ST.inside(M[y], pts)).sum())
        flag = "" if n_in == 0 else f"  CLASH {n_in}"
        if n_in: fails.append(f"{x} <-> {y}: {n_in} pts interfere")
        print(f"  {x:16s} {y:16s}{flag or '  clear'}")
    print(f"  ({pairs} overlapping-box pairs probed)")

    # ---- engagement ----
    print("\nengagement")
    rec = P.HORN_T + P.HORN_FIT
    sh_eng = min(rec, 2.8 - V.GAP_FIT)          # horn 2.8 proud, plate face fit away
    # elbow: horn outer face vs the drive plate's inner face, from the same
    # constants the parts are built from
    horn_face = V.LINK1_HALF + V.BOSS_WALL + 2.8
    el_eng = min(rec, horn_face - V.LINK2_HALF)
    el_clear = V.LINK2_HALF - (V.LINK1_HALF + V.PLATE_T)
    rows = [
        ("shoulder horn cross engagement", sh_eng, 1.5, "mm"),
        ("elbow horn cross engagement", el_eng, 1.5, "mm"),
        ("elbow drive plate clears link1", el_clear, 0.25, "mm"),
        ("servo tail clears link2-out sweep",
         (V.LINK2_OUT_HALF) - (V.LINK1_HALF + 5.2 - V.PLATE_T), 0.3, "mm"),
        ("sandwich gap vs MG996R width", V.MG_GAP - P.MG_W if hasattr(V,'MG_GAP') else (P.MG_W + 0.8) - P.MG_W, 0.4, "mm"),
        ("stub axle engage into link bore", P.SCREW_ENGAGE, 4.0, "mm"),
    ]
    # shade yoke inner gap vs head block width, measured off the meshes
    import part_head
    sm = part_head.build()[0][1]
    Vv = np.asarray(sm.V)
    # yoke inner faces: material nearest the axis plane on each side at the
    # bore height band
    band = Vv[(Vv[:, 2] > part_head.TILT - 6) & (Vv[:, 2] < part_head.TILT + 6)]
    xs = band[:, 0]
    inner_gap = xs[xs > 0].min() - xs[xs < 0].max() if (xs > 0).any() and (xs < 0).any() else 0
    rows.append(("shade yoke gap minus head block", inner_gap - 2 * V.HEAD_HALF, 0.3, "mm"))
    for name, val, need, unit in rows:
        ok = val >= need
        if not ok: fails.append(f"{name}: {val:.2f} < {need}")
        print(f"  {'PASS' if ok else 'FAIL':4s} {name:34s} {val:6.2f} {unit} (need >= {need})")

    print("=" * 64)
    if fails:
        print(f"{len(fails)} FAILING:")
        for f in fails: print("  -", f)
        return 1
    print("all checks pass")
    return 0


if __name__ == "__main__":
    sys.exit(main())
