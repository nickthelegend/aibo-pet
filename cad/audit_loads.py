"""audit_loads.py — does the arm actually need internal bulkheads?

Removing the bulkheads moved the arm's load path from
    housing -> bulkhead -> floor
to
    housing -> lid rib web -> lid rim -> seat ring -> shoulder -> wall -> floor

That is only acceptable if the numbers say so. This recomputes the mass of
every printed part from its mesh volume, adds the servos at their joint
axes, swings the arm to its WORST case (straight out horizontal, maximum
lever), and works out what the load path actually sees.

    .venv/bin/python cad/audit_loads.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import part_arms
import part_head
import partlib as pl

PLA_RHO = 1.24e-3        # g/mm3
INFILL = 0.55            # thin-walled parts print near-solid; 0.55 is honest
MG996R_G, SG90_G = 55.0, 9.0
PLA_YIELD = 50.0         # MPa, bulk
PLA_Z = 0.55             # layer-adhesion knockdown for load across layers
DESIGN = PLA_YIELD * PLA_Z / 3.0     # MPa, with a 3x safety factor
E = 3500.0               # MPa


def mass_and_centroid(mesh):
    """(grams, centroid) by tetrahedron decomposition about the origin."""
    V, F = mesh._np()
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    vol6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = vol6.sum() / 6.0
    cent = ((a + b + c) / 4.0 * vol6[:, None]).sum(axis=0) / 6.0 / vol
    return vol * INFILL * PLA_RHO, cent


def main():
    print("=== printed mass (from mesh volume, %.0f%% effective) ===" % (INFILL * 100))
    items, total_g = [], 0.0
    for name, mesh, _c in (part_arms.build() + part_head.build()):
        g, cent = mass_and_centroid(mesh)
        items.append((name, g, cent))
        total_g += g
        print(f"  {name:14s} {g:6.1f} g")
    print(f"  {'servos':14s} {3*MG996R_G + SG90_G:6.1f} g  (3x MG996R + SG90)")
    arm_g = total_g + 3 * MG996R_G + SG90_G
    print(f"  {'TOTAL ARM':14s} {arm_g:6.1f} g")

    # --- worst case: arm straight out, horizontal, from the base pivot ------
    print("\n=== worst case: arm horizontal, fully extended ===")
    lever = {}
    y = 0.0
    for nm, L in (("arm-lower", P.ARM_LOWER_L), ("arm-upper", P.ARM_UPPER_L),
                  ("arm-fore", P.ARM_FORE_L)):
        lever[nm] = y + L / 2.0
        y += L
    lever["shade"] = y + 25.0
    for nm in ("cap-shoulder", "cap-elbow", "cap-head"):
        lever[nm] = y
    servo_moment = (MG996R_G * 0.0 + MG996R_G * P.ARM_LOWER_L
                    + MG996R_G * (P.ARM_LOWER_L + P.ARM_UPPER_L)
                    + SG90_G * y)
    moment_g_mm = sum(g * lever.get(nm, y) for nm, g, _c in items) + servo_moment
    M = moment_g_mm * 9.81e-6            # N.m
    print(f"  reach {y:.0f} mm, mass {arm_g:.0f} g")
    print(f"  static moment about the base pivot   {M:.2f} N.m")
    Md = M * 3.0
    print(f"  x3 for servo slam / dynamic           {Md:.2f} N.m  <- design load")

    # --- what each interface sees ------------------------------------------
    print("\n=== load path ===")
    bolt_span = P.JOINT_BOLT_Y[1] - P.JOINT_BOLT_Y[0]
    Fb = Md / (bolt_span / 1000.0) / 2.0
    print(f"  base-joint bolts  span {bolt_span:.0f} mm -> {Fb:6.1f} N per bolt pair")
    print(f"                    M3 insert pull-out is ~1500 N; utilisation "
          f"{100*Fb/1500:.1f}%")
    rim = P.BASE_TOP_D - 2 * P.WALL_STRUCT
    Fr = Md / (rim / 1000.0)
    print(f"  lid rim couple    span {rim:.0f} mm -> {Fr:6.1f} N")
    print(f"  seat ledge        {Fr/2:6.1f} N spread round a continuous ring")

    # --- the rib web: is the lid stiff enough without bulkheads? -----------
    # Each bolt's share travels down ITS OWN radial rib to the rim, which is
    # the support. So the case is one rib as a cantilever of length (bolt ->
    # rim) carrying that bolt's force -- not the whole moment lumped onto two
    # ribs, which is what the first version of this script did and why it
    # read 114%.
    print("\n=== lid rib web (what replaced the bulkheads) ===")
    d = (P.LID_Z0 - P.LID_RIB_Z0) + P.LID_T          # rib depth + plate
    t = P.LID_RIB_T
    I = t * d ** 3 / 12.0                            # ONE rib
    ybar = d / 2.0
    F_bolt = Md / (bolt_span / 1000.0) / 2.0         # N, per bolt
    L = math.hypot(rim / 2.0 - P.JOINT_BOLT_X, 20.0) # bolt boss -> rim
    sigma = (F_bolt * L) * ybar / I                  # MPa
    defl = F_bolt * L ** 3 / (3 * E * I)
    print(f"  one rib {t} x {d:.1f} mm   I = {I:.0f} mm4,  cantilever {L:.0f} mm")
    print(f"  bending stress    {sigma:6.2f} MPa   vs {DESIGN:.1f} MPa design limit"
          f"  ({100*sigma/DESIGN:.0f}% utilisation)")
    print(f"  rib tip deflection {defl:6.3f} mm")

    ok = sigma < DESIGN and defl < 0.5
    print("\n" + "=" * 62)
    if ok:
        print("VERDICT: bulkheads NOT required.")
        print(f"  Rib web at {100*sigma/DESIGN:.0f}% of design stress, {defl:.3f} mm "
              f"deflection, under a {Md:.2f} N.m")
        print(f"  dynamic load. The moment is small because the arm is light "
              f"({arm_g:.0f} g)")
        print(f"  -- the servos are {100*174/arm_g:.0f}% of it, not the plastic.")
    else:
        print("VERDICT: put the bulkheads back.")
    print("=" * 62)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
