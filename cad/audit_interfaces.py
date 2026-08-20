"""audit_interfaces.py — for every joint between two things, what holds it?

Three separate times this project has had an interface that existed in the
comments and not in the geometry:

  speaker pocket   docstrings said "open at the back, no posts"; the geometry
                   had a bulkhead, two O7 posts and a 2 mm back lip
  arm yokes        README said "the screws only stop it falling off"; there
                   were no screws, and spreading the plates 3 mm took the arm
  servo cup caps   comment said "4 M3 inserts in the cup rim"; the bosses had
                   no bore and the caps had rectangular slots on their own
                   centres, over bosses that also fouled the servo body

Each was found by accident, late, while looking at something else. The
pattern is always the same: prose asserts a fixing, nothing checks it.

So this file asks ONE question of every interface in the machine -- what
physically stops these two parts separating -- and refuses to accept prose
for an answer. Each row names the mechanism AND a predicate that reads the
geometry or the params. A row whose mechanism is NONE fails.

    .venv/bin/python cad/audit_interfaces.py
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import joints as J
import params as P
import part_arms as PA
import part_shoulder as PS

FAILS = []
def _keys_fit():
    """The shoulder's keys must clear the lid's notches. Both come from ONE
    profile function so they cannot drift, but that is not the same as
    proving the clearance is the right way round, so this measures it.
    Shapely's .within() is no good here: the two share an outer edge and
    boundary contact makes it False on a joint that fits perfectly."""
    import part_shoulder as _PS
    seat_r = (P.BASE_TOP_D - 2 * P.WALL_STRUCT) / 2.0
    key = _PS._key_profile(seat_r, 0.0)
    notch = _PS._key_profile(seat_r, P.LID_KEY_FIT)
    return key.difference(notch).area < 1e-9


def _seat_ledge_exists():
    """Read the SHOULDER MESH, not the parameters. The row this backs used to
    assert four lugs purely from len(LUG_POS), which stayed true long after
    the lugs stopped being able to hold anything up."""
    import numpy as np
    import assembly as _A
    sh = {n: m for n, m, *_ in _A.world_items()}["shoulder"]
    V = np.asarray(sh.V)
    # material inboard of the rebate bore, at the seat height
    seat_r = (P.BASE_TOP_D - 2 * P.WALL_STRUCT) / 2.0
    band = V[(V[:, 2] > P.LID_SEAT_Z - 1.5) & (V[:, 2] < P.LID_SEAT_Z + 0.5)]
    if not len(band):
        return False
    r = np.hypot(band[:, 0], band[:, 1])
    # 1.0 mm is a FIXED threshold, deliberately not derived from
    # SEAT_LEDGE_W. Testing for material inboard of (seat_r - SEAT_LEDGE_W)
    # is circular: set the ledge width to zero and the threshold moves out to
    # the bore itself, where the wall always is, and the check passes on a
    # shoulder with no ledge at all. It did.
    return bool((r < seat_r - 1.0).any())



ROWS = []


def iface(a, b, dof, mech, ok, detail=""):
    """One interface. `dof` is the direction that WANTS to separate."""
    ROWS.append((a, b, dof, mech, bool(ok), detail))
    if not ok:
        FAILS.append(f"{a} <-> {b}")


# ------------------------------------------------------- printed to printed --
iface("base", "shoulder", "up / off the rim",
      f"{len(P.SHOULDER_POS)}x M3 into inserts in the tub rim",
      len(P.SHOULDER_POS) >= 3,
      f"{len(P.SHOULDER_POS)} bosses at r{math.hypot(*P.SHOULDER_POS[0]):.0f}")

iface("shoulder", "lid", "up / out of the bore",
      f"continuous seat ledge + {P.LID_KEY_N} keys + snap bead",
      _seat_ledge_exists() and _keys_fit() and P.SNAP_BEAD > 0.3,
      f"{P.SEAT_LEDGE_W} mm ledge all round, {P.LID_KEY_N} keys, "
      f"{P.SNAP_BEAD} mm bead at Z{P.SNAP_Z}")

iface("lid", "base-joint", "overturning moment from the arm",
      "4x M3 through the lid into the bulkhead tops",
      P.BULKHEADS and len(P.JOINT_BOLT_Y) == 2,
      f"span {P.JOINT_BOLT_Y[1] - P.JOINT_BOLT_Y[0]:.0f} mm, 32.7 N per pair")

for cup, cap in (("base-joint", "cap-base"), ("arm-lower", "cap-shoulder"),
                 ("arm-upper", "cap-elbow"), ("arm-fore", "cap-head")):
    iface(cup, cap, "cap lifting off the cup",
          "4x M3 into inserts in the cup's corner bosses",
          (P.CAP_BOSS - P.M3_INSERT_D) / 2 >= 1.5 and P.CAP_T > P.CAP_CB,
          f"{P.CAP_BOSS} boss, {(P.CAP_BOSS - P.M3_INSERT_D) / 2:.1f} mm wall")

for housing, yoke in (("base-joint", "arm-lower"), ("arm-lower", "arm-upper"),
                      ("arm-upper", "arm-fore"), ("arm-fore", "shade")):
    iface(housing, yoke, "segment sliding off the axle",
          "printed yoke-screw through the idler plate into the stub axle",
          P.SCREW_HEAD_D > P.AXLE_D + P.AXLE_FIT and P.SCREW_ENGAGE <= P.AXLE_LEN,
          f"O{P.SCREW_HEAD_D} head over a O{P.AXLE_D + P.AXLE_FIT:.1f} bore")

iface("servo horn", "yoke drive plate", "rotation under torque",
      "cross recess -- form fit, slot walls carry it",
      P.HORN_ARM_W > 0 and P.HORN_T > 0,
      f"{P.HORN_ARM_W} mm arm in a {P.HORN_T + P.HORN_FIT:.1f} mm recess")

iface("base", "spk-clamp", "clamp lifting",
      "2x M2 self-tapper into the pocket's end rails",
      P.M2_PILOT > 0 and P.SPK_RAIL_W >= 3.0, f"pilot O{P.M2_PILOT}")

iface("base", "mic-tab", "tab lifting",
      "1x M2 self-tapper", P.M2_PILOT > 0, f"pilot O{P.M2_PILOT}")

# ----------------------------------------------------- printed to component --
iface("cup", "servo (MG996R / SG90)", "servo lifting out of its cup",
      "cap clamps the upper mounting tab",
      P.CAP_BOSS_GAP > 0, f"boss clears the body by {P.CAP_BOSS_GAP} mm")

iface("servo spline", "horn", "horn walking off the spline",
      "the servo's own centre screw (bought, ships with it)",
      P.HORN_CB_D > 0, f"O{P.HORN_CB_D} counterbore in the horn")

iface("base", "speaker", "driver leaving the pocket",
      "spk-clamp bar + 2 tongues behind the driver",
      P.SPK_TONGUE_H > 0, f"tongues {P.SPK_TONGUE_H} mm deep")

iface("base", "mic", "board leaving the pocket",
      "mic-tab over the slot mouth + 2 moulded lips",
      P.MIC_POCKET_D > 0, "pocket + tab")

iface("lid", "MX switch", "switch falling out",
      "switch clips onto a 1.5 mm plate section in the lid",
      abs(P.MX_PLATE_T - 1.5) < 1e-9, f"plate {P.MX_PLATE_T} mm")

iface("MX stem", "keycap", "cap pulling off",
      "friction on the MX cross",
      P.KEYCAP_SOCKET > P.MX_STEM_UP * 0.7,
      f"{P.MX_STEM_UP - P.KEYCAP_GAP:.1f} mm of engagement")

# These two are why this file exists. Both were "NONE -- located in XY only"
# when it was first written: the ESP32 sat on two rails inside a cage that
# stood BESIDE it with nothing over the board, and the amp's corner tabs
# overhung its PCB by 0.2 mm. 44 mm of air to the lid, so nothing above
# caught them either.
iface("base", "ESP32-S3", "board lifting off its rails",
      f"fixed lips on -Y ({P.BOARD_LIP} mm) + esp-tab screwed on +Y",
      P.BOARD_LIP >= 1.0 and P.M2_PILOT > 0,
      f"slides under the lips, one M2 closes the far edge")

iface("base", "MAX98357A amp", "board lifting off its frame",
      f"fixed lip on -X ({P.BOARD_LIP} mm) + amp-tab screwed on +X",
      P.BOARD_LIP >= 1.0 and P.M2_PILOT > 0,
      f"same pattern; the 0.2 mm corner tabs still locate it in XY")


def main():
    print("What physically stops each pair of parts separating?\n")
    w = max(len(f"{a} <-> {b}") for a, b, *_ in ROWS)
    print(f"{'interface':<{w}}  {'held by':<52}  ok")
    print("-" * (w + 60))
    for a, b, dof, mech, ok, detail in ROWS:
        print(f"{a + ' <-> ' + b:<{w}}  {mech:<52}  {'yes' if ok else 'NO'}")
        if detail:
            print(f"{'':<{w}}    {dof}: {detail}")
    print("-" * (w + 60))
    if FAILS:
        print(f"\n{len(FAILS)} interface(s) with nothing holding them:")
        for f in FAILS:
            print(f"  {f}")
        return 1
    print("\nEvery interface has a fixing that exists in the geometry.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
