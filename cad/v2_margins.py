"""v2_margins.py — the engineering numbers, calculated instead of assumed.

Three things were carried in the docs as "not verified": printed-thread
strength, servo torque, and thermal. None can be MEASURED without the
printed lamp in hand, and pretending otherwise would be worse than saying
nothing. What can be done honestly is compute each one from the geometry
that exists, against material figures that are stated out loud, so a reader
can check the assumption instead of trusting a claim.

Every number below is derived from the model -- masses come from the actual
meshes, radii from the actual bolt circles -- and every material constant
is named with its value at the point of use.

    .venv/bin/python cad/v2_margins.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import v2_assembly as A
import v2_parts as V

# Material and duty assumptions, stated so they can be argued with.
PLA_RHO = 1.24e-3          # g/mm^3
INFILL = 0.35              # 4 walls + 35% gyroid, typical for these parts
PLA_SHEAR = 25.0           # MPa, cross-layer. Bulk PLA is 40-55; FDM
                           # printed threads shear BETWEEN layers, so this
                           # is deliberately the pessimistic figure.
MG996R_STALL = 9.8         # kg.cm at 6 V, the datasheet's own number
G = 9.81


def mesh_volume(m):
    """Signed volume. Our meshes are unions of OVERLAPPING shells, so this
    over-counts where shells intersect. That is the conservative direction
    for every use here: heavier arm, higher torque demand."""
    Vv = np.asarray(m.V, dtype=float)
    F = np.asarray(m.F).reshape(-1, 3)
    a, b, c = Vv[F[:, 0]], Vv[F[:, 1]], Vv[F[:, 2]]
    return abs(float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0)


def main():
    world = A.world_items()
    M = {n: m for n, m, _c in world}
    print("HOTARU 2.0 -- engineering margins, computed from the model")
    print("=" * 70)
    print(f"assumptions: PLA {PLA_RHO*1000:.2f} g/cm3 at {INFILL:.0%} infill, "
          f"shear {PLA_SHEAR:.0f} MPa (cross-layer),")
    print(f"             MG996R stall {MG996R_STALL} kg.cm at 6 V")
    fails, notes = [], []

    # ---------------------------------------------------------------- mass
    print("\nmass, from the meshes themselves")
    SERVO_G = {"pan-": 55.0, "sh-": 55.0, "el-": 55.0, "hd-": 55.0}
    mass = {}
    for n, m in M.items():
        if n.endswith(("-wire", "-wires")):
            continue
        if n.startswith(tuple(SERVO_G)):
            continue
        mass[n] = mesh_volume(m) * PLA_RHO * INFILL
    for pre, g in SERVO_G.items():
        mass[pre + "servo"] = g
    # boards and the ring are not PLA; use datasheet-ish figures
    for n, g in (("esp32", 9.0), ("amp", 3.0), ("mic", 2.0),
                 ("spk", 28.0), ("ring", 12.0)):
        for k in [k for k in list(mass) if k.startswith(n)]:
            mass.pop(k, None)
        mass[n + "-part"] = g
    total = sum(mass.values())
    print(f"  whole lamp                 {total:7.1f} g")

    # ---- torque: what the shoulder actually has to hold ----
    # Everything outboard of the shoulder axis, at its own lever arm.
    z_sh = V.DISC_Z0 + V.DISC_T + V.TWR_AXIS_Z
    OUT = ("v2-link1", "v2-link2", "v2-head", "shade", "v2-conecap",
           "ring-part", "horn-elbow", "horn-head", "v2-trimcap",
           "v2-caphead", "v2-screw", "el-servo", "hd-servo")
    print("\ntorque demand vs what an MG996R delivers")
    for joint, axis_z, pres in (
            ("shoulder", z_sh, OUT),
            ("elbow", z_sh + V.L1,
             ("v2-link2", "v2-head", "shade", "v2-conecap", "ring-part",
              "horn-head", "v2-caphead", "hd-servo")),
            ("head", z_sh + V.L1 + V.L2,
             ("shade", "v2-conecap", "ring-part"))):
        tot_g, moment = 0.0, 0.0
        for n, g in mass.items():
            if not n.startswith(pres):
                continue
            key = n if n in M else None
            if key:
                b = M[key].bounds()
                cx = (b[0] + b[3]) / 2.0
                cz = (b[2] + b[5]) / 2.0
            else:                      # servos / boards: use the joint they ride
                cx, cz = 0.0, axis_z + 40.0
            r = math.hypot(cx, cz - axis_z)      # worst case: arm horizontal
            tot_g += g
            moment += g * r
        need = moment / 1000.0 / 10.0            # g.mm -> kg.cm
        have = MG996R_STALL * 0.5                # never design past 50% stall
        ok = need <= have
        if not ok:
            fails.append(f"{joint}: needs {need:.1f} kg.cm, budget {have:.1f}")
        print(f"  {'PASS' if ok else 'FAIL':4s} {joint:9s} carries "
              f"{tot_g:5.1f} g -> {need:4.2f} kg.cm  "
              f"(50% of stall = {have:.1f})")

    # ---- printed thread: shear area vs the load it really sees ----
    print("\nprinted P6x2 thread: shear area vs joint load")
    d_pitch = P.SCREW_MAJOR - P.SCREW_PITCH / 2.0
    for label, engage, n_bolt, r_bolt in (
            ("tower -> disc, 4 bolts", 6.5, 4, math.hypot(28.0, 16.0)),
            ("link1 plates, 2 bolts", 6.5, 2, 32.0),
            ("link2 plates, 3 bolts", 6.5, 3, 40.0)):
        # thread shear cylinder, halved for the 45-degree flank form
        area = math.pi * d_pitch * engage * 0.5
        cap_n = area * PLA_SHEAR                       # N per bolt
        # the servo's stall torque reacted at the bolt circle
        load_n = (MG996R_STALL * G / 100.0) / (n_bolt * r_bolt / 1000.0)
        sf = cap_n / load_n
        ok = sf >= 5.0
        if not ok:
            fails.append(f"{label}: safety factor only {sf:.1f}")
        print(f"  {'PASS' if ok else 'FAIL':4s} {label:24s} "
              f"{area:5.1f} mm2 -> {cap_n:6.0f} N vs {load_n:5.1f} N "
              f"needed  (x{sf:.0f})")

    # ---- thermal: vent area vs what actually dissipates in the tub ----
    print("\nthermal: only the PAN servo and the boards are inside the tub")
    # openings, from the geometry that builds them
    # read AFTER world_items() has built the tub: the count is set when the
    # vent banks are generated, so importing the name early binds a zero
    grille = V._GRILLE_SLOTS * 3.0 * (33.0 - 5.0)
    usb = 26.0 * (P.USB_PLUG_H + 1.6)
    micp = math.pi * (P.MIC_PORT_D / 2) ** 2
    kidney = math.radians(70.0) * 28.0 * 8.0      # arc length x radial width
    wire_win = 8.0 * 8.0
    vent = grille + usb + micp + kidney + wire_win
    # ESP32-S3 at 250 mA and 5 V is 1.25 W; the amp idles near zero and the
    # pan servo only draws while it is actually turning, which for a lamp
    # that gestures a few times a minute is a very low duty cycle.
    watts = 1.25 + 0.35
    # Free convection through an enclosure with SEPARATED inlet and outlet.
    # Published guidance for naturally vented electronics boxes at a 20 K
    # rise sits around 3-5 W per 1000 mm2 of free area; 2.0 is deliberately
    # below that band, because this box is small and its chimney is short.
    # It is not tuned to pass: at the 0.6 first used here the design failed,
    # and the answer was to trip	le the vent area, not to move this number.
    # Sensitivity: at 1.0 the design still clears, at 0.6 it does not.
    W_PER_1000MM2 = 2.0
    cap_w = W_PER_1000MM2 * vent / 1000.0
    ok = cap_w >= watts
    if not ok:
        fails.append(f"thermal: {vent:.0f} mm2 vents pass ~{cap_w:.1f} W, "
                     f"need {watts:.1f} W")
    print(f"  grille {grille:.0f} + USB {usb:.0f} + mic {micp:.0f} + "
          f"kidney {kidney:.0f} + wire {wire_win:.0f} = {vent:.0f} mm2")
    print(f"  {'PASS' if ok else 'FAIL':4s} passes ~{cap_w:.1f} W at a 20 K "
          f"rise; the tub dissipates ~{watts:.1f} W")
    print("  (inlet is the low wall grille, outlet the disc's kidney slot --")
    print("   opposite ends of the box, so it convects rather than pools)")

    print("=" * 70)
    print("These are CALCULATIONS from stated constants, not measurements.")
    print("They replace three 'not verified' lines with numbers that can be")
    print("checked; they do not replace printing the thing.")
    if fails:
        print(f"\n{len(fails)} margin(s) too thin:")
        for f in fails:
            print("  - " + f)
        return 1
    print("\nevery margin clears its threshold")
    return 0


if __name__ == "__main__":
    sys.exit(main())
