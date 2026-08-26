"""v2_assembly.py — Hotaru 2.0 world pose, print pose, and plates.

The joint stack, +X is the drive side everywhere:

  pan       MG996R in the tub, spline up on (0,0), horn into the disc.
  shoulder  tower on the disc; horn outside the +X cheek at x 18.45..21.25;
            link1-in recess face at 18.85 -> the cross engages 2.4.
  elbow     MG996R inside link1's far end; spline through link1-in's boss
            bore; horn at 22.85..25.65; link2-in inner face at 26.85 -> the
            cross engages 1.9 of its 3.1 recess. Thinner than v1's full
            depth; carried as a measured number in the audit, not a hope.
  idlers    every -X side rides a printed threaded stub + yoke-screw, v1's
            proven pair, unchanged.
  head      SG90 in the head block between link2's plates; the v1 cone shade
            drops onto the same interface it always had.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import partlib as pl
import v2_parts as V

SH_AX_Z = V.DISC_Z0 + V.DISC_T + V.TWR_AXIS_Z      # 95.5? computed below
EL_AX_Z = None                                      # filled in world_items


def world_items():
    out = []
    for n, m, c in V.tub() + V.disc() + V.tower():
        out.append((n, m.copy(), c))

    z_sh = V.DISC_Z0 + V.DISC_T + V.TWR_AXIS_Z
    z_el = z_sh + V.L1
    z_hd = z_el + V.L2

    # ---- link1. Plates flat in XY; rotate_y(-90) maps length->+Z and
    # thickness->-X, so a plate's local z0 face lands at the translate dx.
    # Where the z0 face (the one carrying the recess or the boss web) must
    # face -X we add rotate_z(180), which mirrors x and y.
    for n, m, c in V.link1():
        q = m.copy()
        q.rotate_y(-90.0)
        if n == "v2-link1-in":
            q.rotate_z(180.0)
            q.translate(dx=V.L1_IN_HALF, dz=z_sh)      # plate 25.75..29.75
        elif n == "v2-link1-out":
            q.translate(dx=-V.L1_OUT_HALF, dz=z_sh)    # plate -30.15..-26.15
        elif n == "v2-link1-spacers":
            q.translate(dx=V.L1_IN_HALF - 0.2, dz=z_sh)
        elif n == "v2-link1-ledges":
            q.translate(dx=V.L1_IN_HALF - 0.05, dz=z_sh)
        out.append((n, q, c))

    # ---- link2: drive -X (recess toward +X onto the elbow horn), idler +X
    for n, m, c in V.link2():
        q = m.copy()
        q.rotate_y(-90.0)
        if n == "v2-link2-in":
            q.translate(dx=-V.L2_IN_HALF, dz=z_el)     # plate -34.45..-30.45
        elif n == "v2-link2-out":
            q.rotate_z(180.0)
            q.translate(dx=V.L2_OUT_HALF + V.PLATE_T, dz=z_el)  # 30.05..34.05
        elif n == "v2-link2-spacers":
            q.translate(dx=V.L2_OUT_HALF - 0.2, dz=z_el)
        out.append((n, q, c))

    # ---- head block between link2's plates at the far end; the tail is
    # asymmetric with the plates, so it shifts by half the difference
    xc = (V.L2_OUT_HALF - V.L2_IN_HALF) / 2.0
    for n, m, c in V.head_block():
        q = m.copy()
        q.rotate_x(90.0)
        q.translate(dx=xc, dy=15.0, dz=z_hd - 22.0)
        out.append((n, q, c))

    # ---- the v1 cone shade, unchanged, on the head axis
    import part_head
    sname, smesh, scol = part_head.build()[0]
    sm = smesh.copy()
    sm.translate(dz=-part_head.TILT)
    sm.rotate_x(180.0)
    sm.translate(dx=xc, dz=z_hd)
    out.append((sname, sm, scol))

    # ---- the hardware that visually and physically closes the joints ----
    scr = V.v2_screw()[0][1].copy()
    scr.rotate_y(90.0)
    scr.translate(dx=-(V.L1_OUT_HALF + V.PLATE_T + 3.2), dz=z_sh)
    out.append(("v2-screw", scr, V.COLORS["v2-accent"]))

    cap = V.v2_trimcap()[0][1].copy()
    cap.rotate_y(90.0)
    cap.translate(dx=-(V.L2_IN_HALF + V.PLATE_T + 3.2), dz=z_el)
    out.append(("v2-trimcap", cap, V.COLORS["v2-accent"]))

    # ---- the servos themselves: the joints read as connected because the
    # thing that connects them is finally in the picture ----
    import components as CO
    import numpy as np
    def servo(kind):
        parts = CO.mg996r() if kind == "mg" else CO.sg90()
        ms = [(n2, mm.copy(), c2) for n2, mm, c2 in parts]
        return ms
    def spline_xy(ms):
        V2 = np.vstack([np.asarray(mm.V) for _n, mm, _c in ms])
        top = V2[V2[:, 2] > V2[:, 2].max() - 2.0]
        return float(top[:, 0].mean()), float(top[:, 1].mean()), float(V2[:, 2].max())
    # pan: spline up on (0,0), body on the tub floor
    ms = servo("mg"); sx, sy, _t = spline_xy(ms)
    for n2, mm, c2 in ms:
        mm.translate(dx=-sx, dy=-sy, dz=V.FLOOR)
        out.append((f"pan-{n2}", mm, c2))
    # shoulder: spline +X, tip at +26.15 (case top 21.45 + 4.7)
    ms = servo("mg"); sx, sy, t = spline_xy(ms)
    for n2, mm, c2 in ms:
        mm.translate(dx=-sx, dy=-sy, dz=-t)
        mm.rotate_y(90.0)
        mm.translate(dx=V.CASE_TOP + 4.7, dz=z_sh)
        out.append((f"sh-{n2}", mm, c2))
    # elbow: spline -X, tip at -30.85 (case top -26.15 - 4.7)
    ms = servo("mg"); sx, sy, t = spline_xy(ms)
    for n2, mm, c2 in ms:
        mm.translate(dx=-sx, dy=-sy, dz=-t)
        mm.rotate_z(180.0)
        mm.rotate_y(-90.0)
        mm.translate(dx=-(V.L1_OUT_HALF + 4.7), dz=z_el)
        out.append((f"el-{n2}", mm, c2))
    # head: SG90 height along X, spline tip flush with the nose face
    ms = servo("sg"); sx, sy, t = spline_xy(ms)
    for n2, mm, c2 in ms:
        mm.translate(dx=-sx, dy=-sy, dz=-t)
        mm.rotate_y(90.0)
        mm.translate(dx=xc + V.HEAD_HALF, dz=z_hd)
        out.append((f"hd-{n2}", mm, c2))
    return out


# ------------------------------------------------------------- printing ----
def print_items():
    """Everything in print pose, flat side down, bottoms on Z=0."""
    out = []
    for n, m, c in (V.tub() + V.disc() + V.tower() + V.link1() + V.link2()
                    + V.head_block() + V.clamp_bars()
                    + V.v2_screw() + V.v2_trimcap()):
        q = m.copy()
        if n == "v2-disc":
            q.rotate_x(180.0)          # skirt up -> face down, prints flat
        if n.endswith("-in"):
            q.rotate_x(180.0)          # horn recess prints UP, not over air
        b = q.bounds()
        q.translate(dz=-b[2])
        out.append((n, q))
    # the shade prints exactly as v1 ships it
    import part_head
    sn, sm, _sc = part_head.build()[0]
    q = sm.copy(); b = q.bounds(); q.translate(dz=-b[2])
    out.append(("shade", q))
    return out


PLATES = [
    ("v2-plate-1-core", "tub + disc + tower: the whole base in one go",
     ["v2-tub", "v2-disc", "v2-tower"]),
    ("v2-plate-2-arm", "every link plate, spacers, head block, clamps",
     ["v2-link1-in", "v2-link1-out", "v2-link1-spacers",
      "v2-link2-in", "v2-link2-out", "v2-link2-spacers",
      "v2-head", "v2-clamps"]),
    ("v2-plate-3-shade", "the cone, unchanged from v1",
     ["shade"]),
]
