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

    # ---- link1: plates flat in XY -> stand up. local X (length) -> world Z,
    # local Z (thickness) -> world X. rotate_y(-90) does exactly that.
    x_in = V.TWR_W / 2.0 + V.GAP_FIT                # 18.85 inner face
    for n, m, c in V.link1():
        q = m.copy()
        if n.endswith("-spacers") or n.endswith("-ledges"):
            continue          # assembly hardware; lives in the print plates
        q.rotate_y(-90.0)
        if n.endswith("-in"):
            q.translate(dx=x_in + V.PLATE_T, dz=z_sh)
        else:
            q.translate(dx=-x_in, dz=z_sh)
        out.append((n, q, c))

    z_el = z_sh + V.L1

    # ---- link2: drive plate -X (recess on the elbow horn), truncated
    # idler plate +X clear of the servo tail's swept annulus
    for n, m, c in V.link2():
        q = m.copy()
        if n.endswith("-spacers"):
            continue
        q.rotate_y(-90.0)
        if n.endswith("-in"):
            q.translate(dx=-V.LINK2_HALF, dz=z_el)
        else:
            q.translate(dx=V.LINK2_OUT_HALF + V.PLATE_T, dz=z_el)
        out.append((n, q, c))

    z_hd = z_el + V.L2

    # ---- head block between link2's plates at the far end; the tail is
    # asymmetric with the plates, so it shifts by half the difference
    xc = (V.LINK2_OUT_HALF - V.LINK2_HALF) / 2.0
    for n, m, c in V.head_block():
        q = m.copy()
        # rotate_x(90): (x,y,z)->(x,-z,y). Nose (local +Y) turns to +Z, along
        # the link; the SG axis at local y=22 lands at world z = 22, so one
        # translate puts it on z_hd. Height (local z 0..30) becomes y -30..0,
        # recentred with dy=+15.
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
    return out


# ------------------------------------------------------------- printing ----
def print_items():
    """Everything in print pose, flat side down, bottoms on Z=0."""
    out = []
    for n, m, c in (V.tub() + V.disc() + V.tower() + V.link1() + V.link2()
                    + V.head_block() + V.clamp_bars()):
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
