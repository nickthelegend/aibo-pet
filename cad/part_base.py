"""part_base.py — HOTARU base: a round turned "pebble", not a box.

A straight cylinder to BASE_STRAIGHT, then a shoulder that tapers in to meet
the lid, so the whole thing reads as one turned form. The taper runs inward
as it rises, which means every layer is smaller than the one below it -- the
silhouette is decorative AND support-free.

Interior (X right, Y back, Z up; origin = centre, Z0 = bottom):

  floor 0..2.4       four foot recesses
  wall  2.4..42      straight, 2.4 thick, carrying:
                       left   speaker grille (slots through a flat land)
                       right  USB-C window, sized to the plug's OVERMOLD
                              (not its shell) -- on a round wall the
                              receptacle ends up ~7.6 deep, so the moulding
                              has to be able to enter the window with it
                       front  3 mic ports
  rim   42           the tub STOPS here. The shoulder is a separate part, so
                     the mouth stays at its full O155.2 and every component
                     drops straight in -- with the shoulder moulded on, the
                     speaker could not physically be got into its pocket.
                     3 M3 insert bosses take the shoulder.
  seat lugs          4 short M3 hold-down lugs webbed into the seat ring
  bulkheads          X = +-35, Y 2..34, WINDOWED -- a 5 mm frame round a big
                     opening, so the harness crosses straight through. The
                     base joint bolts through the lid into inserts in their
                     tops, putting the arm's load on the floor directly; the
                     lid's rib web backs it up. BULKHEADS=False drops them.
  board bay          ESP32-S3 flat, pins DOWN into 13.6 mm
  speaker            fires -X. NO bulkhead and NO clamp posts -- the pocket
                     is open at the top AND across its whole back, so the
                     leads have somewhere to go and the tub itself (~749 cm3)
                     is the back volume. spk-clamp screws down into two bores
                     sunk in the pocket's own end rails.
  mic                U-slot counterbore in the FRONT WALL facing the ports,
                     with two full-height retaining lips
  amp / cap / zip-tie bars
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
from shapely import affinity
from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import joints as J
import params as P
import partlib as pl

OVL = pl.OVL
R = P.BASE_D / 2.0
OUTER = pl.circle(P.BASE_D, 128)
IN_THICK = pl.circle(P.BASE_D - 2 * P.WALL_STRUCT, 128)


def _shoulder_od(z):
    """Quarter-sine shoulder: tangent to the cylinder where it leaves it, so
    the two blend into one turned form instead of meeting at a hard step."""
    t = min(max((z - P.BASE_STRAIGHT) / (P.LID_SEAT_Z - P.BASE_STRAIGHT), 0.0), 1.0)
    return P.BASE_D - (P.BASE_D - P.BASE_TOP_D) * t


# ------------------------------------------------------------- openings ----
def _speaker_slots():
    pitch = P.SPK_SLOT_W + P.SPK_SLOT_GAP
    span = P.SPK_SLOT_N * P.SPK_SLOT_W + (P.SPK_SLOT_N - 1) * P.SPK_SLOT_GAP
    y0 = P.SPK_CTR_Y - span / 2.0
    return unary_union([box(-R - 6, y0 + i * pitch, P.SPK_CTR_X + 2.0,
                            y0 + i * pitch + P.SPK_SLOT_W)
                        for i in range(P.SPK_SLOT_N)])


def _mic_ports():
    w, pitch = 1.6, 3.5
    return unary_union([box(P.MIC_CTR_X + k * pitch - w / 2, -R - 6,
                            P.MIC_CTR_X + k * pitch + w / 2, -R + 8.0)
                        for k in (-1, 0, 1)])


BX, BY = P.ESP_CTR
BOARD_X1 = BX + P.ESP_L / 2
USB_W = P.ESP_USB_WIN_W
USB_CUT = box(BOARD_X1 - 2.0, P.USB_CTR_Y - USB_W / 2, R + 6, P.USB_CTR_Y + USB_W / 2)
OPENINGS = [
    (_speaker_slots(), 14.0, 30.0),
    (USB_CUT, *P.ESP_USB_WIN_Z),
    (_mic_ports(), P.MIC_CTR_Z - 0.8, P.MIC_CTR_Z + 0.8),
]


# ---------------------------------------------------------------- pieces ----
def _floor():
    feet = unary_union([affinity.translate(pl.circle(P.FOOT_D), 58 * math.cos(a),
                                           58 * math.sin(a))
                        for a in np.linspace(0, 2 * math.pi, 4, endpoint=False)
                        + math.radians(45)])
    m = pl.Mesh()
    m += pl.prism(OUTER.difference(feet), 0.0, 0.6)
    m += pl.prism(OUTER, 0.6, P.FLOOR)
    return m


def _walls():
    m = pl.banded(pl.ring2d(OUTER, IN_THICK), P.FLOOR - OVL, P.BASE_STRAIGHT, OPENINGS)
    return m


def _usb_boss():
    """Flat land bridging the curved wall to the flat board end, window cut
    through it -- otherwise a USB-C plug cannot reach the receptacle."""
    land = box(BOARD_X1 - 2.0, P.USB_CTR_Y - USB_W / 2 - 4.0, R,
               P.USB_CTR_Y + USB_W / 2 + 4.0).intersection(OUTER)
    return pl.banded(land, P.ESP_USB_WIN_Z[0] - 5.0, P.ESP_USB_WIN_Z[1] + 4.0,
                     [(USB_CUT, *P.ESP_USB_WIN_Z)])


def _bulkheads():
    """Windowed walls carrying the base joint straight down to the floor.

    Solid ones are what ruin cable management, so these are a frame: the
    opening is BULK_WIN_INSET in from every edge, leaving a 5 mm border and
    a clear path for the harness. Band-split in Z so the window's edges land
    exactly on spec.
    """
    m = pl.Mesh()
    for sx in (-1, 1):
        cx = sx * P.JOINT_BOLT_X
        prof = box(cx - P.BULK_T / 2, P.BULK_Y[0],
                   cx + P.BULK_T / 2, P.BULK_Y[1]).intersection(IN_THICK)
        win = box(cx - P.BULK_T, P.BULK_Y[0] + P.BULK_WIN_INSET,
                  cx + P.BULK_T, P.BULK_Y[1] - P.BULK_WIN_INSET)
        bores = unary_union([affinity.translate(pl.circle(P.M3_INSERT_D), cx, y)
                             for y in P.JOINT_BOLT_Y])
        m += pl.banded(prof, P.FLOOR - OVL, P.LID_SEAT_Z,
                       [(win, P.BULK_WIN_Z[0], P.BULK_WIN_Z[1])])
        m += pl.prism(prof.difference(bores), P.LID_SEAT_Z - OVL, P.BASE_H)
    return m


def _shoulder_bosses():
    """M3 inserts the shoulder ring bolts down into. Three, hand-placed into
    the only gaps left at the rim."""
    m = pl.Mesh()
    for cx, cy in P.SHOULDER_POS:
        a = math.atan2(cy, cx)
        prof = affinity.translate(pl.circle(P.M3_BOSS_D), cx, cy).union(
            pl.stroke([(cx, cy), (cx + 12 * math.cos(a), cy + 12 * math.sin(a))], 3.4))
        prof = prof.intersection(IN_THICK.buffer(OVL))
        bore = affinity.translate(pl.circle(P.M3_INSERT_D), cx, cy)
        m += pl.prism(prof, P.FLOOR - OVL, P.SHOULDER_BOSS_Z)
        m += pl.prism(prof.difference(bore), P.SHOULDER_BOSS_Z - OVL,
                      P.BASE_STRAIGHT)
    return m


def _board_bay():
    x0, x1 = BX - P.ESP_L / 2, BX + P.ESP_L / 2
    y0, y1 = BY - P.ESP_W / 2, BY + P.ESP_W / 2
    top = P.ESP_Z
    cage_z = top + P.ESP_T + 3.0
    m = pl.Mesh()
    # The board is FLIPPED, so its underside carries the WROOM module and the
    # USB-C shells and nothing may touch the middle. It rides on two rails
    # directly under the header rows -- the stiffest line on the board, and
    # the only full-length strip of bare PCB on that face.
    for hy in (y0 + 2.54, y1 - 2.54):
        m += pl.prism(box(x0 - 2.0, hy - 2.0, x1 + 2.0, hy + 2.0), P.FLOOR - OVL, top)
    m += pl.prism(box(x0 - 3.0, y0 - 2.0, x0 - 1.0, y1 + 2.0), P.FLOOR - OVL, cage_z)
    for tx in (x0 + 16.0, x0 + 46.0):
        m += pl.prism(box(tx - 6, y0 - 2.1, tx + 6, y0 - 0.1), P.FLOOR - OVL, cage_z)
        m += pl.prism(box(tx - 6, y1 + 0.1, tx + 6, y1 + 2.1), P.FLOOR - OVL, cage_z)
    # Fixed lip on the -X END. It has to go on an end, not a long side: the
    # header rows run to within 0.3 mm of both Y edges, and only the X ends
    # carry bare PCB (2.8 mm). The board tucks under this, then esp-tab
    # screws down on the far end.
    m += pl.prism(box(x0 - 1.0, y0 + 3.0, x0 + P.BOARD_LIP, y1 - 3.0),
                  top + P.ESP_T, top + P.ESP_T + P.BOARD_LIP_T)
    # M2 post just off the +X end for esp-tab
    m += pl.prism(box(x1 + 0.4, BY - 3.0, x1 + 6.0, BY + 3.0).difference(
        affinity.translate(pl.circle(P.M2_PILOT), x1 + 3.2, BY)),
        P.FLOOR - OVL, top + P.ESP_T)
    return m


def _speaker_bay():
    """Speaker pocket -- open at the TOP and across its WHOLE BACK.

    Nothing free-stands in the bay and nothing closes the pocket behind the
    driver: no clamp posts, no bulkhead, no back lip, no sealed sub-box. The
    driver goes in from the BACK, lands on a shelf, and spk-clamp holds it
    there -- screwed into two bores sunk in the pocket's own END RAILS, which
    are the frame's end walls, material already there.

    The leads leave the driver's back face into open bay, and the whole tub
    (~749 cm3) is the back volume.
    """
    xf = P.SPK_CTR_X + P.SPK_T + P.SPK_FIT
    y0 = P.SPK_CTR_Y - P.SPK_L / 2 - P.SPK_FIT
    y1 = P.SPK_CTR_Y + P.SPK_L / 2 + P.SPK_FIT
    z0 = P.SPK_CTR_Z - P.SPK_W / 2 - P.SPK_FIT
    z1 = P.SPK_CTR_Z + P.SPK_W / 2 + P.SPK_FIT
    land = box(-R, y0 - P.SPK_RAIL_W, xf + 2.0,
               y1 + P.SPK_RAIL_W).intersection(OUTER)
    # The window runs CLEAR THROUGH the land's back face (which is at xf + 2,
    # so +10 is well past it). Stopping it at xf -- which is what it used to
    # do -- left a 2 mm lip spanning the pocket's full 40.8 x 20.8 mm and
    # sealed the driver into a closed box: no way to feed the leads out, and
    # the only way in was down a 20.8 mm slot. What survives of the frame is
    # the two END RAILS and nothing else.
    window = box(-R - 6, y0, xf + 10.0, y1)
    m = pl.Mesh()
    frame = land.difference(window)
    # Screw bores go DOWN into the end rails from the pocket mouth. The rail
    # is the frame's own end wall, so this costs no extra material and puts
    # nothing behind the driver.
    bores = unary_union([affinity.translate(pl.circle(P.M2_PILOT),
                                            P.SPK_SCREW_X, py)
                         for py in (y0 - P.SPK_RAIL_W / 2,
                                    y1 + P.SPK_RAIL_W / 2)])
    m += pl.banded(frame, z0, z1, [(_speaker_slots(), 14.0, 30.0),
                                   (bores, z1 - P.SPK_SCREW_DEPTH, z1)])
    m += pl.prism(land, z0 - 2.0, z0)
    return m


def _mic_mount():
    """A pocket in the FRONT WALL facing the ports -- the mic has to look out
    through them. (The old version extruded its ring in Z, which made a
    vertical tube: the board would have faced the ceiling, not the world.)

    The pocket axis runs along Y, so it is cut as Z slices of a disc, and the
    board presses in from inside the base. One M2 post takes the retainer
    tab so it cannot rattle loose."""
    d = P.MIC_D + P.MIC_FIT
    wall_in = -R + P.WALL_STRUCT
    boss_y = wall_in + P.MIC_POCKET_D + 2.4
    cx, cz = P.MIC_CTR_X, P.MIC_CTR_Z
    half = d / 2 + 3.4
    z0, z1 = cz - half, cz + half
    boss = box(cx - half, wall_in - OVL, cx + half, boss_y)
    mouth = box(cx - d / 2, boss_y - P.MIC_POCKET_D, cx + d / 2, boss_y + OVL)
    # U-slot: circular below the centre, straight-sided above, so the board
    # DROPS IN from the top. A blind disc pocket would need the board to come
    # in sideways through solid plastic.
    cuts = [(box(cx - w, boss_y - P.MIC_POCKET_D, cx + w, boss_y + OVL), zl, zh)
            for zl, zh, w in J._disc_slices(d, 0.0, cz) if zh <= cz + 1e-6]
    cuts.append((mouth, cz, z1 + OVL))
    m = pl.banded(boss, z0, z1, cuts)
    # two vertical retaining lips across the slot mouth. They run the full
    # height of the pocket, so they start on solid plastic and print clean --
    # a lip that only spanned the top would begin in mid-air.
    for sx in (-1, 1):
        lx = sx * (d / 2 - 0.6)
        m += pl.prism(box(min(lx, sx * (d / 2 + 0.6)), boss_y - 0.8,
                          max(lx, sx * (d / 2 + 0.6)), boss_y),
                      cz - d / 2 + 2.0, z1)
    m += pl.prism(box(cx - 1.6, wall_in - OVL, cx + 1.6, boss_y),   # wire notch
                  z0 - 2.0, z0 + OVL)
    return m


def _amp_pocket():
    """A frame platform, not corner ridges. The amp is FLIPPED (pins up, so
    you can jumper it), which puts its 10 mm screw terminal underneath -- the
    terminal hangs down through the middle of the frame while the PCB's edges
    rest on the rim."""
    ax, ay = P.AMP_CTR
    outer = pl.rounded_rect(P.AMP_L + 5.0, P.AMP_W + 5.0, 2.0)
    inner = pl.rounded_rect(P.AMP_L - 3.0, P.AMP_W - 3.0, 1.5)
    frame = affinity.translate(pl.ring2d(outer, inner), ax, ay)
    m = pl.prism(frame, P.FLOOR - OVL, P.FLOOR + P.AMP_STANDOFF)
    # four corner tabs locating the PCB in XY
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx = ax + sx * (P.AMP_L / 2 + 1.4)
            cy = ay + sy * (P.AMP_W / 2 - 3.0)
            m += pl.prism(box(min(cx, cx - sx * 1.6), min(cy, cy - sy * 5.0),
                              max(cx, cx - sx * 1.6), max(cy, cy - sy * 5.0)),
                          P.FLOOR + P.AMP_STANDOFF - OVL,
                          P.FLOOR + P.AMP_STANDOFF + P.AMP_T + 1.6)
    # Those corner tabs overhang the PCB by 0.2 mm, which locates it in XY and
    # holds it down not at all. Fixed lip on the -X end, amp-tab screwed on
    # the +X end. Ends, not sides: the header runs to 0.3 mm of the +Y edge.
    pcb_top = P.FLOOR + P.AMP_STANDOFF + P.AMP_T
    m += pl.prism(box(ax - P.AMP_L / 2 - 1.6, ay - 6.0,
                      ax - P.AMP_L / 2 + P.BOARD_LIP, ay + 2.0),
                  pcb_top, pcb_top + P.BOARD_LIP_T)
    m += pl.prism(box(ax + P.AMP_L / 2 + 0.4, ay - 5.0,
                      ax + P.AMP_L / 2 + 6.0, ay + 1.0).difference(
        affinity.translate(pl.circle(P.M2_PILOT), ax + P.AMP_L / 2 + 3.2, ay - 2.0)),
        P.FLOOR - OVL, pcb_top)
    return m


def _cap_clamp():
    cx, cy = P.CAP_CTR
    d = P.CAP_D + P.CAP_FIT
    ring = pl.ring2d(affinity.translate(pl.circle(d + 4.0), cx, cy),
                     affinity.translate(pl.circle(d), cx, cy))
    mouth = box(cx, cy - d * 0.34, cx + d, cy + d * 0.34)
    return pl.prism(ring.difference(mouth), P.FLOOR - OVL, P.FLOOR + 12.0)


def _tie_bars():
    m = pl.Mesh()
    for bar, slots in ((box(-34.0, -62.0, 12.0, -56.0), [-24.0, -2.0]),
                       (box(-14.0, 46.0, 14.0, 52.0), [-6.0, 6.0])):
        bar = bar.intersection(IN_THICK)
        cut = unary_union([box(x - P.CLIP_SLOT[0] / 2, bar.bounds[1] - 1.0,
                               x + P.CLIP_SLOT[0] / 2, bar.bounds[3] + 1.0)
                           for x in slots])
        m += pl.banded(bar, P.FLOOR - OVL, P.FLOOR + 8.0, [(cut, 3.0, 6.0)])
    return m


def build():
    m = pl.Mesh()
    steps = [_floor, _walls, _usb_boss, _shoulder_bosses, _board_bay,
             _speaker_bay, _mic_mount, _amp_pocket, _cap_clamp, _tie_bars]
    if P.BULKHEADS:
        steps.append(_bulkheads)
    for f in steps:
        m += f()
    return [("base", m, P.COLORS["base"])]


if __name__ == "__main__":
    ok = True
    for name, mesh, _c in build():
        r = pl.validate(mesh)
        fit, d = pl.fits_build_plate(mesh)
        print(f"{name}: shells={r['shells']} tris={r['triangles']} "
              f"watertight={r['watertight']} fits={fit} "
              f"bbox={tuple(round(float(v),1) for v in d)} {r['problems'][:3]}")
        ok &= r["watertight"] and fit
    sys.exit(0 if ok else 1)
