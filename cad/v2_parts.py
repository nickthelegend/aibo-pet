"""v2_parts.py — HOTARU 2.0. v1 is deprecated.

What changed and why, from the printed v1 and the reference mechanism the
user supplied (a Chinese "robot arm 3.18.1" 3MF, measured, plus photos of a
flat-link desk arm):

  pan        v1 had no pan at all; the arm sat on a fixed lid. 2.0 puts a
             MG996R dead centre in the tub, spline up, driving a turntable
             disc. 180 degrees in the plane, 1:1, no gears. The reference
             gears a micro servo into a ring gear because a 9 g servo is
             weak; an MG996R direct-drives the same load and a 180 servo
             through a reduction cannot reach 180 at the table, so its
             gearing is exactly what we do not copy.
  bearing    the disc rides the tub RIM (r72..75), not the servo shaft. The
             one good idea in the reference at our scale: a big slew surface
             takes the arm's tipping moment; the spline only takes torque.
  links      flat sandwich plates with round joint caps, like the reference
             photos, instead of v1's printed tubes. Each link is two plates;
             the next joint's servo body IS the structure between them.
  bed        printed on a P1S: 256 x 256, so the tub, disc and links plate
             together instead of nine A1-mini plates.

Drive interfaces are v1's, unchanged, because they are printed and proven:
cross-recess horn pocket (HORN_*), threaded stub axle + yoke-screw idler
(AXLE_*, SCREW_*), servo tab clamp bars (cup-cap concept). The v1 cone shade
mounts on the head end UNMODIFIED -- the head block copies v1's housing
interface dimensions (H_HX half-width, SG90 spline on axis, AXLE_D stub).
"""
from __future__ import annotations

import math
import os
import sys

from shapely import affinity
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import partlib as pl

OVL = pl.OVL
BED = 256.0                     # P1S

# ---------------------------------------------------------------- layout ----
TUB_OD = 150.0
TUB_WALL = 3.0
TUB_H = 46.0                    # rim top = slew surface
FLOOR = 2.4

DISC_T = 4.5
DISC_Z0 = TUB_H                 # underside rides the rim
SKIRT_ID = TUB_OD + 0.6         # 0.3 per side round the rim
SKIRT_T = 2.4
SKIRT_DROP = 6.5
# The face reaches PAST the skirt: the first cut had the skirt at r75.3..77.7
# under a face that ended at r74 -- a ring attached to nothing. Each shell
# was watertight on its own, which is exactly why per-part validate() said
# nothing; the over-air audit is what caught it.
DISC_OD = SKIRT_ID + 2 * SKIRT_T

# pan servo: standing on the floor, spline tip at 2.4 + 47.6 = 50.0, which is
# 0.5 below the disc top face (50.5) -- the M3 goes down the centre into it
PAN_TAB_Z = FLOOR + P.MG_TAB_Z

# shoulder tower
TWR_W = 51.3                    # |-25.85| + 25.45, the derived cheek span
TWR_AXIS_Z = 45.0               # above disc top
TWR_BASE = (74.0, 44.0, 4.0)
TWR_H = TWR_AXIS_Z + 14.0

# links: axis to axis
L1 = 100.0                      # shoulder -> elbow
L2 = 90.0                       # elbow -> head
PLATE_T = 4.0
CAP_D = 46.0                    # round joint ends
LINK_W = 34.0                   # plate width between caps
GAP_FIT = 0.4                   # per side, yoke over tower / link over link

# ---- the joint stack, derived once and audited, never transcribed ----
# An MG996R crosses a joint along its HEIGHT: 42.9 of case plus 4.7 of boss
# and spline, 47.6 spline tip to tail. The first cut sized the sandwich to
# the servo's WIDTH (19.7) and only placing the real servo meshes exposed
# it. Every face below is a number with a reason:
#
#   case centred: top +21.45, tail -21.45           (42.9 case)
#   drive cheek  [21.45, 25.45]   4 thick, web 3.0 at its inner face and a
#                                 O20 x 1.0 counterbore at its outer, so the
#                                 boss pokes 1.0 into the counterbore and the
#                                 horn socket takes 1.7 of spline -- v1's
#                                 printed numbers exactly
#   idler cheek  [-25.85,-21.85]  4 thick, threaded stub outward, len 5
#   link1 inner faces: drive +25.75 (cheek outer +0.3), idler -26.15
#   shoulder horn: back 24.45, face 27.25 -> 1.5 into the 25.75 recess
#   elbow mirrors it inside link1 (drive -X): case top on link1-out's inner
#   face -26.15, horn face -31.95, link2-in inner -30.45 -> 1.5 engagement
#   elbow case tail +16.75: fully INSIDE the sandwich. No window, no proud
#   tail, no swept annulus -- the truncated idler plate is no longer needed
#   for clearance but stays for weight.
CASE_TOP = 21.45
DRIVE_CHEEK = (21.45, 25.45)
IDLER_CHEEK = (-25.85, -21.85)
L1_IN_HALF = 25.75
L1_OUT_HALF = 26.15
L2_IN_HALF = 30.45
L2_OUT_HALF = 30.05
STUB_LEN = 5.0
BOSS_WALL = 3.0
LINK1_HALF = L1_IN_HALF          # kept as an alias for the audit
LINK2_HALF = L2_IN_HALF
LINK2_OUT_HALF = L2_OUT_HALF
LINK2_OUT_START = 32.0
HUB_BORE = P.HORN_HUB_D + P.HORN_FIT
ELBOW_TAIL = -26.15 + P.MG_H     # +16.75, must stay inside L1_IN_HALF

# head: nose sized to the shade's MEASURED 37.3 yoke gap; tail spans link2
HEAD_HALF = 18.3
HEAD_TAIL_W = L2_IN_HALF + L2_OUT_HALF - 2 * GAP_FIT   # 59.7
HEAD_NOSE_LEN = 26.0
HEAD_BLOCK_H = 30.0

# Standoff and ledge positions, shared by the plates (through-holes), the
# standoff parts (inserts) and the audit's hole-exists probe.
def l1_spots():
    # x=32: the shoulder servo's upper tab sits at +16.45 with its clamp
    # bar above it; a post at 20 sat straight on the tab
    return [(32.0, -LINK_W / 2 + 6), (32.0, LINK_W / 2 - 6)]

def l1_ledge_spots():
    # ledge centres sit just BEYOND the tab tips, so the 9-wide posts clear
    # the 40.7 case with margin while the tabs still land on them
    bc = L1 - (P.MG_L / 2.0 - P.MG_SHAFT_OFF)        # elbow body centre
    return [(bc - P.MG_TAB_SPAN / 2 - 1.8, 0.0),
            (bc + P.MG_TAB_SPAN / 2 + 1.8, 0.0)]

def l2_spots():
    return [(LINK2_OUT_START + 8, -LINK_W / 2 + 6),
            (LINK2_OUT_START + 8, LINK_W / 2 - 6), (47.0, 0.0)]

SEG = 96

COLORS = {
    "v2-tub": "#E9E9EE", "v2-disc": "#E9E9EE", "v2-tower": "#E9E9EE",
    "v2-link1-in": "#E9E9EE", "v2-link1-out": "#E9E9EE",
    "v2-link2-in": "#E9E9EE", "v2-link2-out": "#E9E9EE",
    "v2-cap": "#26262B", "v2-clamp": "#26262B", "v2-accent": "#4D17F5",
}


def _stadium(length, w, cap_d):
    """Sculpted link outline, per the reference: full cap discs at the
    joints, a TAPERED body between them, everything filleted. A parallel
    stadium is what read as stale."""
    a = pl.circle(cap_d, SEG)
    b = affinity.translate(pl.circle(cap_d, SEG), length, 0)
    body = Polygon([(4, -w / 2 - 2), (length - 4, -w / 2 + 4),
                    (length - 4, w / 2 - 4), (4, w / 2 + 2)])
    return pl.smooth(unary_union([a, body, b]), 6.0)


def _skeleton(length, keepout):
    """The truss cutouts and glow slots that make the reference read as a
    machine instead of a plank. Cut candidates are clipped to the plate's
    safe interior and then MINUS the functional keepout, so styling can
    never eat a bore, a recess, a standoff seat or the servo window."""
    tris = []
    x0, x1 = CAP_D / 2 + 8.0, length - CAP_D / 2 - 8.0
    if x1 - x0 > 24:
        xm = (x0 + x1) / 2
        tris.append(Polygon([(x0, -LINK_W / 2 + 7), (xm - 3, -2),
                             (x0, LINK_W / 2 - 7)]))
        tris.append(Polygon([(x1, -LINK_W / 2 + 7), (xm + 3, -2),
                             (x1, LINK_W / 2 - 7)]))
    tris = [pl.smooth(t, 3.0) for t in tris if t.area > 40]
    slots = [affinity.translate(pl.slot(13.0, 4.2), length - 37.0, dy)
             for dy in (-9.0, 0.0, 9.0)]
    cuts = unary_union(tris + slots)
    return cuts.difference(keepout.buffer(3.0))


def _horn_recess_openings(plate, x_axis, z0, t):
    """v1 cross recess, expressed as banded() openings cut into a flat plate
    lying in XY (z0..z0+t): cross arms + hub bore, centred at x_axis."""
    aw = P.HORN_ARM_W + P.HORN_FIT
    hub = P.HORN_HUB_D + P.HORN_FIT
    rec_d = P.HORN_T + P.HORN_FIT
    cx, cy = x_axis
    arm1 = box(cx - P.HORN_ARM_HALF, cy - aw / 2, cx + P.HORN_ARM_HALF, cy + aw / 2)
    arm2 = box(cx - aw / 2, cy - P.HORN_ARM_HALF, cx + aw / 2, cy + P.HORN_ARM_HALF)
    hubc = pl.circle(hub, 48)
    hubc = affinity.translate(hubc, cx, cy)
    return [
        (unary_union([arm1, arm2]), z0 - OVL, z0 + rec_d),   # recess, part way
        (hubc, z0 - OVL, z0 + t + OVL),                       # hub clear through
    ]


# ------------------------------------------------------------------ tub ----
def tub():
    R = TUB_OD / 2.0
    m = pl.Mesh()
    # Pebble profile: the wall blends out of the desk through a 12 mm fillet
    # and bulges 3 past the rim diameter through the middle -- the reference
    # base's soft soap-bar read -- then lands back exactly on O150 at the
    # rim, because the rim is the slew surface and the slew is not styling.
    import math as _math
    # Base blend is a straight 40-degree chamfer, not a sine: the sine's
    # tangent at z=0 sat at 46 degrees and the audit flagged the first two
    # millimetres of wall as over air. A 16-over-19 line never exceeds 40.
    # The bulge window still ends at 36 -- the disc's skirt hangs to 39.5 at
    # r75.3, and the first curve peaked at r76.1 inside it.
    def od(z):
        if z < 19.0:
            return TUB_OD - 16.0 + 16.0 * (z / 19.0)
        if z < 36.0:
            t = (z - 19.0) / 17.0
            return TUB_OD + 6.0 * _math.sin(t * _math.pi)
        return TUB_OD
    # The wall is a stack of banded rings, not a revolve_shell: this kernel
    # has no 3D CSG, so a lofted shell cannot take a window afterwards. The
    # rings can. Two FLAT FACETS are shaved into the profile (front -Y and
    # back +Y) so the MX switch and the USB port get clean rectangular
    # openings in flat wall instead of ragged holes in a curve -- the
    # reference base does exactly this for its side port.
    FACET_Y = 66.0
    from shapely.geometry import box as _box
    def _facets(prof):
        cut_n = _box(-60, -95, 60, -FACET_Y)
        cut_p = _box(-60, FACET_Y, 60, 95)
        return prof.difference(cut_n).difference(cut_p)

    # wall openings, all expressed in XY + a z band:
    mx_win  = _box(-P.MX_CUT / 2, -95, P.MX_CUT / 2, -FACET_Y + 1.5)
    mx_rel  = _box(-P.MX_BODY_SQ / 2, -95, P.MX_BODY_SQ / 2, -FACET_Y - P.MX_PLATE_T)
    usb_win = _box(-P.USB_PLUG_W / 2, FACET_Y - 8.0, P.USB_PLUG_W / 2, 95)
    mic_hole = affinity.translate(pl.circle(P.MIC_PORT_D, 24), -74.0, 0.0)
    mic_hole = mic_hole.union(_box(-80, -P.MIC_PORT_D / 2, -70, P.MIC_PORT_D / 2))
    grille = unary_union([
        affinity.rotate(_box(60.0, -1.1, 80.0, 1.1), a, origin=(0, 0))
        for a in (-18, -12, -6, 0, 6, 12, 18)])

    WALL_OPEN = [
        (mx_win,  22.0, 22.0 + P.MX_CUT),        # switch snaps into the plate
        (mx_rel,  20.0, 20.0 + P.MX_BODY_SQ + 2.0),   # relief behind the plate
        (usb_win, FLOOR + 2.1, FLOOR + 2.1 + P.USB_PLUG_H),
        (mic_hole, 26.0, 26.0 + P.MIC_PORT_D),
        (grille,   8.0, 26.0),
    ]
    steps = 40
    for k in range(steps):
        za = TUB_H * k / steps
        zb = TUB_H * (k + 1) / steps + (OVL if k < steps - 1 else 0.0)
        o = od(za)
        ring = _facets(pl.ring2d(pl.circle(o, 160),
                                 pl.circle(o - 2 * TUB_WALL, 160)))
        cuts = unary_union([g for g, zl, zh in WALL_OPEN
                            if zl <= za + 0.01 and zh >= zb - 0.01] or
                           [Polygon()])
        ring = ring.difference(cuts)
        if not ring.is_empty:
            m += pl.prism(ring, za, zb)
    m += pl.prism(_facets(pl.circle(od(0.4), 160)), 0.0, FLOOR)   # floor

    # ---- the rear crown: the bulged / non-bulged split the pan RANGE asks
    # for. The MG996R sweeps 180 degrees, all of it over the +Y front, so
    # the front rim stays LOW (the arm and shade fold down past it) and the
    # rear rises into a shroud around the disc, the reference base's tall
    # back. Sector 195..345 degrees, wall 78.2..81.2 -- 0.5 outside the
    # skirt's 77.7 so the disc still drops in from above -- grounded to the
    # floor through its own 40-degree foot, capped flat at 56.
    from shapely.geometry import Polygon as _Poly
    def _sector(a0, a1):
        pts = [(0.0, 0.0)] + [
            (200.0 * math.cos(math.radians(a0 + t * (a1 - a0) / 40)),
             200.0 * math.sin(math.radians(a0 + t * (a1 - a0) / 40)))
            for t in range(41)]
        return _Poly(pts)
    CR_IR, CR_OR = 78.2, 81.2
    sec = _sector(195.0, 345.0)
    mx_gap = _box(-P.MX_BODY_SQ / 2 - 4, -95, P.MX_BODY_SQ / 2 + 4, -60)
    for k in range(28):
        za = 56.0 * k / 28.0
        zb = 56.0 * (k + 1) / 28.0 + (OVL if k < 27 else 0.0)
        ro = CR_OR if za >= 8.0 else CR_OR - (8.0 - za) * 1.1
        ring = pl.ring2d(pl.circle(2 * ro, 160), pl.circle(2 * CR_IR, 160))
        ring = ring.intersection(sec)
        # the MX switch keeps its window: the crown parts around it, which
        # reads as the reference's deliberate rear port valley
        ring = ring.difference(mx_gap)
        if not ring.is_empty:
            m += pl.prism(ring, za, zb)

    # MX plate boss: the 1.5 plate the switch clips into is the facet itself;
    # a frame around the relief stiffens it from inside
    frame = _box(-P.MX_BODY_SQ / 2 - 3, -FACET_Y + P.MX_PLATE_T,
                 P.MX_BODY_SQ / 2 + 3, -FACET_Y + P.MX_PLATE_T + 2.6)
    m += pl.prism(frame, FLOOR - OVL, 44.0)   # grounded, no over-air
    # USB tunnel: solid land bridging facet to the board edge, plug cavity cut
    tun = _box(-P.USB_PLUG_W / 2 - 2.4, 59.0, P.USB_PLUG_W / 2 + 2.4, FACET_Y + OVL)
    cav = _box(-P.USB_PLUG_W / 2, 58.0, P.USB_PLUG_W / 2, FACET_Y + 1)
    m += pl.banded(tun, FLOOR - OVL, FLOOR + 2.1 + P.USB_PLUG_H + 2.4,
                   [(cav, FLOOR + 2.1, FLOOR + 2.1 + P.USB_PLUG_H)])
    # mic cradle behind its port
    mic_boss = affinity.translate(pl.circle(P.MIC_D + 3.2, 48), -66.0, 0.0)
    mic_pock = affinity.translate(pl.circle(P.MIC_D + 2 * P.MIC_FIT, 48), -66.0, 0.0)
    m += pl.banded(mic_boss.difference(_box(-95, -20, -73, 20)), FLOOR - OVL, 38.0,
                   [(mic_pock, 22.0, 38.0 + OVL)])

    # Disc retention is the pan horn's centre M3, full stop. A clip system
    # was tried and produced three unsolvable conflicts (a radial screw
    # would have to cross the ROTATING skirt). The load says it was never
    # needed: 3 N at 200 mm of reach is 0.6 Nm, which is ~8 N of uplift at
    # the centre against a steel M3 in the spline. Documented, not hidden.

    # ---- pan servo mount: pocket walls + two tab pillars ----
    # Body centre is +10.35 from the spline, not -10.35: the component mesh
    # was probed (body spans spline -10 .. +30.7) after the world sweep put
    # 2409 points inside pillar and body at once. Probe the mesh, never
    # transcribe the datasheet's sign convention.
    bx = P.MG_L / 2.0 - P.MG_SHAFT_OFF
    body = box(bx - P.MG_L / 2 - 0.4, -P.MG_W / 2 - 0.4,
               bx + P.MG_L / 2 + 0.4, P.MG_W / 2 + 0.4)
    collar = body.buffer(2.4, join_style=2).difference(body)
    m += pl.prism(collar, FLOOR - OVL, 16.0)                    # lateral collar
    for sx in (-1, 1):
        px = bx + sx * (P.MG_TAB_SPAN / 2 - 3.2)
        pil = box(px - 5.0, -P.MG_W / 2 - 0.4, px + 5.0, P.MG_W / 2 + 0.4)
        pil = pil.difference(body)
        # the lead leaves the case at the tail face; the tail-side pillar
        # gets a notch (a 16-wide bridge, anchored on its y-legs)
        wire_w = box(px - 5.0 - OVL, -8.0, px + 5.0 + OVL, 8.0)
        m += pl.banded(pil, FLOOR - OVL, PAN_TAB_Z,
                       [(wire_w, 12.0, 22.0)] if sx < 0 else [])
        # M3 insert bosses beside the tab, for the clamp bar
        for sy in (-1, 1):
            bpos = (px, sy * (P.MG_W / 2 + 3.6))
            bos = affinity.translate(pl.circle(7.4, 32), *bpos)
            bor = affinity.translate(pl.circle(P.M3_INSERT_D, 24), *bpos)
            m += pl.prism(bos.difference(bor), FLOOR - OVL, PAN_TAB_Z + 6.0)

    # ---- electronics, on the floor, v1 footprints ----
    def posts(cx, cy, dx, dy):
        pp = pl.Mesh()
        for ex, ey in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            p = (cx + ex * dx / 2, cy + ey * dy / 2)
            b = affinity.translate(pl.circle(5.4, 24), *p)
            h = affinity.translate(pl.circle(P.M2_PILOT, 16), *p)
            pp += pl.prism(b.difference(h), FLOOR - OVL, FLOOR + 5.0)
        return pp
    m += posts(0.0, 44.0, P.ESP_L - 5, P.ESP_W - 5)             # ESP32-S3
    m += posts(-38.0, -40.0, P.AMP_L - 4, P.AMP_W - 4)          # MAX98357A

    # speaker against +X wall: two rails + open back grille (v1 lesson: the
    # pocket stays open behind the driver)
    # Speaker seat, INBOARD at x=58: the pebble chamfer pulls the wall away
    # from under the old wall-hugging position, and the rails' outboard ends
    # stood on the void (58 mm2, found at r70 by the over-air audit). On the
    # flat floor there is nothing to hang over. The wall grille lands in the
    # electronics pass, aimed at this seat.
    sx0 = 58.0
    for sy in (-1, 1):
        y_in = sy * (P.SPK_L / 2 + 0.4)
        rail = box(sx0 - 3.0, min(y_in, y_in + sy * 3.0),
                   sx0 + P.SPK_T + 1.2, max(y_in, y_in + sy * 3.0))
        m += pl.prism(rail, FLOOR - OVL, FLOOR + 26.0)

    # mic port + USB window + MX plate would go through the wall here; kept
    # for the v2 electronics pass so this file stays reviewable
    return [("v2-tub", m, COLORS["v2-tub"])]


# ----------------------------------------------------------------- disc ----
def disc():
    m = pl.Mesh()
    face = pl.circle(DISC_OD, 160)

    # kidney slot for the arm harness: +Y side, 70 degrees of arc
    ring = pl.ring2d(pl.circle(64.0, 96), pl.circle(48.0, 96))
    wedge = Polygon([(0, 0)] + [(90 * math.cos(math.radians(a)),
                                 90 * math.sin(math.radians(a)))
                                for a in [55 + k * (70 / 24) for k in range(25)]])
    kidney = ring.intersection(wedge)

    # centre: screw counterbore + clearance, and the horn cross recess below
    cb = pl.circle(6.8, 32)
    thr = pl.circle(P.M3_CLEAR, 24)

    openings = _horn_recess_openings(face, (0.0, 0.0), DISC_Z0, DISC_T)
    openings += [
        (kidney, DISC_Z0 - OVL, DISC_Z0 + DISC_T + OVL),
        (thr, DISC_Z0 - OVL, DISC_Z0 + DISC_T + OVL),
        (cb, DISC_Z0 + DISC_T - 2.0, DISC_Z0 + DISC_T + OVL),
    ]
    # 4 insert bores for the tower flange
    for (ix, iy) in _tower_bolts():
        openings.append((affinity.translate(pl.circle(P.M3_INSERT_D, 24), ix, iy),
                         DISC_Z0 - OVL, DISC_Z0 + DISC_T + OVL))
    # Platter arcs, cut CLEAR THROUGH: a raised hub clashed with the tower
    # base; an engraved groove over-airs its ceiling when the show face
    # prints down. A through slot has neither failure mode, prints clean in
    # both orientations, and matches the reference's vent language. Two
    # rings of four 70-degree arcs, outside the tower base's 43 mm corners.
    for gd, w in ((98.0, 3.2), (114.0, 3.2)):
        ringc = pl.ring2d(pl.circle(gd + w, 128), pl.circle(gd - w, 128))
        for k in range(4):
            a0 = 10.0 + k * 90.0
            wed = Polygon([(0, 0)] + [
                (95.0 * math.cos(math.radians(a0 + t * 70.0 / 20)),
                 95.0 * math.sin(math.radians(a0 + t * 70.0 / 20)))
                for t in range(21)])
            openings.append((ringc.intersection(wed),
                             DISC_Z0 - OVL, DISC_Z0 + DISC_T + OVL))
    m += pl.banded(face, DISC_Z0, DISC_Z0 + DISC_T, openings)

    # rim skirt: hides the joint and locates the disc round the rim
    sk = pl.ring2d(pl.circle(SKIRT_ID + 2 * SKIRT_T, 160), pl.circle(SKIRT_ID, 160))
    m += pl.prism(sk, DISC_Z0 - SKIRT_DROP, DISC_Z0 + OVL)
    return [("v2-disc", m, COLORS["v2-disc"])]


def _tower_bolts():
    return [(sx * 28.0, sy * 16.0) for sx in (-1, 1) for sy in (-1, 1)]


# ---------------------------------------------------------------- tower ----
def tower():
    """Shoulder mount. Prints on its back. Bolts to the disc, 4x M3.
    Cheek faces come from the derived joint stack at the top of this file;
    the servo hangs between them on two tab posts, clamped by printed bars."""
    z0 = DISC_Z0 + DISC_T
    m = pl.Mesh()
    bw, bd, bt = TWR_BASE
    base = pl.rounded_rect(bw, bd, 6.0)
    holes = unary_union([affinity.translate(pl.circle(P.M3_CLEAR, 24), x, y)
                         for x, y in _tower_bolts()])
    heads = unary_union([affinity.translate(pl.circle(P.M3_HEAD_D + 0.6, 24), x, y)
                         for x, y in _tower_bolts()])
    m += pl.banded(base, z0, z0 + bt, [
        (holes, z0 - OVL, z0 + bt + OVL),
        (heads, z0 + bt - 1.6, z0 + bt + OVL)])

    ax_z = z0 + TWR_AXIS_Z
    top = ax_z + 22.0

    # drive cheek +X: web 3.0 at the inner face, counterbore at the outer
    x0, x1 = DRIVE_CHEEK
    prof = box(x0, -bd / 2 + 2, x1, bd / 2 - 2)
    bore = [(box(x0 - OVL, -w, x1 + OVL, w), zl, zh)
            for zl, zh, w in _slices(P.MG_BOSS_D + 1.0, ax_z)]
    cb = [(box(x0 + BOSS_WALL, -w, x1 + OVL, w), zl, zh)
          for zl, zh, w in _slices(P.MG_BOSS_D + 7.0, ax_z)]
    # the case's four screw bosses stand ~0.5 proud of its top face; relieve
    # the cheek 1.0 where they land (probed at 16..23 below the axis)
    relief = [(box(x0 - OVL, -10.4, x0 + 1.0, 10.4), ax_z - 31.2, ax_z - 11.0),
              # the case's secondary hump (probed O7 x 2.0 at axis-20) gets a
              # clear window; the cheek is wide, the hole is not structural
              (box(x0 - OVL, -4.9, x1 + OVL, 4.9), ax_z - 24.9, ax_z - 15.1)]
    m += pl.banded(prof, z0 + bt - OVL, top, bore + cb + relief)

    # idler cheek -X: with the threaded stub outward on the axis, and a
    # notch where the servo's lead leaves the case tail -- the world sweep
    # put 1660 probe points inside cheek and wire at once, which is the
    # audit's way of saying the wire had nowhere to exist.
    x0, x1 = IDLER_CHEEK
    body_c0 = (DISC_Z0 + DISC_T + TWR_AXIS_Z) - (P.MG_L / 2.0 - P.MG_SHAFT_OFF)
    wire = box(x0 - OVL, -8.0, x1 + OVL, 8.0)
    m += pl.banded(box(x0, -bd / 2 + 2, x1, bd / 2 - 2), z0 + bt - OVL, top,
                   [(wire, body_c0 - 26.0, body_c0 - 12.0)])
    ax = pl.threaded_bore(P.AXLE_D, P.SCREW_MAJOR, P.SCREW_PITCH,
                          0.0, P.SCREW_ENGAGE, clearance=P.SCREW_FIT / 2)
    ax.rotate_y(-90.0)
    ax.translate(dx=x0 + OVL, dz=ax_z)
    m += ax

    # tab posts: the servo's tabs sit at case-bottom + 26.6, which is
    # x = +5.15, at z beyond both ends of the 40.7 case. Post face at the tab
    # underside; the clamp bar and an M3 through the drive cheek close it.
    body_c = ax_z - (P.MG_L / 2.0 - P.MG_SHAFT_OFF)
    for tz in (body_c - P.MG_TAB_SPAN / 2 - 1.8, body_c + P.MG_TAB_SPAN / 2 + 1.8):
        pm = pl.prism(affinity.translate(pl.rounded_rect(
            (CASE_TOP - P.MG_TAB_Z) - IDLER_CHEEK[1], 12.0, 2.0),
            ((CASE_TOP - P.MG_TAB_Z) + IDLER_CHEEK[1]) / 2.0, 0.0), tz - 4.5, tz + 4.5)
        m += pm
    return [("v2-tower", m, COLORS["v2-tower"])]


def _slices(d, cz, seg=24):
    """NB: callers get d + 0.8 of slack folded in -- ten slabs approximate
    the bore as a polygon whose steps bite inward, and the shoulder boss
    interfered with exactly those steps in the world sweep."""
    """Horizontal slabs approximating a Y-axis... X-axis bore of diameter d
    centred at height cz -- v1's _disc_slices, reimplemented for banded()."""
    out = []
    r = (d + 0.8) / 2.0
    n = 10
    for k in range(n):
        z1 = cz - r + (2 * r) * k / n
        z2 = cz - r + (2 * r) * (k + 1) / n
        zm = (z1 + z2) / 2.0
        half = math.sqrt(max(r * r - (zm - cz) ** 2, 0.0))
        out.append((z1, z2, half))
    return out


# ---------------------------------------------------------------- links ----
def _link_plate(name, length, colour, near, far, spots=(), holes_at=()):
    """One flat plate, lying in XY at z 0..PLATE_T. near/far feature menu:
    recess | idler | boss3 | window | steadybore | mount | none.
    `spots` get M3 through-holes (standoff screws); `holes_at` likewise (the
    servo-tab clamp screws). Both also become styling keepouts."""
    prof = _stadium(length, LINK_W, CAP_D)
    openings = []
    keep = [affinity.translate(pl.circle(36.0, 48), x0, 0) for x0 in (0.0, length)]
    for x0, feat in ((0.0, near), (length, far)):
        at = lambda g: affinity.translate(g, x0, 0)
        if feat == "recess":
            openings += _horn_recess_openings(prof, (x0, 0.0), 0.0, PLATE_T)
        elif feat == "idler":
            openings.append((at(pl.circle(P.AXLE_D + P.AXLE_FIT, 48)),
                             -OVL, PLATE_T + OVL))
        elif feat == "boss3":
            openings.append((at(pl.circle(P.MG_BOSS_D + 1.0, 48)),
                             -OVL, PLATE_T + OVL))
            openings.append((at(pl.circle(P.MG_BOSS_D + 7.0, 48)),
                             BOSS_WALL, PLATE_T + OVL))
            # relief for the case screw bosses, minus a contact ring at the
            # axis so the case still registers on the plate
            rel = box(x0 - 20.8, -10.4, x0 + 20.8, 10.4).difference(
                at(pl.circle(26.0, 48)))
            openings.append((rel, -OVL, 1.0))
            openings.append((affinity.translate(
                pl.circle(9.8, 32), x0 - 19.7, 0.0), -OVL, PLATE_T + OVL))
        elif feat == "window":
            bc = x0 - (P.MG_L / 2.0 - P.MG_SHAFT_OFF)
            w = box(bc - (P.MG_L + 0.8) / 2, -(P.MG_W + 0.8) / 2,
                    bc + (P.MG_L + 0.8) / 2, (P.MG_W + 0.8) / 2)
            openings.append((w, -OVL, PLATE_T + OVL))
            keep.append(w.buffer(2.0))
        elif feat == "mount":
            for dy in (-10.0, 10.0):
                openings.append((affinity.translate(pl.circle(P.M3_CLEAR, 24), x0, dy),
                                 -OVL, PLATE_T + OVL))
    if "stub" in (near, far):
        x0 = length if far == "stub" else 0.0
        keep.append(affinity.translate(pl.circle(30.0, 48), x0, 0))
    for (cx, cy) in tuple(spots) + tuple(holes_at):
        openings.append((affinity.translate(pl.circle(P.M3_CLEAR, 24), cx, cy),
                         -OVL, PLATE_T + OVL))
        keep.append(affinity.translate(pl.rounded_rect(13.0, 12.0, 2.0), cx, cy))
    cuts = _skeleton(length, unary_union(keep))
    if not cuts.is_empty:
        openings.append((cuts, -OVL, PLATE_T + OVL))
    m = pl.banded(prof, 0.0, PLATE_T, openings)
    if "stub" in (near, far):
        # threaded stub on the OUTER face, so the next link's idler plate
        # rides it and a yoke-screw traps it -- v1's proven pair
        x0 = length if far == "stub" else 0.0
        boss = pl.prism(pl.circle(P.AXLE_D, 48), PLATE_T - OVL, PLATE_T + 0.6)
        boss.translate(dx=x0)
        ax = pl.threaded_bore(P.AXLE_D, P.SCREW_MAJOR, P.SCREW_PITCH,
                              0.0, P.SCREW_ENGAGE, clearance=P.SCREW_FIT / 2)
        ax.translate(dx=x0, dz=PLATE_T + 0.6 - OVL)
        m += boss
        m += ax
    return (name, m, colour)


def _ledge_posts(name, spots, h, colour):
    """Tab ledges whose top 9 mm is two prongs: the servo lead exits at the
    case bottom right where a solid post face would pin it. The interference
    sweep found 1772 probe points inside ledge and wire at once."""
    m = pl.Mesh()
    for (cx, cy) in spots:
        b = affinity.translate(pl.rounded_rect(10.0, 12.0, 2.0), cx, cy)
        hh = affinity.translate(pl.circle(P.M3_INSERT_D, 24), cx, cy)
        m += pl.prism(b.difference(hh), 0.0, h - 9.0)
        slot = affinity.translate(box(-5.5, -3.2, 5.5, 3.2), cx, cy)
        m += pl.prism(b.difference(hh).difference(slot), h - 9.0 - OVL, h)
    return (name, m, colour)


def _standoffs(name, spots, h, colour):
    """M3-insert standoff posts, flat on the bed."""
    m = pl.Mesh()
    for (cx, cy) in spots:
        b = affinity.translate(pl.rounded_rect(10.0, 9.0, 2.0), cx, cy)
        hh = affinity.translate(pl.circle(P.M3_INSERT_D, 24), cx, cy)
        m += pl.prism(b.difference(hh), 0.0, h)
    return (name, m, colour)


def link1():
    c = COLORS["v2-link1-in"]
    return [
        # drive side +X at the shoulder; elbow flips drive to -X. The elbow
        # servo lives ENTIRELY inside the sandwich (tail at +16.75 against an
        # inner face at +25.75), so there is no window and nothing sweeps.
        # far end grows the elbow's idler stub: with the elbow servo now
        # fully inside the sandwich there is nothing proud to dodge, so the
        # elbow is a real two-sided yoke instead of the single-sided
        # compromise -- which is why it read as a missing cap.
        _link_plate("v2-link1-in", L1, c, near="recess", far="stub",
                    spots=l1_spots(), holes_at=l1_ledge_spots()),
        _link_plate("v2-link1-out", L1, c, near="idler", far="boss3",
                    spots=l1_spots()),
        _standoffs("v2-link1-spacers", l1_spots(),
                   L1_IN_HALF + L1_OUT_HALF - 0.4, COLORS["v2-clamp"]),
        # ledge height: from link1-in's inner face down to the tab underside,
        # tail + MG_TAB_Z above the case bottom. Derived, because the first
        # value was transcribed from the wrong end of the servo.
        _ledge_posts("v2-link1-ledges", l1_ledge_spots(),
                     L1_IN_HALF - (ELBOW_TAIL - P.MG_TAB_Z) - P.MG_TAB_T,
                     COLORS["v2-clamp"]),
    ]


def _truncated_plate(name, length, start, colour, far, spots=()):
    """A plate that begins `start` from the near axis: no near cap at all.
    Same sculpt and skeleton treatment as the full plates."""
    prof = pl.smooth(unary_union([
        Polygon([(start, -LINK_W / 2 + 3), (length - 4, -LINK_W / 2),
                 (length - 4, LINK_W / 2), (start, LINK_W / 2 - 3)]),
        affinity.translate(pl.circle(CAP_D, SEG), length, 0)]), 6.0)
    openings = []
    keep = [affinity.translate(pl.circle(36.0, 48), length, 0),
            box(start - 2, -LINK_W / 2, start + 8, LINK_W / 2)]
    if far == "mount":
        for dy in (-10.0, 10.0):
            openings.append((affinity.translate(pl.circle(P.M3_CLEAR, 24), length, dy),
                             -OVL, PLATE_T + OVL))
    for (cx, cy) in spots:
        openings.append((affinity.translate(pl.circle(P.M3_CLEAR, 24), cx, cy),
                         -OVL, PLATE_T + OVL))
        keep.append(affinity.translate(pl.rounded_rect(13.0, 12.0, 2.0), cx, cy))
    cuts = _skeleton(length, unary_union(keep)).intersection(
        prof.buffer(-6.0))
    if not cuts.is_empty:
        openings.append((cuts, -OVL, PLATE_T + OVL))
    m = pl.banded(prof, 0.0, PLATE_T, openings)
    return (name, m, colour)


def link2():
    c = COLORS["v2-link2-in"]
    return [
        _link_plate("v2-link2-in", L2, c, near="recess", far="none",
                    spots=l2_spots(),
                    holes_at=[(L2 - 22.0, -10.0), (L2 - 22.0, 10.0)]),
        _link_plate("v2-link2-out", L2, c, near="idler", far="none",
                    spots=list(l2_spots())
                        + [(L2 - 22.0, -10.0), (L2 - 22.0, 10.0)]),
        _standoffs("v2-link2-spacers", l2_spots(),
                   L2_IN_HALF + L2_OUT_HALF - 0.4, COLORS["v2-clamp"]),
    ]


# ----------------------------------------------------------------- head ----
def head_block():
    """Nose carries the v1 shade on the SG90 axis; tail bolts between
    link2's plates.

    Local frame: X width, Y depth (tail y -13..13, nose y 9..35), Z height
    0..30. In world it is rotated x+90, so local (x,y,z) lands at
    (x, 15-z, z_head-22+y). The SG axis is local (.,22,15).

    The SG90 lies height-along-X: case bottom at x=-17.2, spline tip flush
    with the nose's +X face at 18.3 through a 3.0 wall -- 35.5 total, which
    is the one orientation the servo actually reaches through. The first
    cut pointed its LENGTH along the pocket and the spline at a wall.

    Tail screws: link2's plates bolt through at world (y=+/-10, z_head-22),
    which is local (y=0, z=5) and (z=25). Those are 2.8 x 3.2 tunnels cut
    clear through the tail width; M3s self-thread into the printed flats,
    the same way v1's M2 pilots worked."""
    m = pl.Mesh()
    # HEAD_TAIL_W already nets out the running fit; subtracting it again
    # here left the tail rattling 0.8 per side (the contact row caught it)
    tail_w = HEAD_TAIL_W
    D = 26.0
    tail = pl.rounded_rect(tail_w, D, 5.0)
    # screw tunnels as z-bands cut across the full tail
    tun = box(-tail_w / 2 - OVL, -1.4, tail_w / 2 + OVL, 1.4)
    # the SG's lower length end reaches local y < 13, i.e. INTO the tail
    # band, so the tail carries the same case and tab cuts as the nose
    case_t = box(-17.6, 4.5, 12.7, 28.3)
    tabs_t = box(-4.6, -0.5, 1.4, 33.0)
    hump_t = box(12.3 - OVL, 8.5, 15.5, 14.2)
    m += pl.banded(tail, 0.0, HEAD_BLOCK_H, [
        (tun, 3.4, 6.6), (tun, 23.4, 26.6),
        (case_t, 8.5, HEAD_BLOCK_H + OVL),
        (tabs_t, 8.5, HEAD_BLOCK_H + OVL),
        (hump_t, 12.9, 17.1)])

    nose = affinity.translate(pl.rounded_rect(2 * HEAD_HALF, D, 5.0), 0, D - 4.0)
    # The SG90 sits height-along-X (the only orientation whose spline spans
    # the wall), which puts its LENGTH along local Y and its WIDTH along
    # local Z. The first pocket had length and width swapped and the sweep
    # put 1218 points inside servo and nose at once.
    #   case:   x -17.6..12.7,  y 4.5..28.3,  z 8.5 .. open top
    #   tabs:   thin x column at -1.3, poking past both length ends
    #   boss:   x 12.7..15.4 round-ish channel; spline x 15.4..18.4
    case = box(-17.6, 4.5, 12.7, 31.5)      # +31.5: the lead exits up here
    tabs = box(-4.6, -0.5, 1.4, 33.0)
    bossc = box(12.7 - OVL, 22 - 4.4, 15.4, 22 + 4.4)
    hump = box(12.3 - OVL, 8.5, 15.5, 14.2)
    spl = box(15.4 - OVL, 22 - 2.8, HEAD_HALF + OVL, 22 + 2.8)
    m += pl.banded(nose, 0.0, HEAD_BLOCK_H, [
        (case, 8.5, HEAD_BLOCK_H + OVL),
        (tabs, 8.5, HEAD_BLOCK_H + OVL),
        (bossc, 15.0 - 4.4, 15.0 + 4.4),
        (hump, 12.9, 17.1),
        (spl, 15.0 - 2.8, 15.0 + 2.8),
    ])
    ax = pl.prism(pl.circle(P.AXLE_D, 48), 0.0, P.AXLE_LEN + OVL)
    ax.rotate_y(-90.0)
    ax.translate(dx=-HEAD_HALF + OVL, dy=22.0, dz=15.0)
    m += ax
    return [("v2-head", m, COLORS["v2-tower"])]


def _slot_head(d):
    """A coin-slot disc: the visual signature of the reference's joints.
    The slot goes clear through -- it prints clean in any orientation and
    reads as machined from both sides."""
    return pl.circle(d, 96).difference(box(-d * 0.36, -1.7, d * 0.36, 1.7))


def v2_screw():
    """The shoulder idler's retaining screw, v1's proven M6x2 printed thread
    under a big slotted head. Turned with a coin or a flat blade."""
    m = pl.Mesh()
    m += pl.prism(_slot_head(26.0), 0.0, 3.2)
    m += pl.prism(pl.circle(P.SCREW_MAJOR - 2 * pl.THREAD_RAMP * P.SCREW_PITCH, 48),
                  3.2 - OVL, 4.2)
    m += pl.thread(P.SCREW_MAJOR, P.SCREW_PITCH, 4.2 - OVL, 4.2 + P.SCREW_ENGAGE)
    return [("v2-screw", m, COLORS["v2-accent"])]


def v2_trimcap():
    """The elbow's matching cap: same slotted face, a plug that glues into
    the steady bore after the horn's M3 is home. Cosmetic, and honest about
    it -- the slot turns nothing here."""
    m = pl.Mesh()
    m += pl.prism(_slot_head(32.0), 0.0, 3.2)
    m += pl.prism(pl.circle(HUB_BORE - 0.25, 48), 3.2 - OVL, 3.2 + 2.4)
    return [("v2-trimcap", m, COLORS["v2-accent"])]


def clamp_bars():
    """Servo tab clamp bars: pan (2) + shoulder (2) + elbow (2)."""
    m = pl.Mesh()
    for i in range(6):
        b = affinity.translate(pl.rounded_rect(12.0, 8.0, 2.0), i * 16.0, 0)
        h = affinity.translate(pl.circle(P.M3_CLEAR, 24), i * 16.0, 0)
        m += pl.prism(b.difference(h), 0.0, 4.0)
    return [("v2-clamps", m, COLORS["v2-clamp"])]
