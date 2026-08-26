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
TWR_GAP = P.MG_W + 0.8          # servo drops through
TWR_CHEEK = 8.0
TWR_W = TWR_GAP + 2 * TWR_CHEEK          # 36.9 outer
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

# link1's sandwich gap is set by the TOWER it straddles, not by the servo:
# inner faces at +/-18.85, gap 37.7. An MG996R is 46.9 spline to tail, so at
# the elbow the body passes THROUGH a window in the far plate and its tail
# sits proud on the outboard face -- exactly what the reference photos show.
LINK1_HALF = TWR_W / 2.0 + GAP_FIT            # 18.65 (MG_W is 19.7)
# The elbow is SINGLE SIDED, and that is a measurement, not a preference:
# an MG996R is 47.6 spline tip to tail. Spanning link1's 37.7 gap, a 4 plate
# and a bearing ring leaves the spline 0.5 into the horn -- nothing. v1
# seated its horns through a 3.0 wall for 1.7 of spline, printed, and held;
# so the elbow copies v1 exactly: the boss bore sits in a plate thinned to
# 3.0, the horn lands directly on it, and the horn's M3 is the retention.
# The far plate rides a shallow steady ring for lateral stiffness only.
BOSS_WALL = 3.0
# Drive plate inner face: pinned from BOTH sides. The horn's outer face sits
# at 18.85 + 3.0 + 2.8 = 24.65; engagement into the 3.1 recess is
# (horn_face - LINK2_HALF), so smaller is better -- but the plate must clear
# link1's outer face (18.65 + 4 = 22.65) by 0.3. With the horn face at
# 18.65 + 3.0 + 2.8 = 24.45, LINK2_HALF = 22.95 gives exactly 1.5 of cross
# engagement and exactly 0.3 of running clearance. No slack on either side;
# the audit holds both numbers, and the first cut of this constant was 0.2
# out because a comment claimed MG_W was 19.9. Derive, never transcribe.
LINK2_HALF = 22.95
# The idler side clears the elbow servo's tail, which stands 1.2 proud of
# link1's outer face and SWEEPS as link1 rotates under link2. Nothing on +X
# can coexist with that sweep near the axis, so the outer plate starts 32 mm
# out and never enters the swept annulus. Single sided at the elbow, said
# plainly; the three spacers and the head tail make the box rigid.
LINK2_OUT_HALF = 24.45
LINK2_OUT_START = 32.0
# There is NO steady ring at the elbow, and the reason is now written down
# instead of rediscovered: a ring around the horn seat needs a bore in the
# facing plate, and that plate's face is the one carrying the horn's cross
# recess -- whose arms span 48. The bore would eat the recess. v1's elbow
# bore on the horn hub itself (O13 in a O13.3 bore) and printed fine, so
# 2.0 does the same. The trim cap plugs that hub bore after the M3 is home.
HUB_BORE = P.HORN_HUB_D + P.HORN_FIT

# Standoff and ledge positions, shared by the plates (through-holes), the
# standoff parts (inserts) and the audit (hole-exists check). One list each,
# because the first cut gave the plates NO holes at all -- the outer plate
# could never have been screwed on -- and put the servo-tab ledges at
# +/-25.8 from the elbow axis, where the MG996R's tabs are not: the tabs sit
# at body-centre +/-26.8, and the body centre is 10.35 inboard of the axis.
def l1_spots():
    return [(20.0, -LINK_W / 2 + 6), (20.0, LINK_W / 2 - 6)]

def l1_ledge_spots():
    bc = L1 - (P.MG_L / 2.0 - P.MG_SHAFT_OFF)        # body centre
    return [(bc - P.MG_TAB_SPAN / 2 + 1.5, 0.0),
            (bc + P.MG_TAB_SPAN / 2 - 1.5, 0.0)]

def l2_spots():
    return [(LINK2_OUT_START + 8, -LINK_W / 2 + 6),
            (LINK2_OUT_START + 8, LINK_W / 2 - 6), (L2 / 2 + 14, 0.0)]

# head: nose narrow enough for the v1 shade's MEASURED yoke gap (37.3 --
# H_HX was the MG housing's number and simply wrong for the SG head), tail
# wide enough to bolt between link2's plates.
HEAD_HALF = 18.3
HEAD_TAIL_W = LINK2_HALF + LINK2_OUT_HALF - 2 * GAP_FIT
HEAD_NOSE_LEN = 26.0
HEAD_BLOCK_H = 30.0

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
    m += pl.revolve_shell(0.0, TUB_H, od, TUB_WALL, steps=26)
    m += pl.prism(pl.circle(od(0.4), 160), 0.0, FLOOR)          # floor

    # ---- pan servo mount: pocket walls + two tab pillars ----
    # servo body 40.7 x 19.7 centred so the SPLINE (offset 10.0 along the long
    # axis) lands exactly on (0,0): body centre at (-(20.35-10), 0)
    bx = -(P.MG_L / 2.0 - P.MG_SHAFT_OFF)
    body = box(bx - P.MG_L / 2 - 0.4, -P.MG_W / 2 - 0.4,
               bx + P.MG_L / 2 + 0.4, P.MG_W / 2 + 0.4)
    collar = body.buffer(2.4, join_style=2).difference(body)
    m += pl.prism(collar, FLOOR - OVL, 16.0)                    # lateral collar
    for sx in (-1, 1):
        px = bx + sx * (P.MG_TAB_SPAN / 2 - 3.2)
        pil = box(px - 5.0, -P.MG_W / 2 - 0.4, px + 5.0, P.MG_W / 2 + 0.4)
        pil = pil.difference(body)
        m += pl.prism(pil, FLOOR - OVL, PAN_TAB_Z)              # tab ledge
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

    # rim skirt: hides the joint, locates the disc laterally round the rim
    sk = pl.ring2d(pl.circle(SKIRT_ID + 2 * SKIRT_T, 160), pl.circle(SKIRT_ID, 160))
    m += pl.prism(sk, DISC_Z0 - SKIRT_DROP, DISC_Z0 + OVL)
    return [("v2-disc", m, COLORS["v2-disc"])]


def _tower_bolts():
    return [(sx * 28.0, sy * 16.0) for sx in (-1, 1) for sy in (-1, 1)]


# ---------------------------------------------------------------- tower ----
def tower():
    """Shoulder mount. Prints on its back. Bolts to the disc, 4x M3."""
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

    ax_z = z0 + TWR_AXIS_Z               # absolute axis height
    # two cheeks along X (axis runs along X)
    for sx, cheek_drive in ((1, True), (-1, False)):
        x0 = sx * (TWR_GAP / 2)
        x1 = sx * (TWR_GAP / 2 + TWR_CHEEK)
        prof = box(min(x0, x1), -bd / 2 + 2, max(x0, x1), bd / 2 - 2)
        if cheek_drive:
            # bore for the servo output boss, on the axis: an X-axis hole
            # approximated as banded slabs, the same trick v1's yoke uses
            cuts = [(box(min(x0, x1) - OVL, -w, max(x0, x1) + OVL, w), zl, zh)
                    for zl, zh, w in _slices(P.MG_BOSS_D + 1.0, ax_z)]
            m += pl.banded(prof, z0 + bt - OVL, TWR_H + z0, cuts)
        else:
            m += pl.prism(prof, z0 + bt - OVL, TWR_H + z0)
            # threaded stub axle for link1's idler plate, on the axis, -X out
            ax = pl.threaded_bore(P.AXLE_D, P.SCREW_MAJOR, P.SCREW_PITCH,
                                  0.0, P.SCREW_ENGAGE, clearance=P.SCREW_FIT / 2)
            ax.rotate_y(-90.0)
            ax.translate(dx=min(x0, x1) + OVL, dz=ax_z)
            m += ax
    return [("v2-tower", m, COLORS["v2-tower"])]


def _slices(d, cz, seg=24):
    """Horizontal slabs approximating a Y-axis... X-axis bore of diameter d
    centred at height cz -- v1's _disc_slices, reimplemented for banded()."""
    out = []
    r = d / 2.0
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
    for (cx, cy) in tuple(spots) + tuple(holes_at):
        openings.append((affinity.translate(pl.circle(P.M3_CLEAR, 24), cx, cy),
                         -OVL, PLATE_T + OVL))
        keep.append(affinity.translate(pl.rounded_rect(13.0, 12.0, 2.0), cx, cy))
    cuts = _skeleton(length, unary_union(keep))
    if not cuts.is_empty:
        openings.append((cuts, -OVL, PLATE_T + OVL))
    m = pl.banded(prof, 0.0, PLATE_T, openings)
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
        # drive side +X at the shoulder; elbow flips drive to -X
        _link_plate("v2-link1-in", L1, c, near="recess", far="window",
                    spots=l1_spots(), holes_at=l1_ledge_spots()),
        _link_plate("v2-link1-out", L1, c, near="idler", far="boss3",
                    spots=l1_spots()),
        _standoffs("v2-link1-spacers", l1_spots(),
                   2 * LINK1_HALF - 0.4, COLORS["v2-clamp"]),
        _standoffs("v2-link1-ledges", l1_ledge_spots(),
                   P.MG_TAB_Z - P.MG_TAB_T, COLORS["v2-clamp"]),
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
        _link_plate("v2-link2-in", L2, c, near="recess", far="mount",
                    spots=l2_spots()),
        _truncated_plate("v2-link2-out", L2, LINK2_OUT_START, c, far="mount",
                         spots=l2_spots()),
        _standoffs("v2-link2-spacers", l2_spots(),
                   LINK2_HALF + LINK2_OUT_HALF - 0.4, COLORS["v2-clamp"]),
    ]


# ----------------------------------------------------------------- head ----
def head_block():
    """Nose carries the v1 shade on the SG90 axis; tail bolts between
    link2's plates. Nose width 2*HEAD_HALF = 36.6, matched to the shade's
    MEASURED yoke gap of 37.3 -- the first cut used v1's H_HX (24.45) and
    the audit showed the yoke 11.6 mm narrower than the block it had to
    straddle. Measured beats copied."""
    m = pl.Mesh()
    tail_w = HEAD_TAIL_W - 2 * GAP_FIT
    D = 26.0
    # build lying DOWN: block height along Z is HEAD_BLOCK_H, length along Y
    tail = pl.rounded_rect(tail_w, D, 5.0)
    nose = pl.rounded_rect(2 * HEAD_HALF, D, 5.0)
    m += pl.banded(affinity.translate(tail, 0, 0), 0.0, HEAD_BLOCK_H, [
        (unary_union([affinity.translate(pl.circle(P.M3_INSERT_D, 24), sx * (tail_w / 2 - 6), 0)
                      for sx in (-1, 1)]), -OVL, HEAD_BLOCK_H + OVL)])
    sg = box(HEAD_HALF - 3.0 - (P.SG_L + 0.4), -(P.SG_W + 0.4) / 2,
             HEAD_HALF - 3.0, (P.SG_W + 0.4) / 2)
    nose_m = pl.banded(affinity.translate(nose, 0, D - 4.0), 0.0, HEAD_BLOCK_H,
                       [(affinity.translate(sg, 0, D - 4.0), 3.0, HEAD_BLOCK_H + OVL)])
    m += nose_m
    ax = pl.prism(pl.circle(P.AXLE_D, 48), 0.0, P.AXLE_LEN + OVL)
    ax.rotate_y(-90.0)
    ax.translate(dx=-HEAD_HALF + OVL, dy=D - 4.0, dz=HEAD_BLOCK_H / 2.0)
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
    """Servo tab clamp bars: pan servo (2) + tower shoulder servo (2)."""
    m = pl.Mesh()
    for i in range(4):
        b = affinity.translate(pl.rounded_rect(12.0, 8.0, 2.0), i * 16.0, 0)
        h = affinity.translate(pl.circle(P.M3_CLEAR, 24), i * 16.0, 0)
        m += pl.prism(b.difference(h), 0.0, 4.0)
    return [("v2-clamps", m, COLORS["v2-clamp"])]
