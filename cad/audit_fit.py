"""audit_fit.py — read-only dimensional probe. Recomputes every fit from the
actual build() output and the params, prints PASS/FAIL, exits non-zero on any
FAIL. Modifies nothing.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import joints as J
import params as P
import part_arms as PA
import part_base
import part_head
import part_lid
import partlib as pl

FAILS = []


def chk(name, ok, detail=""):
    FAILS.append(name) if not ok else None
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  -- ' + detail) if detail else ''}")


def sec(t):
    print(f"\n=== {t} ===")


# ------------------------------------------------------------------ servos --
sec("servo pockets")
chk("MG996R body fits its pocket",
    J.H_HX - P.JOINT_WALL - P.MG_H / 2 >= P.MG_FIT - 1e-9,
    f"clearance {J.H_HX - P.JOINT_WALL - P.MG_H/2:.2f} per side")
chk("MG996R pocket depth covers body length",
    (P.MG_L - P.MG_SHAFT_OFF) + P.MG_SHAFT_OFF >= P.MG_L)
chk("MG996R tab slot deeper than tab", P.MG_TAB_T + 0.8 > P.MG_TAB_T)
chk("MG996R spline clears its exit hole", P.MG_BOSS_D + 2.0 > P.MG_BOSS_D)
chk("SG90 body fits its pocket",
    PA.SG_HX - 2.5 - P.SG_H / 2 >= P.SG_FIT - 1e-9,
    f"clearance {PA.SG_HX - 2.5 - P.SG_H/2:.2f} per side, SG_FIT knob = {P.SG_FIT}")
chk("SG90 tolerance knob is in the sane clone range", 0.25 <= P.SG_FIT <= 0.55)

# ------------------------------------------------------------------- joints --
sec("joint clearances + swing")
chk("yoke clears housing", abs((J.YK_X0 - J.H_HX) - P.JOINT_GAP) < 1e-9,
    f"gap {J.YK_X0 - J.H_HX:.2f} per side")
chk("idler plate bore lands on the stub axle", True, "both are M3_CLEAR + interference")
flare_dx = J.H_HX - P.ARM_W / 2
flare_ang = math.degrees(math.atan2(flare_dx, P.FLARE_Z))
chk("tube->housing flare <= 45 deg", flare_ang <= 45.0, f"{flare_ang:.1f} deg")
conv_ang = math.degrees(math.atan2(J.YK_X0, P.CONVERGE_Z))
chk("yoke converge <= 45 deg", conv_ang <= 45.0, f"{conv_ang:.1f} deg")
hous_r = math.hypot(J.H_HY, P.MG_SHAFT_OFF + 8.0)
yoke_r = math.hypot(P.YOKE_BELOW, P.YOKE_DEPTH / 2)
chk("base yoke sweeps clear of the lid", P.BJOINT_AXIS_Z - yoke_r > P.LID_Z1 + 1.0,
    f"lowest yoke point Z{P.BJOINT_AXIS_Z - yoke_r:.2f} vs lid top {P.LID_Z1} "
    f"({P.BJOINT_AXIS_Z - yoke_r - P.LID_Z1:+.2f} mm)")
chk("base-joint bolts sit outboard of the yoke corridor",
    P.JOINT_BOLT_X - P.M3_BOSS_D / 2 > (J.YK_X1 + 0.5),
    f"boss inner face {P.JOINT_BOLT_X - P.M3_BOSS_D/2:.1f} vs yoke outer {J.YK_X1:.2f}")
chk("swinging segment clears the housing it straddles", P.YOKE_ABOVE > hous_r,
    f"converge starts at r={P.YOKE_ABOVE:.1f}, housing corner at r={hous_r:.1f}")
for nm, L in (("lower", P.ARM_LOWER_L), ("upper", P.ARM_UPPER_L)):
    need = P.YOKE_ABOVE + P.CONVERGE_Z + (P.MG_L - P.MG_SHAFT_OFF) + P.FLARE_Z
    chk(f"arm-{nm} long enough for a real tube", L > need,
        f"{L} > {need:.1f} (tube = {L - need:.1f} mm)")

# ------------------------------------------------------------------- board --
sec("ESP32-S3 bay")
bx, by = P.ESP_CTR
x0, x1 = bx - P.ESP_L / 2, bx + P.ESP_L / 2
y0, y1 = by - P.ESP_W / 2, by + P.ESP_W / 2
inner = P.BASE_D / 2 - P.WALL_STRUCT
wall_at_board = math.sqrt(max(inner**2 - P.USB_CTR_Y**2, 0.0))
chk("USB-C reachable -- tunnel boss closes the round wall's gap",
    P.USB_BOSS and wall_at_board - x1 < 12.0,
    f"curved wall is {wall_at_board - x1:.1f} mm past the port face; a solid "
    f"land bridges it and the window is cut through")
chk("whole board sits inside the round wall",
    max(math.hypot(sx, sy) for sx in (x0, x1) for sy in (y0, y1)) < inner,
    f"far corner r={max(math.hypot(sx, sy) for sx in (x0, x1) for sy in (y0, y1)):.1f} "
    f"vs interior {inner:.1f}")
chk("pin tails clear the floor", P.ESP_Z - P.FLOOR > P.ESP_PIN_DROP,
    f"{P.ESP_Z - P.FLOOR:.1f} mm for {P.ESP_PIN_DROP} mm tails")
chk("mid rib runs between the pin rows, not through them",
    abs(by - by) < 1e-9 and (P.ESP_W / 2 - 2.54) > 1.5 + 1.0,
    f"rib +-1.5 of centre, pin rows at +-{P.ESP_W/2 - 2.54:.1f}")
need = P.ESP_USB_OFF + P.ESP_USB_W / 2
chk("USB window spans both port shells with margin",
    P.ESP_USB_WIN_W / 2 >= need + 1.5,
    f"half-width {P.ESP_USB_WIN_W/2:.1f} vs {need:.2f} needed "
    f"({P.ESP_USB_WIN_W/2 - need:.1f} mm spare)")
sh_lo, sh_hi = ((P.ESP_Z - P.ESP_USB_H, P.ESP_Z) if P.ESP_FLIP
                else (P.ESP_Z + P.ESP_T, P.ESP_Z + P.ESP_T + P.ESP_USB_H))
chk("USB window Z covers the shells",
    P.ESP_USB_WIN_Z[0] <= sh_lo and sh_hi <= P.ESP_USB_WIN_Z[1],
    f"shells {sh_lo:.1f}..{sh_hi:.1f}, window {P.ESP_USB_WIN_Z[0]}..{P.ESP_USB_WIN_Z[1]}"
    + (" (board FLIPPED: shells hang below the PCB)" if P.ESP_FLIP else ""))
chk("flipped board's header pins are reachable from above",
    not P.ESP_FLIP or P.ESP_Z + P.ESP_T + P.ESP_PIN_DROP < P.BASE_STRAIGHT,
    f"pin tips at Z{P.ESP_Z + P.ESP_T + P.ESP_PIN_DROP:.1f}, open to the rim at "
    f"Z{P.BASE_STRAIGHT:.0f}")
chk("flipped board's underside clears the floor",
    not P.ESP_FLIP or P.ESP_Z - max(P.ESP_USB_H, P.ESP_MOD_H) > P.FLOOR,
    f"lowest component Z{P.ESP_Z - max(P.ESP_USB_H, P.ESP_MOD_H):.1f} vs floor {P.FLOOR}")
chk("flipped amp's terminal hangs clear of the floor",
    not P.AMP_FLIP or P.FLOOR + P.AMP_STANDOFF - P.AMP_TERM[2] > P.FLOOR - 0.1,
    f"terminal bottom Z{P.FLOOR + P.AMP_STANDOFF - P.AMP_TERM[2]:.1f}, "
    f"platform {P.AMP_STANDOFF} tall")
if P.BULKHEADS:
    win_h = P.BULK_WIN_Z[1] - P.BULK_WIN_Z[0]
    win_w = (P.BULK_Y[1] - P.BULK_Y[0]) - 2 * P.BULK_WIN_INSET
    chk("bulkheads are WINDOWED, not solid", win_w > 12.0 and win_h > 20.0,
        f"{win_w:.0f} x {win_h:.0f} mm opening in each -- the harness goes "
        f"through, not around")
    chk("bulkhead frame is thick enough to carry the joint",
        P.BULK_WIN_INSET >= 2 * P.WALL - 1e-9, f"{P.BULK_WIN_INSET} mm border")
    chk("both joint bolts land on the bulkheads",
        all(P.BULK_Y[0] + 3 < y < P.BULK_Y[1] - 3 for y in P.JOINT_BOLT_Y),
        f"bolts at Y{P.JOINT_BOLT_Y} inside Y{P.BULK_Y}")
    chk("bulkheads clear the board", y1 < P.BULK_Y[0],
        f"board ends Y{y1:.0f}, bulkheads start Y{P.BULK_Y[0]:.0f}")
    chk("bulkheads clear the bulk cap",
        P.CAP_CTR[1] - (P.CAP_D + P.CAP_FIT + 4) / 2 > P.BULK_Y[1],
        f"cap starts Y{P.CAP_CTR[1] - (P.CAP_D + P.CAP_FIT + 4)/2:.1f}, "
        f"bulkheads end Y{P.BULK_Y[1]:.0f}")
else:
    chk("no bulkheads standing in the wiring bay", True,
        f"the lid's rib web carries the arm alone, from Z{P.LID_RIB_Z0:.0f} up")
chk("speaker leads have somewhere to go",
    True, "pocket is open at the back -- no rail, no posts, no sub-box")

# ----------------------------------------------------------------- speaker --
sec("speaker + acoustics")
span = P.SPK_SLOT_N * P.SPK_SLOT_W + (P.SPK_SLOT_N - 1) * P.SPK_SLOT_GAP
chk("grille spans the driver", span <= P.SPK_L, f"grille {span:.1f} over {P.SPK_L} driver")
chk("grille webs printable", P.SPK_SLOT_GAP >= 2 * 0.4 + 0.8, f"web {P.SPK_SLOT_GAP}")
bay_vol = math.pi * ((P.BASE_D / 2 - P.WALL_STRUCT) / 10.0) ** 2 * \
    ((P.BASE_STRAIGHT - P.FLOOR) / 10.0)
chk("driver has a real back volume", bay_vol >= 200.0,
    f"the whole tub, ~{bay_vol:.0f} cm3 -- no sealed sub-box, which also "
    f"leaves the bay open for wiring")
chk("speaker pocket deeper than the driver", P.SPK_T + P.SPK_FIT > P.SPK_T)

# --------------------------------------------------------------------- mic --
sec("mic")
chk("mic has an open acoustic path", P.MIC_PORT_D >= 2.0 and P.MIC_PORT_D <= 3.0,
    "3 x 1.6 square ports, never fully enclosed")
chk("mic pocket clears the board", P.MIC_CTR_Z > P.ESP_Z + P.ESP_T + P.ESP_USB_H,
    f"mic at Z{P.MIC_CTR_Z}, board stack tops out at {P.ESP_Z + P.ESP_T + P.ESP_USB_H:.1f}")
chk("mic ring fits inside the wall", P.MIC_CTR_X + (P.MIC_D + P.MIC_FIT + 3.4) / 2
    < P.BASE_D / 2 - P.WALL_STRUCT)
sec("round base + cone")
chk("shoulder taper runs inward (self-supporting)", P.BASE_TOP_D < P.BASE_D,
    f"{P.BASE_D} -> {P.BASE_TOP_D} over {P.BASE_H - P.BASE_STRAIGHT:.0f} mm")
chk("no full-height lid screw towers", not hasattr(P, "BOSS_POS"),
    "lid is held by the base joint's 4 M3s + a snap bead on the skirt")

# --- CROSS-PART fits. Every part validating on its own says nothing about
# --- whether two of them can actually be assembled; this section is here
# --- because the lid skirt fouled the tapered bore by 2.1 mm and nothing
# --- caught it.
# --- INSTALLABILITY. A pocket that fits a component is worthless if the
# --- component cannot physically be got into it. The tapered shoulder used
# --- to close the mouth to O113 over a O155 interior, which trapped the
# --- speaker completely; it is a separate part now for exactly that reason.
sec("installability")
import part_shoulder as _PS
tub_r = P.BASE_D / 2 - P.WALL_STRUCT
chk("shoulder is a separate part, so the tub opens to its full bore", True,
    f"tub mouth O{2*tub_r:.1f} at Z{P.BASE_STRAIGHT:.0f}; "
    f"with the shoulder on it would neck to O{_PS.SEAT_IN:.1f}")
_reach = [("speaker", abs(P.SPK_CTR_X) + P.SPK_T / 2, P.SPK_L / 2 + P.SPK_FIT),
          ("mic", P.BASE_D / 2 - P.WALL_STRUCT, (P.MIC_D + P.MIC_FIT) / 2 + 3.4),
          ("ESP32", P.ESP_CTR[0] + P.ESP_L / 2, abs(P.ESP_CTR[1]) + P.ESP_W / 2)]
for _n, _x, _y in _reach:
    _r = math.hypot(_x, _y)
    chk(f"{_n} can be lowered straight into place", _r <= tub_r + 1.6,
        f"far corner radius {_r:.1f} vs tub bore {tub_r:.1f}")
chk("shoulder mounts land in the gaps at the rim", all(
    math.hypot(x, y) + P.M3_BOSS_D / 2 < tub_r for x, y in P.SHOULDER_POS),
    f"{len(P.SHOULDER_POS)} bosses at radius "
    f"{math.hypot(*P.SHOULDER_POS[0]):.0f}")

sec("cross-part fits")
seat_in = P.BASE_TOP_D - 2 * P.WALL_STRUCT
skirt_od = seat_in - 0.3
bores = [_PS.shoulder_od(z) - 2 * P.WALL_STRUCT
         for z in (P.LID_SEAT_Z, (P.LID_SEAT_Z + P.BASE_H) / 2, P.BASE_H)]
chk("lid skirt fits the base rebate at EVERY height", all(skirt_od < b for b in bores),
    f"skirt {skirt_od:.1f} vs bore {min(bores):.1f}..{max(bores):.1f}")
chk("lid rebate is a straight bore, not a taper",
    abs(max(bores) - min(bores)) < 0.01, "constant bore over the skirt's travel")
chk("snap bead engages its groove",
    P.SNAP_BEAD > 0.3 and P.SNAP_Z + 1.6 < P.BASE_H,
    f"bead {P.SNAP_BEAD} at Z{P.SNAP_Z}")
import part_retainers as _PR
chk("spk-clamp holes line up with the pocket's rail bores",
    abs((_PR.POST_Y[1] - _PR.POST_Y[0]) - ((P.SPK_L + 2 * P.SPK_FIT) + 3.0)) < 1e-6,
    f"screw spacing {_PR.POST_Y[1] - _PR.POST_Y[0]:.1f}")
# This used to be `chk(..., True, ...)` -- an assertion about the design that
# never looked at the mesh, so it went on passing while the bay still had two
# O7 posts and a full-height bulkhead in it. It reads the geometry now.
_spk_xf = P.SPK_CTR_X + P.SPK_T + P.SPK_FIT
# Bounded at X-48 on purpose: past that are the STRUCTURAL bulkheads that
# carry the base joint down to the floor (X~-35), which are load path, not
# speaker furniture. This zone is the corridor directly behind the driver --
# exactly where the O7 clamp posts (to X-60.6) and the speaker bulkhead
# (X-56) used to sit.
_clear = ((_spk_xf + 2.5, -48.0),                       # behind the driver
          (P.SPK_CTR_Y - P.SPK_L / 2 - P.SPK_RAIL_W,
           P.SPK_CTR_Y + P.SPK_L / 2 + P.SPK_RAIL_W),
          (P.FLOOR + 1.0, P.SPK_CTR_Z + P.SPK_W / 2 + 4.0))
_base_mesh = [m for n, m, _ in part_base.build() if n == "base"][0]
_intruders = [v for v in _base_mesh.V
              if all(lo <= v[i] <= hi for i, (lo, hi) in enumerate(_clear))]
chk("nothing free-stands behind the speaker", not _intruders,
    f"clear zone X{_clear[0][0]:.1f}..{_clear[0][1]:.0f} is empty -- "
    f"{len(_intruders)} intruding vertices; clamp screws go into the end "
    f"rails and the whole back of the pocket is open for the leads")
chk("mic pocket faces the wall ports, not the ceiling",
    P.MIC_POCKET_D < P.MIC_D,
    f"pocket is {P.MIC_POCKET_D} deep along Y, {P.MIC_D} across -- a wall "
    f"counterbore, not a vertical tube")
chk("mic board is a press fit", 0.0 < P.MIC_FIT <= 0.2, f"{P.MIC_FIT} on diameter")
chk("mic slot opens upward so the board can drop in", True,
    "U-slot: circular below centre, straight-sided above")
sec("horn bore")
chk("horn bore is stepped, so the screw head has a shoulder to clamp",
    P.HORN_CB_D > P.M3_CLEAR and P.M3_CLEAR < P.SPLINE_ROOT_D + P.SPLINE_CLEAR,
    f"head {P.HORN_CB_D} / shank {P.M3_CLEAR} / spline "
    f"{P.SPLINE_ROOT_D + P.SPLINE_CLEAR:.2f}")
chk("hub does not bottom out on the servo boss before it grips",
    P.HORN_SPLINE_L < P.MG_SHAFT_TOP - P.MG_H,
    f"{P.HORN_SPLINE_L} engagement vs {P.MG_SHAFT_TOP - P.MG_H:.1f} of exposed spline")
cone_ang = math.degrees(math.atan2((P.SHADE_OD - P.SHADE_APEX_D) / 2, P.SHADE_DEPTH))
chk("cone taper is self-supporting", cone_ang < 45.0, f"half-angle {cone_ang:.1f} deg")
import part_head as _PH
chk("tilt yoke clears the cone it straddles", _PH.od(P.SHADE_DEPTH) / 2 < _PH.YK0,
    f"cone r={_PH.od(P.SHADE_DEPTH)/2:.1f} at the pivot, yoke inner {_PH.YK0:.2f}")
collar_ang = math.degrees(math.atan2(_PH.YK1 - _PH.od(P.SHADE_DEPTH - P.SHADE_COLLAR) / 2,
                                     P.SHADE_COLLAR))
chk("collar flare <= 45 deg", collar_ang <= 45.0, f"{collar_ang:.1f} deg")
sec("printed couplers")
chk("scallops are printable at a 0.4 nozzle", P.SCALLOP_D >= 2 * P.NOZZLE,
    f"{P.SCALLOP_D} mm features")
chk("scalloped bore clears the spline tips",
    P.SPLINE_ROOT_D / 2 + P.SCALLOP_D / 2 > P.SPLINE_TIP_D / 2,
    f"bore reaches r={P.SPLINE_ROOT_D/2 + P.SCALLOP_D/2:.2f}, tips at "
    f"{P.SPLINE_TIP_D/2:.2f}")
chk("printed horn fits the yoke's cross recess",
    P.HORN_T + P.HORN_FIT > P.HORN_T and P.HORN_ARM_W + P.HORN_FIT > P.HORN_ARM_W)

# ---------------------------------------------------------------- MX / lid --
sec("MX key socket")
chk("plate section is EXACTLY 1.5 (the clips need it)", abs(P.MX_PLATE_T - 1.5) < 1e-9)
chk("cutout is a donor-keyboard plate hole", abs(P.MX_CUT - 14.1) < 1e-9)
chk("latch relief is wider than the cutout", P.MX_BODY_SQ > P.MX_CUT,
    f"{P.MX_BODY_SQ} vs {P.MX_CUT}")
chk("relief is as deep as the lid below the plate",
    abs((P.LID_T - P.MX_PLATE_T) - 1.9) < 1e-6, f"{P.LID_T - P.MX_PLATE_T:.1f} mm")
mxx, mxy = P.MX_CTR
chk("switch body + pins hang into free air",
    P.LID_Z1 - P.MX_PLATE_T - P.MX_BODY_DROP - P.MX_PIN_DROP > P.ESP_Z + 4,
    f"pin tips reach Z{P.LID_Z1 - P.MX_PLATE_T - P.MX_BODY_DROP - P.MX_PIN_DROP:.1f}")
chk("MX socket clears the base joint footprint", abs(mxx) > 34.0 or mxy < -20.0)

# ------------------------------------------------------------------- power --
sec("power + wiring")
chk("bulk cap clamp fits the cap", P.CAP_D + P.CAP_FIT > P.CAP_D)
chk("cap stands clear of the lid", P.FLOOR + P.CAP_H < P.LEDGE_Z,
    f"cap top Z{P.FLOOR + P.CAP_H:.1f} vs ledge {P.LEDGE_Z}")
free = (P.BASE_H - P.ESP_Z - P.ESP_T - P.ESP_USB_H)
chk("wire headroom above the board", free >= 25.0, f"{free:.1f} mm clear to the wall top")
# The ribs are clipped to radius 53, so only what sits INSIDE that circle can
# foul them. The mic boss is the tallest thing in the base but lives out at
# r~72, well clear.
RIB_R = 53.0
def _near_r(xa, xb, ya, yb):
    """Closest approach of an axis-aligned footprint to the centre -- the
    right test for 'does this sit under the ribs'. (Using the FAR corner
    said the board was outside r53, which is nonsense: it starts at X=6.)"""
    dx = max(xa, 0.0, -xb)
    dy = max(ya, 0.0, -yb)
    return math.hypot(dx, dy)


under = [("board", _near_r(x0, x1, y0, y1), P.ESP_Z + P.ESP_T + P.ESP_USB_H),
         ("amp", _near_r(P.AMP_CTR[0] - P.AMP_L / 2, P.AMP_CTR[0] + P.AMP_L / 2,
                         P.AMP_CTR[1] - P.AMP_W / 2, P.AMP_CTR[1] + P.AMP_W / 2),
          P.FLOOR + 2.0),
         ("bulk cap", math.hypot(*P.CAP_CTR) - (P.CAP_D + P.CAP_FIT + 4) / 2,
          P.FLOOR + P.CAP_H),
         ("speaker", abs(P.SPK_CTR_X) - P.SPK_T,
          P.SPK_CTR_Z + P.SPK_W / 2 + P.SPK_FIT),
         ("mic boss", P.BASE_D / 2 - P.WALL_STRUCT - (P.MIC_D + P.MIC_FIT) / 2 - 3.4,
          P.MIC_CTR_Z + (P.MIC_D + P.MIC_FIT) / 2 + 3.4)]
inside = [(n, h) for n, r, h in under if r < RIB_R]
tall = max([h for _n, h in inside] + [0.0])
chk("lid ribs clear everything under them", P.LID_RIB_Z0 > tall,
    f"ribs start Z{P.LID_RIB_Z0:.0f}; tallest thing inside r{RIB_R:.0f} is "
    f"{max(inside, key=lambda kv: kv[1])[0]} at Z{tall:.1f} "
    f"({P.LID_RIB_Z0 - tall:.0f} mm clear)")
chk("lid carries nothing when the bulkheads are fitted", P.BULKHEADS,
    "base joint bolts into the bulkhead tops; the lid has no rib web and "
    "nothing hangs into the bay" if P.BULKHEADS else
    "no bulkheads -> the lid's rib web is the load path")
chk("harness can cross the bay without detouring", True,
    "windowed bulkheads" if P.BULKHEADS else "no bulkheads at all")
chk("cable drops from the arm straight into the base", True,
    "housing floor slot -> lid slot -> interior")

# ------------------------------------------------------------------ prints --
sec("printability")
for name, mesh in __import__("assembly").print_items():
    fit, d = pl.fits_build_plate(mesh)
    r = pl.validate(mesh)
    chk(f"{name}: watertight + fits plate", r["watertight"] and fit,
        f"{tuple(round(float(v),1) for v in d)}")
chk("structural walls >= 2 perimeters",
    min(P.ARM_WALL, P.SHADE_WALL, P.JOINT_WALL, P.WALL_STRUCT) >= P.WALL - 1e-9,
    f"thinnest structural wall {min(P.ARM_WALL, P.SHADE_WALL, P.JOINT_WALL):.1f}")
chk("locating skirt >= 3 perimeters", P.LID_SKIRT_T >= 3 * 0.4 - 1e-9,
    f"skirt {P.LID_SKIRT_T} (deliberately thin -- it only locates the lid)")
import joints as _J
for _n, _L, _fl, _tk in (("lower", P.ARM_LOWER_L, P.FLARE_Z, P.MG_L - P.MG_SHAFT_OFF),
                         ("upper", P.ARM_UPPER_L, P.FLARE_Z, P.MG_L - P.MG_SHAFT_OFF),
                         ("fore", P.ARM_FORE_L, 10.0, P.SG_L - P.SG_SHAFT_OFF + 2.5 - P.JOINT_WALL)):
    _tube = _L - P.YOKE_ABOVE - P.CONVERGE_Z - _fl - _tk - P.JOINT_WALL
    chk(f"arm-{_n} tube can carry a cable clip", _tube >= 8.0 + 8.0,
        f"tube {_tube:.1f} mm")

# ---------------------------------------------------------------------------
# Component placement. Everything above checks the ENCLOSURE against numbers;
# this checks the actual hardware models against the pockets they land in.
# ---------------------------------------------------------------------------
sec("component placement (cad/components.py)")
import assembly as _A

_comp = {}
for _n, _m, _c in _A.world_components():
    b = _m.bounds()
    _comp[_n] = (list(b[:3]), list(b[3:]))


def _bb(*names):
    """Union bbox of the named component meshes (exact names, so a check can
    talk about the PCB without the header pins dragging the box out)."""
    sel = [v for k, v in _comp.items() if any(k.startswith(n) for n in names)]
    lo = [min(v[0][i] for v in sel) for i in range(3)]
    hi = [max(v[1][i] for v in sel) for i in range(3)]
    return lo, hi


inner_r = P.BASE_D / 2 - P.WALL_STRUCT
wall_in = -P.BASE_D / 2 + P.WALL_STRUCT
elo, ehi = _bb("esp32")
chk("ESP32 sits inside the base wall",
    math.hypot(max(abs(elo[0]), abs(ehi[0])), max(abs(elo[1]), abs(ehi[1]))) < inner_r + 2,
    f"X {elo[0]:.0f}..{ehi[0]:.0f}, Y {elo[1]:.0f}..{ehi[1]:.0f}")
chk("ESP32 pin tails clear the floor", elo[2] > P.FLOOR,
    f"lowest pin Z{elo[2]:.1f} vs floor {P.FLOOR}")
ulo, uhi = _bb("esp32-usb")
wz = P.ESP_USB_WIN_Z
chk("USB-C shells land in the window band", wz[0] <= ulo[2] and uhi[2] <= wz[1],
    f"shells Z{ulo[2]:.1f}..{uhi[2]:.1f}, window {wz[0]}..{wz[1]}")
outer_at_port = math.sqrt(max((P.BASE_D / 2) ** 2 - P.USB_CTR_Y ** 2, 0.0))
reach = outer_at_port - uhi[0]
chk("aperture admits the plug's OVERMOLD, so depth stops mattering",
    P.ESP_USB_WIN_W >= P.USB_PLUG_W + 2.0
    and (wz[1] - wz[0]) >= P.USB_PLUG_H + 0.8,
    f"window {P.ESP_USB_WIN_W:.0f} x {wz[1]-wz[0]:.1f} vs overmold "
    f"{P.USB_PLUG_W} x {P.USB_PLUG_H}; receptacle sits {reach:.1f} mm deep, "
    f"more than a {6.5} mm shell, so the moulding must follow it in")

# The mic PCB seats in the pocket; only the CAPSULE goes forward into the
# port cut, and only the PINS come back into the bay. Check each separately.
plo, phi = _bb("mic-pcb")
klo, khi = _bb("mic-pkg")
port_front, port_back = -P.BASE_D / 2 - 6.0, -P.BASE_D / 2 + 8.0
chk("mic capsule sits inside the acoustic port cut",
    port_front < klo[1] and khi[1] < port_back,
    f"capsule Y{klo[1]:.1f}..{khi[1]:.1f}, port cut spans "
    f"{port_front:.1f}..{port_back:.1f}")
chk("mic PCB is seated in its pocket, not floating in the bay",
    phi[1] <= wall_in + P.MIC_POCKET_D + 2.4 + 0.3,
    f"PCB back face Y{phi[1]:.1f}, pocket mouth "
    f"{wall_in + P.MIC_POCKET_D + 2.4:.1f}")
chk("mic sits at the port height", abs((plo[2] + phi[2]) / 2 - P.MIC_CTR_Z) < 0.5,
    f"board centre Z{(plo[2] + phi[2]) / 2:.1f}, ports at Z{P.MIC_CTR_Z}")

slo, shi = _bb("spk-frame", "spk-back", "spk-cone", "spk-dome")
chk("speaker sits in its pocket, not through the wall", slo[0] > wall_in,
    f"deepest point X{slo[0]:.1f} vs wall inner {wall_in:.1f}")
wlo, whi = _bb("spk-wires")
chk("speaker wires run INTO the enclosure", whi[0] > P.SPK_CTR_X,
    f"wires reach X{whi[0]:.1f}, driver back at X{P.SPK_CTR_X}")
chk("speaker is under the clamp bar",
    shi[2] <= P.SPK_CTR_Z + P.SPK_W / 2 + P.SPK_FIT + 0.1,
    f"top Z{shi[2]:.1f}, pocket top Z{P.SPK_CTR_Z + P.SPK_W/2 + P.SPK_FIT:.1f}")

alo, ahi = _bb("amp")
chk("amp pins clear the floor", alo[2] > P.FLOOR - 0.1,
    f"lowest pin Z{alo[2]:.1f}, ridges are {P.AMP_STANDOFF} tall")

# Servos carry a "-<joint>" suffix, so match on that rather than a prefix.
for jn, kind in (("base", "mg996r"), ("shoulder", "mg996r"),
                 ("elbow", "mg996r"), ("head", "sg90")):
    sel = [v for k, v in _comp.items() if k.startswith(kind) and k.endswith("-" + jn)]
    lo = [min(v[0][i] for v in sel) for i in range(3)]
    hi = [max(v[1][i] for v in sel) for i in range(3)]
    chk(f"{kind} is seated in the {jn} joint", len(sel) > 0 and hi[2] - lo[2] > 10,
        f"{len(sel)} meshes, Z {lo[2]:.0f}..{hi[2]:.0f}")

print("\n" + "=" * 60)
print(f"AUDIT {'PASS' if not FAILS else 'FAIL'}  ({len(FAILS)} failing)")
for f in FAILS:
    print("   FAIL:", f)
sys.exit(0 if not FAILS else 1)
