# HOTARU — dimensional spec

Animated Pixar-style desk lamp: round base, three MG996R lift joints, SG90
head tilt, cone shade with a WS2812 ring, I2S mic + speaker, ESP32-S3.

All dimensions **mm**. Axes: X right, Y back (away from you), Z up.
Origin: centre of the base footprint, Z=0 at the base's outer bottom face.

Every number lives in [`cad/params.py`](cad/params.py); nothing downstream
hardcodes one. Measure your parts, edit that file, re-run `assembly.py`.
`TODO` in params = from a datasheet or listing, not from calipers.

## Kinematics

| Joint | Servo | Axis | Drives |
|---|---|---|---|
| base | MG996R | X, Z=82.65, Y=18.0 | lower arm pitch (no yaw) |
| shoulder | MG996R | 120.0 from base | upper arm |
| elbow | MG996R | 120.0 from shoulder | forearm |
| head | SG90 | 105.0 from elbow | shade tilt |

Reach 345 mm; 316 mm tall in the neutral pose.

Every joint is **driven one side, bearing-supported the other**. The drive
plate captures the horn in a cross recess — slot walls carry the torque,
screws only retain it axially — and the idler plate rides a stub axle on the
far housing wall. Servos sit in **clamshell cup + screw-on cap**, clamped by
the body with their tabs trapped in slots. The tab *hole* pattern is never a
load path, which matters: the brief's "20 mm c-c" matches no MG996R drawing.

## Base

Round turned form: cylinder Ø160 to Z=42, then a
**quarter-sine shoulder** lofted in to Ø118 at Z=58. The shoulder is
tangent to the cylinder where it leaves it, so they read as one turned piece
rather than a box with a chamfer. It only ever narrows going up, so the whole
silhouette is decorative *and* support-free.

**The height is the point.** 56 mm of interior, 37 mm of it clear above the
board — wire volume, so cable management is not an afterthought this time.

| Feature | Where | Detail |
|---|---|---|
| ESP32-S3 | X 6..70, Y -29..1, underside Z=16.0 | flat, USB-C + components UP, pins DOWN into 13.6 mm |
| USB-C | right, Y ±15 of -14.0, Z 15.5..23.0 | window cut through a **tunnel boss** |
| speaker | left, face X=-69.0, Y=-6.0 Z=22.0 | 7 slots x 2.2; sealed back volume behind X=-56.0 |
| mic | front, X=0.0 Z=30.0 | 3 x 1.6 ports + a wall counterbore the board drops into |
| amp | floor (0.0, 30.0) | four L-ridges |
| lid | — | **structural**: underside rib web (Z 50..58) carries the arm |
| lid rim | — | 4 M3s into the seat lugs + a snap bead on the skirt |
| bulk cap | floor (46.0, 46.0) | C-clamp, press fit |
| MX key | lid (-28.0, -30.0) | see below |
| seat lugs | (±53, 0), (0, ±53) | 4 short lid hold-downs, webbed into the rim |

The shoulder stops tapering at Z=52 and runs **straight to the top** — a lid
skirt dropping into a cone only fits at one height, and the first attempt
fouled the bore by 2.1 mm. The rebate is now a constant Ø113.2, with a snap
groove at Z=55 that the skirt's 0.45 bead clicks into.

**Component retention.** The speaker pocket is open at the top: the driver
drops in onto a shelf, framed on four sides, and `spk-clamp` screws down over
it into two posts that deliberately stop 0.4 mm *below* the pocket top, so
tightening preloads the driver instead of bottoming out. The mic sits in a
U-slot counterbore in the front wall — circular below centre, straight-sided
above, so the board drops in from the top and two full-height lips retain it
against the ports. Neither part relies on tape.

A round wall curves away from a flat board end, so the **USB tunnel boss** is
a flat land filling that gap with the window cut through it. Without it the
receptacle sits too deep for a plug to reach — the single thing a circular
base breaks that a square one does not.

### MX key socket

Plate section **exactly 1.5** (Z 59.9..61.4), cutout 14.1 x 14.1 — donor-keyboard
geometry, so a Cherry MX clips straight in. Below it the lid opens to
16.0 for 1.9 mm: the relief the latches spring into. Push it in
from above; it pops in and stays.

## Base joint

Bolts on top of the lid on an 8 mm plinth, its four M3s threading into
heat-set inserts in the **lid's own rib web**. There are no internal
bulkheads: load goes housing → rib → lid rim → seat ring → shoulder → wall →
floor, so nothing stands in the wiring bay.
Pivot height 82.65 is set by the **yoke's swing radius (18.87)**, not the
housing. The plinth has an open corridor at |X| 25.45..29.45 for the yoke to
sweep through, and the bolts sit outboard at ±35 to clear it — the boss OD has to miss the
corridor, not just its centre.

## Cone

Ø66 mouth tapering to Ø20 over 52 mm — 23.9° half-angle,
1.6 wall, 8 vent slots, apex left OPEN (no bridge, and it is the
ring's wire exit).

The tilt pivot sits at local Z=60, **behind** the cone — where the taper is
down to r=10.0, narrower than the servo housing's 18.65. That is the
only place a yoke can straddle a cone without burying itself in it. A
10 mm collar flares out at 37.6° to carry the yoke plates.

## Printed couplers

A true involute spline at the MG996R's 5.92 OD works out to ~0.37 mm per
tooth flank — under a 0.4 nozzle, so it will not print. The bore is instead a
root circle plus **25 round-bottomed scallops** of Ø0.9 that the metal teeth
seat into: bore reaches r=3.10 against tips at 2.96.

| Part | Use |
|---|---|
| `horn-mg996r` | printed cross horn, **stepped bore**: Ø6.5 head counterbore, Ø3.4 shank, then 4.5 mm of spline. A straight 5.45 bore would let the screw head drop into it and clamp nothing; 4.5 mm of engagement stays under the 4.7 mm of shaft that actually protrudes. |
| `horn-sg90` | printed cross horn for the head tilt |
| `horn-adapter` | captures the STOCK metal horn — zero printed-spline risk |
| `spline-test` | three bores at −0.1 / 0 / +0.1. **Print this first.** |

Torque ends up in the yoke's cross slot either way, so a loose spline costs
backlash, not a stripped joint.

## Print plates

Bambu A1 mini 180³, PLA, 0.2 mm layers, 3 walls, **no supports on any part**.

| Plate | Parts | Used | Why |
|---|---|---|---|
| `plate-1-test.stl` | horn-mg996r, spline-test | 118 x 48 x 6 | print FIRST -- verifies the printed spline before you commit |
| `plate-2-base.stl` | base | 160 x 160 x 58 | the round base; it owns the bed on its own |
| `plate-3-arms.stl` | arm-lower, arm-upper, arm-fore | 123 x 52 x 154 | the three segments, standing on their yokes |
| `plate-4-head.stl` | lid | 118 x 118 x 9 | cone (mouth down) + lid (top face down) |
| `plate-4-head-2.stl` | shade | 66 x 66 x 72 | cone (mouth down) + lid (top face down) |
| `plate-5-small.stl` | horn-adapter, base-joint, horn-sg90, cap-base, cap-shoulder, cap-elbow, cap-head | 167 x 100 x 52 | everything that is quick |

Arms print standing on the yoke: the two plates start as separate islands and
merge into the tube as the print rises. Yoke converge 42.7°, tube→housing
flare 38.8°, cone 23.9°, collar 37.6° — all inside the 45° rule.

## Electrical

- 4 servos on 4 hardware LEDC channels (no PCA9685 at this count)
- I2S0 → INMP441 (RX), I2S1 → MAX98357A (TX); separate controllers
- WS2812 ring on one GPIO via RMT
- **Split the 5 V rail at the USB-C connector**: one leg to the ESP32-S3, one
  to the servo bus, common ground, bulk cap across the servo rail *at the
  branch* — the clamp at (46.0, 46.0) is placed for that, not back at the input.
  Never feed servos from the board's onboard regulator.
