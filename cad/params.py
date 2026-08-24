"""params.py — every AIBO dimension, in one place.

Units mm. World frame: X right, Y back (away from you), Z up.
Origin: center of the base footprint, Z=0 at the base's outer bottom face.

TODO markers flag numbers taken from datasheets/listings rather than from
calipers on the actual part. Measure, edit here, re-run assembly.py --
nothing downstream hardcodes a dimension.
"""

# ============================================================ print rules ==
NOZZLE      = 0.4
WALL        = 1.6      # absolute minimum wall (2 perimeters)
WALL_STRUCT = 2.4      # load-bearing walls (6 perimeters-ish)
FLOOR       = 2.4
OVL         = 0.2      # shell-fusing overlap (see partlib)
FIT         = 0.3      # sliding clearance, printed-to-printed
FIT_TIGHT   = 0.15     # press/located features
PLATE       = 180.0    # Bambu A1 mini build volume (cube)

# M3 heat-set inserts (arms, joints, lid)
M3_INSERT_D   = 4.0    # bore for a standard M3 short heat-set insert
M3_INSERT_L   = 6.0
M3_CLEAR      = 3.4    # M3 clearance hole
M3_HEAD_D     = 6.2    # socket-cap head
M3_BOSS_D     = 7.4
# M2 self-tappers (SG90, horns, LED ring)
M2_PILOT      = 1.7    # self-tap pilot in PLA
M2_CLEAR      = 2.4

# ====================================================== MG996R (x3) =======
# base / shoulder / elbow. Body numbers from the brief; hole pattern from the
# MG996R standard drawing -- the brief's "20 mm c-c" does not match any
# MG996R I know of, and the clamshell CLAMPS THE BODY rather than bolting
# through the tabs, so a wrong tab pattern cannot break the fit. The tab
# pilots below are a bonus, not the load path.
MG_L          = 40.7   # body length  (shaft axis is perpendicular to this)
MG_W          = 19.7   # body width   (the clamshell split direction)
MG_H          = 42.9   # body bottom -> top of case
MG_TAB_SPAN   = 53.6   # tip-to-tip across the two mounting tabs, along L
MG_TAB_T      = 2.5    # TODO verify -- tab thickness
MG_TAB_Z      = 26.6   # body bottom -> tab underside (brief "26.6 main body")
MG_SHAFT_TOP  = 47.6   # body bottom -> top of output spline (brief "47.6")
MG_SHAFT_D    = 5.9    # 25T spline OD
MG_SHAFT_OFF  = 10.0   # TODO verify -- shaft axis from the near end of MG_L
MG_BOSS_D     = 13.0   # TODO verify -- raised boss around the spline
MG_BOSS_H     = 4.0    # TODO verify
MG_HOLE_DX    = 49.5   # tab holes along L   (MG996R standard)
MG_HOLE_DY    = 10.0   # tab holes across W  (MG996R standard)
MG_FIT        = 0.4    # pocket clearance around the body (per side /2)

# MG996R output horn. The yoke captures the horn in a CROSS RECESS -- torque
# goes through the slot walls, screws only retain it axially. That makes the
# horn's screw-hole pattern irrelevant, which is good, because clone horns
# vary wildly.
HORN_ARM_HALF = 24.0   # TODO verify -- hub center to arm tip
HORN_ARM_W    = 6.5    # TODO verify -- arm width at the hub
HORN_T        = 2.8    # TODO verify -- arm thickness
HORN_HUB_D    = 13.0   # TODO verify -- hub outer diameter
HORN_HUB_T    = 7.3    # = HORN_T + HORN_SPLINE_L
HORN_FIT      = 0.3    # recess clearance

# ========================================================= SG90 (head) ====
# Clone variance is the whole problem here -- SG90_FIT is deliberately its
# own knob. Print head-servo-fit-test.stl first, then set it.
SG_L          = 23.0
SG_W          = 12.2
SG_H          = 29.5   # 29.0-29.5 across batches -- taking the larger
SG_TAB_SPAN   = 32.5
SG_TAB_T      = 2.5    # TODO verify
SG_TAB_Z      = 15.9   # TODO verify -- body bottom -> tab underside
SG_SHAFT_D    = 4.8
SG_SHAFT_OFF  = 5.9    # TODO verify -- shaft axis from the near end of SG_L
SG_HOLE_DX    = 27.8   # TODO verify -- SG90 standard tab hole spacing
SG_HOLE_D     = 2.2    # tab hole (M2 self-tapper)
SG_FIT        = 0.4    # <-- BATCH TOLERANCE KNOB: 0.3 tight ... 0.5 loose
SG_HORN_ARM   = 15.5   # TODO verify -- single-arm horn, hub center to tip
SG_HORN_W     = 4.2    # TODO verify
SG_HORN_T     = 1.8    # TODO verify
SG_HORN_HUB_D = 7.0    # TODO verify

# ================================================ ESP32-S3 N16R8 ROBODUINO =
# Dual Type-C, components + both USB-C on the TOP face, factory male headers
# soldered pointing DOWN along both long edges. Envelope measured on this
# exact board for orchestrator-pad.
ESP_L         = 64.0   # long axis (runs along X here)
ESP_W         = 30.0   # short axis
ESP_T         = 1.6    # PCB
ESP_PIN_DROP  = 8.5    # soldered header pin tails below the PCB
ESP_USB_W     = 8.94   # one USB-C shell, width
ESP_USB_H     = 3.30   # shell height above the PCB top face
ESP_USB_OFF   = 8.53   # TODO verify -- port center offset from board center
ESP_FIT       = 0.4
ESP_USB_WIN_W = 30.0   # right-wall window; must clear BOTH shells with margin
# On a round base the receptacle ends up ~7.6 mm behind the outer surface --
# deeper than a USB-C plug's 6.5 mm metal shell. That is fine ONLY if the
# aperture is bigger than the plug's OVERMOLD, so the moulding can enter the
# window with the shell. Size the window to the overmold, not to the shell.
ESP_USB_WIN_Z = (7.0, 16.5)   # flipped board -> shells hang BELOW the PCB
USB_PLUG_W    = 13.0   # typical USB-C plug overmold
USB_PLUG_H    = 8.5

# ============================================ INMP441 + MAX98357A combo ====
# The INMP441 on this combo is the ROUND board; the amp is the purple
# rectangle. Both TODO -- calipers on arrival.
MIC_D         = 15.0   # TODO verify -- round INMP441 board diameter
MIC_T         = 1.6
MIC_PORT_D    = 2.5    # acoustic port through the wall (2-3 mm per brief)
MIC_FIT       = 0.10   # PRESS fit -- the board is pushed into the pocket
MIC_POCKET_D  = 1.9    # pocket depth (board 1.6 + 0.3)
MIC_POST_D    = 6.0    # M2 post for the retainer tab
AMP_L         = 20.0   # TODO verify -- MAX98357A breakout
AMP_W         = 18.0   # TODO verify
AMP_T         = 1.6
AMP_FLIP      = True   # pins UP too; the screw terminal hangs into the frame
AMP_STANDOFF  = 12.0   # platform height -- must clear the 10 mm terminal

# ================================================== speaker (2040, 4R 2W) ==
SPK_L         = 40.0   # TODO verify
SPK_W         = 20.0   # TODO verify
SPK_T         = 5.5    # TODO verify -- overall depth incl. magnet
SPK_FIT       = 0.4
SPK_RAIL_W    = 3.0    # end rail either side of the pocket -- carries the
                       # clamp screw. NOT a post: it is the frame's own end
                       # wall, in the wall plane, nothing behind the driver.
SPK_CLAMP_T   = 3.0    # clamp bar thickness
SPK_CLAMP_W   = 10.0   # bar width in X -- overhangs the driver back
SPK_TONGUE_W  = 8.0    # each retaining tongue, along the driver
SPK_TONGUE_H  = 16.0   # how far a tongue reaches down behind the driver
SPK_TONGUE_Y  = 13.0   # tongue centres, +-, from the driver centre. Inboard
                       # of the ends and clear of the middle, so the leads
                       # have the whole centre of the pocket to come out of.
SPK_SCREW_DEPTH = 6.0  # how far the clamp screw sinks into the rail
SPK_SLOT_W    = 2.2    # grille slot width
SPK_SLOT_N    = 7      # slots
SPK_SLOT_GAP  = 2.6    # web between slots

# ================================================ WS2812 16-LED ring =======
RING_OD       = 45.0   # TODO verify -- Robocraze 16-bit 5050 ring
RING_ID       = 31.7   # TODO verify
RING_T        = 2.2    # TODO verify -- PCB + LED height
RING_FIT      = 0.4

# ===================================================== Cherry MX (blue) ====
# Donor-keyboard plate geometry: a 14.1 hole in a 1.5 plate, switch clips
# latch underneath. "Pop-open" = the plate section is exactly 1.5 so the
# clips actually engage, with a relief cavity below for body + pins.
MX_CUT        = 14.1   # 14.0 nominal + 0.1 print tolerance
MX_PLATE_T    = 1.5    # MUST stay 1.5 -- the clips need it
MX_CORNER_R   = 0.4
MX_BODY_SQ    = 16.0   # below-plate body + wiggle, relief pocket
MX_BODY_DROP  = 6.5    # plate underside -> switch base
MX_PIN_DROP   = 3.5    # pins below the switch base
MX_SOCKET_W   = 1.6    # relief pocket wall

# ---- board hold-downs -------------------------------------------------
# The ESP32 and the amp were located in XY and free in Z: rails and a cage
# beside the ESP32 with nothing over it, and corner tabs overhanging the
# amp's PCB by 0.2 mm, which is not a fixing. Same pattern as spk-clamp and
# mic-tab: a fixed lip on one edge to slide under, a screwed bar on the other.
BOARD_LIP     = 1.0    # how far a fixed lip reaches over the PCB edge.
                       # Sized to the AMP, which has only 1.1 mm of bare
                       # PCB past its header; the ESP32 has 2.8. Both lips
                       # sit on the SHORT (X) ends -- along Y the headers
                       # come within 0.3 mm of the board edge.
BOARD_LIP_T   = 1.4    # lip thickness
BOARD_TAB_W   = 8.0    # screwed retainer bar width
BOARD_TAB_T   = 2.0

# ------------------------------------------------ MX switch + keycap ----
# Cherry MX geometry above the plate. The switch itself is bought; the cap
# is printed. TODO verify against your actual switch -- clone uppers vary.
MX_UPPER_SQ   = 15.6   # upper housing, sits ON the plate
MX_UPPER_H    = 6.2    # plate top -> top of the upper housing
MX_STEM_SQ    = 7.2    # stem boss the cap slides over
MX_STEM_UP    = 3.6    # stem standing proud of the upper housing
MX_CROSS_A    = 4.1    # stem cross, long arm
MX_CROSS_B    = 1.35   # ...and its width
KEYCAP_BASE   = 18.0   # 1u
KEYCAP_TOP    = 13.4   # top face, after the taper
KEYCAP_H      = 9.0    # cap underside -> cap top
KEYCAP_WALL   = 1.6
KEYCAP_SOCKET = 4.2    # how deep the cross socket runs up into the cap
KEYCAP_GAP    = 0.6    # cap underside clear of the upper housing when up

# ============================================== bulk capacitor (servo rail) =
CAP_D         = 16.0   # TODO verify -- 2200uF 16V radial
CAP_H         = 26.0   # TODO verify
CAP_FIT       = 0.6

# ================================================================= base ====
# Round "pebble" base: a straight cylinder that rolls into a tapered shoulder,
# so the silhouette reads as one turned form rather than a box with a lid.
BASE_D        = 160.0  # outer diameter
BASE_H        = 58.0   # floor bottom -> shoulder top (TALL: the wire volume)
# THE SHOULDER IS A SEPARATE PART. It has to be: the taper closes the mouth
# to O113 while the interior is O155, so anything sitting against the wall --
# the speaker especially -- physically cannot be got in past it. Split at
# BASE_STRAIGHT and the tub is open to its full bore; everything drops
# straight down into place.
# Z34: as low as the taper can start. The speaker pocket tops out at Z32.4
# and the tub wall has to stay straight past it, so this is the constraint,
# not a preference. Every mm of taper height is a degree off the overhang.
BASE_STRAIGHT = 34.0   # tub rim / shoulder split line
BASE_TOP_D    = 118.0  # diameter where the shoulder meets the lid
# The lid drops INSIDE that bore and its top face finishes flush with the
# shoulder's rim, so the base reads as one turned form. It used to be
# BASE_TOP_D itself -- a O118 plate sitting on a O118 opening, standing
# LID_T (3.4) proud of it, which is the step you could see in the assembly.
# 0.30 diametral, 0.15 per side. It was 0.40 and the printed lid rattled.
# Clearance is NOT what stops it turning though -- no achievable slip fit
# does. LID_KEY_N tabs do that.
LID_FIT       = 0.30
LID_OD        = BASE_TOP_D - 2 * WALL_STRUCT - LID_FIT

# The lid used to land on four M3 lugs cantilevered 12 mm into the bore.
# They printed as drooping string with the screw holes in mid air, and the
# screws were the only thing stopping the lid spinning. Both jobs are now
# done by the ring itself: a continuous seat ledge takes the weight all the
# way round, and three keys take the torque. No screws, no lugs.
SEAT_LEDGE_W  = 2.0    # how far the ledge steps inward from the rebate bore
SEAT_RAMP     = 2.0    # 45 deg relief under it, so it self supports
LID_KEY_N     = 3
LID_KEY_W     = 6.0    # tab width across the chord
LID_KEY_D     = 1.6    # radial depth
LID_KEY_FIT   = 0.25   # per side, on the key flanks
BASE_W        = BASE_D # (compat) the footprint is square-bounded by the circle
BASE_R        = BASE_D / 2.0
LID_T         = 3.4
LID_Z0        = BASE_H              # 58.0
LID_Z1        = BASE_H + LID_T      # 61.4
LID_SKIRT_T   = 1.2
LID_SKIRT_H   = 6.0
LEDGE_Z       = BASE_H - LID_SKIRT_H   # 52.0 -- wall thins above this
FOOT_D        = 12.0
FOOT_H        = 3.0    # lifts the base so the speaker slots breathe
# NO lid screw towers. Four 56 mm x 7.4 posts from the floor is a lot of
# fragile plastic to hold a lid that is already clamped by the base joint's
# four M3s (they pass through the lid into the bulkhead inserts). The rim is
# held by a snap bead on the skirt instead.
LID_SEAT_Z    = 55.0   # shoulder stops tapering here; straight rebate above,
#                        so the skirt has a CONSTANT bore to fit
SNAP_BEAD     = 0.45   # bead on the skirt / groove in the seat
SNAP_Z        = 55.0

# base servo bay: MG996R lying on its SIDE at the top rear, shaft along +X.
# The lower arm's yoke straddles it and swings up through a lid slot.
# The base joint housing bolts ON TOP OF THE LID (like the reference lamp,
# whose arm pivots above the base surface), through the lid into inserts in
# the bulkhead tops -- so the arm's load reaches the floor directly, and the
# lid's rib web backs it up.
BSERVO_AXIS_Y = 18.0
# Bulkheads: back in by request. audit_loads.py says the lid rib web alone is
# enough (38% of design stress), so treat these as belt-and-braces -- a second,
# direct load path straight down to the floor, and a much stiffer tub.
#
# What wrecked the last build was not bulkheads, it was SOLID bulkheads. These
# are WINDOWED: a 5 mm frame around a big opening, so the harness crosses
# straight through instead of routing around. BULKHEADS = False drops them and
# nothing else needs changing -- the lid web still carries the arm on its own.
BULKHEADS      = True
BULK_T         = 5.0
BULK_Y         = (2.0, 34.0)   # clears the board (ends Y=1) and the bulk cap
BULK_WIN_INSET = 5.0           # frame width around the cable window
BULK_WIN_Z     = (6.0, 40.0)
JOINT_BOLT_X   = 35.0  # base-joint bolts, +-X. Set by the yoke corridor: the
#                        boss OD must clear |X| 25.45..29.85, not just its centre.
JOINT_BOLT_Y   = (8.0, 30.0)   # both land on the bulkheads

# Rib depth is set by audit_loads.py, not by eye: at Z0=50 / t=3.2 the web
# ran over the design stress. Nothing under the rib footprint (radius < 53)
# is taller than the board at Z 20.9, so there is room to go deeper.
LID_RIB_Z0    = 44.0
LID_RIB_T     = 4.0
# Lid hold-down lugs on the seat ring. Short (Z 48..58) and merged into the
# wall by a radial web, so they are rim detail, not towers in the bay.
# Pulled in from r53 to r48. With the lid recessed to O112.8 its rim is at
# r56.4, and an M3 counterbore (O6.2) centred at r53 reached r56.1 -- it would
# have broken straight out of the edge. At r48 there is 5.3 mm of rim left.
# LUG_POS and LUG_Z0 are gone. They described four cantilevered M3 lugs that
# printed in mid air; the lid now lands on a continuous ledge and is keyed.
# Leaving them here would have let the audits keep validating against a
# fixing the geometry no longer has, which is exactly how this shipped.
# Shoulder-to-tub mounts. Three points, hand-placed into the only gaps left
# at the rim (the speaker land, USB boss, mic boss and cap clamp take the rest).
SHOULDER_POS  = [(-51.0, 51.0), (-51.0, -51.0), (51.0, -51.0)]
SHOULDER_BOSS_Z = 26.0
YOKE_GAP      = 1.0    # yoke inner face -> housing outer face, per side
BJOINT_PLINTH = 8.0    # standoff under the base joint
BJOINT_AXIS_Z = 82.65  # world Z of the base pivot. Set by the yoke's swing
#                        radius (18.87), NOT by the housing: the yoke has to
#                        sweep past the lid without touching it.

# electronics bay (front / middle)
# Board mounts UPSIDE DOWN: header pins UP where you can actually reach them,
# components + USB-C hanging below. ESP_Z is the PCB's lower face either way.
ESP_FLIP      = True
ESP_Z         = 12.0   # PCB lower face; 9.6 below it for the USB-C shells
#                        (3.3) and the WROOM module (3.1)
ESP_CTR       = (38.0, -14.0)  # board center (X, Y). Pushed +X and toward the
#                                centreline: on a ROUND wall that is where the
#                                skin comes closest to the board's port edge.
USB_BOSS      = True   # local flat land bridging curved wall -> flat board end
SPK_WALL      = "left"          # fires -X through the left-wall grille
SPK_CTR_X     = -69.0           # speaker face X (fires -X)
SPK_CTR_Y     = -6.0
SPK_CTR_Z     = 22.0
# No bulkhead. The whole tub is the back volume, which also leaves the bay
# open for the driver's leads -- see part_base._speaker_bay.
SPK_SCREW_X   = SPK_CTR_X + SPK_T + SPK_FIT - 2.0   # clamp screw centres
# Same point in spk-clamp's own frame, whose origin is the screw axis: the
# tongues start at the pocket's back face, just clear of the driver.
SPK_SCREW_X_LOCAL = 2.0
MIC_CTR_X     = 0.0             # on the FRONT wall, facing the user
MIC_CTR_Z     = 30.0
USB_CTR_Y     = -14.0           # right wall window, follows the board
CAP_CTR       = (46.0, 46.0)    # rear-right corner, outboard of the bulkhead
AMP_CTR       = (0.0, 30.0)     # rear floor, between the bulkheads

# MX key in the lid -- front left, clear of the yoke slot
# On the lid's centreline. Dead centre is not available -- the base joint
# occupies X-40..40, Y-15.8..36 -- so the key sits on X=0 in the clear band
# in FRONT of it, 6.2 mm off the joint and 6.9 mm off the front seat lug.
MX_CTR        = (0.0, -30.0)

# =================================================================== arm ====
# Joint axis to joint axis. Three lift joints (base/shoulder/elbow) + head
# tilt, per the locked layout.
ARM_LOWER_L   = 120.0  # base pivot   -> shoulder axis
ARM_UPPER_L   = 120.0  # shoulder     -> elbow axis
ARM_FORE_L    = 105.0  # elbow        -> head tilt axis
# Segment minimum = YOKE_ABOVE + CONVERGE_Z + (MG_L - MG_SHAFT_OFF) + FLARE_Z.
# The yoke has to straddle a 49-wide housing, so it cannot neck down to the
# 24-wide tube in less than ~26 mm at 45 deg. That sets the arm's scale.
ARM_W         = 24.0   # segment width  (across the pitch axis)
ARM_D         = 16.0   # segment depth  (in the swing plane)
ARM_WALL      = 2.0
CABLE_CH_W    = 5.0    # exterior cable channel width
CABLE_CH_D    = 2.2    # channel depth into the segment wall
CLIP_PITCH    = 32.0   # zip-tie clip spacing along a segment
CLIP_SLOT     = (2.6, 4.0)   # zip-tie slot (w, l) -- fits a 2.5 mm tie

# ---- joint geometry (shared by base / shoulder / elbow) -------------------
# Every segment is modelled AND PRINTED standing on its yoke, long axis along
# Z. That makes each cross-section a print layer: nothing overhangs, nothing
# bridges, no supports anywhere. The servo drops into the cup from the TOP.
YOKE_BELOW    = 16.0   # yoke plate reach below its pivot axis
YOKE_ABOVE    = 26.0   # ...and above, before it necks into the arm tube
YOKE_PLATE_T  = 4.0
YOKE_DEPTH    = 20.0   # yoke plate depth in Y
JOINT_GAP     = 1.0    # yoke inner face -> housing outer face

# ---- printed retaining screw (the arm's axial keeper) ------------------
# The yoke had NOTHING holding it on: spread the plates 3 mm and the whole
# segment lifted off the stub axle. This is that keeper, and it prints.
#
# M6 x 2.0 trapezoidal, not a real M-profile. An M3 at 0.5 pitch is a
# quarter of a 0.4 nozzle per flank and simply does not exist once sliced;
# 2.0 mm of pitch is 10 layers a turn at 0.2 and comes out as a thread you
# can actually run a nut down.
AXLE_D        = 11.0   # stub axle OD -- also the joint's bearing surface
AXLE_FIT      = 0.3    # idler bore clearance over it
AXLE_LEN      = 5.0    # reaches the yoke plate's outer face, flush
SCREW_MAJOR   = 6.0
SCREW_PITCH   = 2.0
SCREW_FIT     = 0.35   # cut on the BORE, so the screw itself stays nominal
SCREW_ENGAGE  = 5.0    # == AXLE_LEN. Kept inside the axle deliberately: any
                       # deeper and the bore breaks into the servo pocket
                       # behind it, and it lets ONE screw serve the MG996R
                       # joints and the smaller SG90 head joint alike.
SCREW_HEAD_D  = 16.0   # > idler bore, which is the whole point
SCREW_HEAD_T  = 3.5
SCREW_KNURL_N = 12
JOINT_WALL    = 3.0    # housing wall
# Servo-cup cap fixing. CAP_BOSS has to clear an M3 insert bore with 1.6 of
# wall round it, so it cannot live in the 3.0 rim -- it grows inward from the
# cup corner instead, ABOVE the servo body where there is nothing to hit.
CAP_BOSS      = 7.2    # M3_INSERT_D 4.0 + 2 x 1.6
CAP_BOSS_GAP  = 0.5    # boss starts this far above the servo's top face
CAP_T         = 3.0    # cap plate thickness
CAP_CB        = 1.8    # counterbore so the M3 heads sit flush
FLARE_Z       = 16.0   # 45-deg-safe tube -> housing transition
CONVERGE_Z    = 28.0   # yoke plates -> tube (inner edge travels 25.45 @ 42 deg)

# ================================================================== head ====
# Proper Pixar cone: wide mouth, long taper, apex behind. The tilt pivot sits
# BEHIND the cone (where the taper is narrower than the servo housing) -- that
# is the only place a yoke can straddle a cone without burying itself in it.
SHADE_OD      = 66.0   # mouth outer diameter
SHADE_APEX_D  = 20.0   # diameter at the back of the taper
SHADE_DEPTH   = 52.0   # mouth -> back of taper
SHADE_WALL    = 1.6    # thin on purpose: it hangs off an SG90
SHADE_COLLAR  = 10.0   # outward flare tying the yoke plates to the cone
# Was 60.0, which put the cone's apex 8 mm from the head pivot while the
# head cup's cap is 35 mm across and reaches ~18 mm out. The shade's collar
# and yoke arm ran straight through the cap: 779 mm3 of solid overlap, found
# by the pairwise interference sweep and not by any fit check.
# TILT is where the pivot bore sits in the shade's own frame, so raising it
# lengthens the yoke arm and carries the cone outboard, clear of the cap.
# Measured: clashes at 68, clear from 70. 74 leaves 4 mm of margin, and the
# print height is unchanged at 72 mm so plate-4-head-2 is the same size.
SHADE_TILT_Z  = 74.0   # tilt axis, local Z (mouth = 0)
SHADE_VENTS   = 8      # apex vent slots
SHADE_LIP     = 2.0    # bezel lip retaining the LED ring

# ------------------------------------------------- printed horn couplers ----
# The MG996R output is a 25-tooth spline at ~5.92 OD. A true involute spline
# at that pitch is ~0.37 mm per tooth flank -- under a 0.4 nozzle, so it will
# NOT print. Instead the bore is a root circle with SPLINE_N round-bottomed
# scallops the metal teeth seat into: 0.9 mm features print cleanly and the
# shaft self-centres. Torque still ends up in the yoke's cross slot.
SPLINE_N      = 25
SPLINE_ROOT_D = 5.30   # TODO verify -- MG996R spline root circle
SPLINE_TIP_D  = 5.92   # TODO verify -- spline tip circle
SPLINE_CLEAR  = 0.15
SCALLOP_D     = 0.90   # scallop diameter (>= 2 * nozzle to print)
# The printed horn needs a STEPPED bore: spline at the servo end, M3 shank
# through the middle, head counterbore at the outer face. A straight 5.45
# spline bore all the way through would let the screw head drop right into
# it and clamp nothing.
HORN_CB_D     = 6.5    # head counterbore
HORN_CB_H     = 1.6
HORN_SPLINE_L = 4.5    # spline engagement -- MUST stay under the ~4.7 mm of
#                        shaft that actually protrudes, or the hub bottoms
#                        out on the servo's boss before it grips
HORN_DISC_D   = 26.0   # adapter that captures a STOCK round horn
HORN_SCREW_R  = 8.0    # bolt circle onto the stock horn  TODO verify

# ================================================================ colors ====
# ---- component models (cad/components.py) --------------------------------
# Dimensions off the product photos + datasheets. These are VISUAL FIT
# models: they exist so you can see whether a thing seats, not to be
# manufactured. Everything here is TODO until calipered.
ESP_MOD_L, ESP_MOD_W, ESP_MOD_H = 18.0, 25.5, 3.1   # ESP32-S3-WROOM-1
ESP_ANT_L     = 6.0    # antenna tab overhanging the PCB edge
ESP_USB_D     = 7.3    # USB-C shell depth into the board
ESP_BTN       = 6.0    # tactile buttons
ESP_HDR_N     = 23     # pins per long edge
ESP_HDR_PITCH = 2.54
ESP_HDR_BODY  = 2.5    # yellow plastic strip under the PCB
MIC_FLAT      = 12.6   # INMP441 round board is a circle with two flats
MIC_PKG       = (4.7, 3.8, 1.0)
MIC_PINS      = (2, 3)
AMP_TERM      = (10.0, 7.5, 10.0)   # green 2-pin screw terminal
AMP_PINS      = 7
SPK_CONE      = (30.0, 14.0)        # oval driver
SPK_WIRE_D    = 2.2
MG_LABEL_H    = 0.6    # the coloured sticker on top
SG_HORN_ON    = True

COLORS = {
    "base":   "#EDE7DC",   # warm off-white, like the reference lamp
    "lid":    "#E4DCCE",
    "arm":    "#F2EDE4",
    "joint":  "#3A3D42",   # dark joint covers
    "shade":  "#F2EDE4",
    "ring":   "#FFC85C",   # the glow
    "keycap": "#2C6BED",   # the one blue thing on the whole lamp
    "servo2": "#1E44A8",   # SG90 -- TowerPro blue, so the head servo reads
                           # as its own part and not as more dark joint
                           # ("servo" itself lives in the component block
                           #  below -- it was declared up here too, and the
                           #  later one silently won)
    "board":  "#1E6E3C",
    "metal":  "#9BA1A9",
    # component models
    "pcb-black":  "#14171C", "pcb-purple": "#5B2A83", "pcb-green": "#0E5B3A",
    "gold":       "#C9A227", "shield":     "#B9A98A", "silver":    "#B8BCC2",
    "white":      "#F2F2F0", "terminal":   "#2FBF71", "speaker":   "#1A1A1A",
    "servo":      "#1E1F22",   # MG996R -- black case
    "brass":      "#C9A227", "label":     "#6B4FA8",
    "wire-red":   "#C0392B", "wire-black": "#202020", "cone":      "#101010",
}
