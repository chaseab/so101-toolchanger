# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Extra end-effectors for the SO-101 tool changer (v4 coupling).  2026-10-28, agent tc-tools.
Run:  python toolchanger/tools_x/build_tools_x.py
Writes tools_x/<tool>/stl/*.stl (assembled pose, TOOL frame) + tools_x/<tool>/parts.json.  Checks: check_tools_x.py
(system python), export/GLB/STEP + rack layout: export_tools_x.py.

TOOL frame = build_gripper_tool / build_changer: coupling face z = 0, ring axis = z, tool along -z, +x = UP in the dock.
Every tool keeps the GT1 coupling half EXACTLY: gt_plate (+3 steel pins), rotor_v4, gt_shaft and the GT neck (36 sq dock
key, 4 inserts at r 19, flush N52 neck magnet).  VAC1 and MAG1 also keep the whole GT body + pinion (same prints): only the
parts under the body change.  So the rotor stops and their clocking are GT1's (closed s = 0, open s = 21 mm = 120.3 deg).

  VAC1  suction cup, NO pump: the -y rack drives the plunger of a 5 mL Luer syringe 21 mm (~2.4 mL); a 20 mm bellows cup
        on the tool axis is teed to the syringe by a short silicone tube.  open stop = plunger in = released.
  MAG1  magnetic pickup: both racks carry a cam plate with an inclined slot; a carrier with 7x N52 6x3 rides on 2 steel
        dowels in the slots and lifts 16 mm off a 1.0 mm face.  open stop = magnets up = part drops.
  PEN1  sprung marker holder: Sharpie on the tool axis in a carriage on 2 springs (6 mm of travel); a stop dog on the GT
        shaft gives the rotor its two solid stops (the blade search needs them), at GT1's shaft angles.
"""
import json
import sys
from math import atan2, cos, degrees, radians, sin, sqrt
from pathlib import Path

from build123d import Box, Cone, Cylinder, Plane, Polygon, Pos, Rot, export_stl, extrude

H = Path(__file__).resolve().parent
CAD = H.parent / "cad"
sys.path.insert(0, str(CAD))
import build_changer as B          # noqa: E402
import build_changer_v4 as V       # noqa: E402
import build_gripper_tool as GT    # noqa: E402

S_OPEN = GT.S_OPEN                 # 21 mm per rack
Z_BOT = GT.Z_BOT                   # -32.4 body underside
SCREWS = GT.COVER["screw"]         # (+-20, +-20.8): the GT cover's 4 pilots in the body underside


def bx(xs, ys, zs):
    return Pos(sum(xs) / 2, sum(ys) / 2, sum(zs) / 2) * Box(abs(xs[1] - xs[0]), abs(ys[1] - ys[0]), abs(zs[1] - zs[0]))


def rack1_bar():
    """GT jaw 1's rack bar + back tongue + keel (closed pose, +y lane), without the web / stub."""
    z0, z1 = GT.RACK_Z
    r = Pos(0, 0, z1) * extrude(Polygon(*GT.rack_teeth_profile(GT.RACK_X[0], GT.TEETH_X1), align=None), amount=z0 - z1)
    r += bx((GT.TEETH_X1, GT.RACK_X[1]), (GT.Y_TIP, GT.RACK_BACK), GT.RACK_Z)
    r += GT.prism_yz(GT.tongue_profile(), *GT.RACK_X)
    r -= Pos(sum(GT.RACK_X) / 2, GT.RACK_BACK, z0) * Rot(45, 0, 0) * Box(GT.RACK_X[1] - GT.RACK_X[0] + 1, 0.8, 0.8)
    r += bx(GT.KEEL["x"], GT.KEEL["y"], GT.KEEL["z"])
    return r


def case_shell(z_floor_top, z_bottom, pocket_hy=GT.COVER["skirt_in"], cb_top=None):
    """Lower case under the GT body: 78 x 46 box from the body underside down to z_bottom, open-top pocket (+-36.2 x
    pocket_hy, the GT cover window / skirt line, so the racks' webs are guided like GT1's), 4 M3 through holes at the GT
    cover screw positions with counterbores from below (M3 socket head)."""
    c = bx((-GT.BX, GT.BX), (-GT.BY, GT.BY), (Z_BOT, z_bottom))
    c -= bx((-GT.COVER["win_x"], GT.COVER["win_x"]), (-pocket_hy, pocket_hy), (Z_BOT + 1, z_floor_top))
    for x, y in SCREWS:
        c -= Pos(x, y, (Z_BOT + z_bottom) / 2) * Cylinder(1.7, Z_BOT - z_bottom + 2)
        top = cb_top if cb_top is not None else z_bottom + 8.0
        c -= Pos(x, y, (z_bottom + top) / 2 - 0.5) * Cylinder(3.3, top - z_bottom + 1)
    # lightening: the 8 mm side walls are hollowed from below between the screw bosses (2 mm skins stay, 3 mm roof)
    for sy in (1, -1):
        c -= bx((-14.0, 14.0), (sy * 17.05, sy * 21.0), (z_bottom - 1, Z_BOT - 3.0))
    return c


# ====================================================================== VAC1
SYR = dict(z=-52.0, barrel_od=14.0, barrel_id=12.06, flange_x=(-6.6, -5.0), flange=(30.0, 20.0), barrel_x=(-63.6, -6.6),
           tip_x=(-73.6, -63.6), thumb_d=16.0, thumb_t=1.5, rod_w=11.0, thumb_x_closed=26.0, wall=1.0)   # BD-style 5 mL, UNMEASURED
# r1 (critics A4/B2): the syringe sits 3.6 mm further out so 2 case walls can hug its flange (pull + push), and the rod is
# sized so the stopper face is 0.5 mm short of the barrel bottom at the open stop (the syringe must never be the stop; a
# real syringe whose bottomed thumb-to-flange distance is > 10.0 mm needs a shim behind the flange).
SYR["rod_l"] = (SYR["thumb_x_closed"] - S_OPEN) - (SYR["barrel_x"][0] + SYR["wall"] + 0.5) - 6.0
FL_WALL = 2.0                                   # flange walls (x), flange clearance 0.2/side
VAC_FLOOR, VAC_BOT = -62.0, -65.0               # 3 mm floor: 2 countersunk M3 hold the cup puck from inside
YOKE = dict(x=(-29.5, -20.5), hy=14.9, z_bot=-61.5)      # rack-1 frame (the part is turned 180 deg into the -y lane)
CUP = dict(d=20.0, h=18.0, boss_r=11.0, boss_h=8.0, insert_d=6.4, insert_l=7.0)
CUP_SCREWS = [(2.0, 7.5), (2.0, -7.5)]          # r1: moved off x -7.5 (now under the flange wall)   # bellows cup (M5 male) in an M5 heat-set insert in a separate puck


def vac_yoke():
    """Rack 1 bar + a yoke block that hangs through the open tunnel bottom and grips the syringe thumb disc.
    (rack-1 frame: thumb at x = -26 when closed; in the tool the part is Rot z 180 -> -y lane, thumb at +26.)"""
    r = rack1_bar()
    xs, hy = YOKE["x"], YOKE["hy"]
    zc = SYR["z"]
    # web from the keel down to the block (inside the cover skirt line, like GT1's jaw web)
    r += bx(xs, (GT.KEEL["y"][0], hy), (GT.RACK_Z[1], zc))
    r += bx(xs, (-hy, hy), (GT.WEB_LOW["z_full"], YOKE["z_bot"]))
    # 45-deg step under the rack teeth so the block prints without support (as GT1's web)
    r += GT.prism_yz([(GT.KEEL["y"][0] + 0.01, GT.RACK_Z[1]), (GT.KEEL["y"][0] + 0.01, GT.WEB_LOW["z_full"]),
                      (-hy, GT.WEB_LOW["z_full"]), (-hy, GT.WEB_LOW["z_full"] + 0.01)], *xs)
    # thumb-disc slot (1.5 disc + 0.3) and the plunger-rod slot toward the barrel (+x here), both open at the bottom
    xt = -SYR["thumb_x_closed"]
    r -= bx((xt - 0.9, xt + 0.9), (-SYR["thumb_d"] / 2 - 0.4, SYR["thumb_d"] / 2 + 0.4), (zc + SYR["thumb_d"] / 2 + 0.4, YOKE["z_bot"] - 1))
    r -= bx((xt, xs[1] + 1), (-SYR["rod_w"] / 2 - 0.4, SYR["rod_w"] / 2 + 0.4), (zc + SYR["rod_w"] / 2 + 0.4, YOKE["z_bot"] - 1))
    return r


def vac_case():
    """Case under the body: guides the yoke between the skirt walls, holds the syringe flange in a slot, the barrel in a
    snap cradle that runs out past the body's -x face, and the cup boss (M5 heat-set insert) on the tool axis below."""
    c = case_shell(VAC_FLOOR, VAC_BOT)
    zc, ro = SYR["z"], SYR["barrel_od"] / 2
    # flange slot (30 x 20 flange, long axis along y) cut into the side walls
    fx = SYR["flange_x"]
    c -= bx((fx[0] - 0.2, fx[1] + 0.2), (-SYR["flange"][0] / 2 - 0.6, SYR["flange"][0] / 2 + 0.6), (zc + SYR["flange"][1] / 2 + 0.6, VAC_FLOOR + 0.01))
    # r1: two cross walls hug the flange (0.2/side): the outer one takes the push (notch for the barrel), the inner one the
    # pull (notch for the plunger rod); both open on top so the syringe drops in on the bench
    hy, ztop = GT.COVER["skirt_in"], zc + 6.0
    wo = bx((fx[0] - 0.2 - FL_WALL, fx[0] - 0.2), (-hy, hy), (VAC_FLOOR - 0.01, ztop))
    wo -= Pos(fx[0] - 1.2, 0, zc) * Rot(0, 90, 0) * Cylinder(ro + 0.3, FL_WALL + 2)
    wo -= bx((fx[0] - 4, fx[0]), (-ro - 0.3, ro + 0.3), (zc, ztop + 1))
    wi = bx((fx[1] + 0.2, fx[1] + 0.2 + FL_WALL), (-hy, hy), (VAC_FLOOR - 0.01, ztop))
    wi -= bx((fx[1], fx[1] + FL_WALL + 1), (-SYR["rod_w"] / 2 - 0.7, SYR["rod_w"] / 2 + 0.7), (zc - SYR["rod_w"] / 2 - 0.7, ztop + 1))
    c += wo + wi
    # cradle beyond the body: an open U channel (sides to 1 mm over the barrel axis: no snap, the barrel drops in) from
    # x -36 to -66; 2 zip-tie tunnels across it hold the barrel down (r1, critic B3: the snap arms would crack)
    cx = (-66.0, -36.0)
    cr = bx(cx, (-ro - 2.5, ro + 2.5), (VAC_BOT, zc + 1.0))
    cr -= Pos(sum(cx) / 2, 0, zc) * Rot(0, 90, 0) * Cylinder(ro + 0.4, cx[1] - cx[0] + 2)   # r2: 0.4 (opening 14.7)
    for xt in (-61.0, -45.0):
        cr -= bx((xt - 1.5, xt + 1.5), (-ro - 3.0, ro + 3.0), (VAC_BOT + 1.5, VAC_BOT + 3.0))     # tie tunnel under the barrel
        for sy in (1, -1):
            cr -= bx((xt - 1.5, xt + 1.5), (sy * (ro + 1.0), sy * (ro + 3.0)), (VAC_BOT + 1.5, zc + 2))
    c += cr
    # barrel bore through the case's -x end wall (the barrel runs from the flange out to x -60)
    c -= Pos(-37.5, 0, zc) * Rot(0, 90, 0) * Cylinder(ro + 0.3, 6)
    c -= bx((-40, -35), (-ro - 0.3, ro + 0.3), (zc, zc + 20))           # open from above so the barrel drops in
    # 2 countersunk holes in the floor for the cup puck (heads flush with the floor top: the yoke slides 0.5 above)
    for x, y in CUP_SCREWS:
        c -= Pos(x, y, (VAC_FLOOR + VAC_BOT) / 2) * Cylinder(1.7, 5)
        c -= Pos(x, y, VAC_FLOOR - 0.85 + 0.01) * Cone(1.7, 3.4, 1.7)
    return c


def vac_puck():
    """Cup puck under the case floor on the tool axis: M5 heat-set insert from below (the cup's M5 stud), 2 M3 pilots
    from above for the countersunk screws through the floor.  Separate so the case prints floor-down without supports."""
    z0, z1 = VAC_BOT, VAC_BOT - CUP["boss_h"]
    p = Pos(0, 0, (z0 + z1) / 2) * Cylinder(CUP["boss_r"], z0 - z1)
    p -= Pos(0, 0, z1 + CUP["insert_l"] / 2 - 0.01) * Cylinder(CUP["insert_d"] / 2, CUP["insert_l"])
    for sx in (1, -1):
        p -= Pos(*CUP_SCREWS[(1 - sx) // 2], z0 - 3.0 + 0.01) * Cylinder(GT.PILOT_D / 2, 6.0)
    return p


def syringe(s):
    """5 mL syringe proxy at rack opening s (barrel + flange + plunger rod + thumb disc, one solid for renders)."""
    zc = SYR["z"]
    x0, x1 = SYR["barrel_x"]
    t = Pos((x0 + x1) / 2, 0, zc) * Rot(0, 90, 0) * Cylinder(SYR["barrel_od"] / 2, x1 - x0)
    t += Pos(sum(SYR["tip_x"]) / 2, 0, zc) * Rot(0, 90, 0) * Cone(2.0, 1.6, SYR["tip_x"][1] - SYR["tip_x"][0])
    fx = SYR["flange_x"]
    t += bx(fx, (-SYR["flange"][0] / 2, SYR["flange"][0] / 2), (zc - SYR["flange"][1] / 2, zc + SYR["flange"][1] / 2))
    return t


def plunger(s):
    """Plunger rod + thumb disc + rubber stopper at rack opening s (moves with the -y rack)."""
    zc = SYR["z"]
    xt = SYR["thumb_x_closed"] - s
    t = Pos(xt, 0, zc) * Rot(0, 90, 0) * Cylinder(SYR["thumb_d"] / 2, SYR["thumb_t"])
    rod_x = (xt - SYR["rod_l"], xt)                                             # rod runs into the barrel; stopper at its end
    t += bx(rod_x, (-SYR["rod_w"] / 2, SYR["rod_w"] / 2), (zc - 1.0, zc + 1.0))
    t += bx(rod_x, (-1.0, 1.0), (zc - SYR["rod_w"] / 2, zc + SYR["rod_w"] / 2))
    t += Pos(rod_x[0] - 3.0, 0, zc) * Rot(0, 90, 0) * Cylinder(SYR["barrel_id"] / 2 - 0.05, 6.0)
    return t


def cup():
    """20 mm bellows suction cup with an M5 elbow fitting (proxy)."""
    z = VAC_BOT - CUP["boss_h"]
    f = Pos(0, 0, z - 3.0) * Cylinder(4.0, 6.0)                                            # hex fitting body
    f += Pos(0, 0, z - 4.0) * Rot(0, 90, 0) * Pos(0, 0, -6) * Cylinder(1.6, 12.0)          # barb toward -x
    zb = z - 6.0
    f += Pos(0, 0, zb - 3.0) * Cylinder(6.0, 6.0)
    f += Pos(0, 0, zb - 6.0 - 4.0) * Cone(CUP["d"] / 2 - 2, 7.0, 8.0)                      # bellows
    f += Pos(0, 0, zb - 14.0 - 2.0) * Cone(CUP["d"] / 2, CUP["d"] / 2 - 3, 4.0)            # lip
    return f


def tube():
    """Silicone tube 3 x 5 mm from the syringe tip round to the cup barb (polyline of cylinders, render proxy).
    r1 (critic B5, no kinks): a flat R 10 U-turn beside the syringe tip (in the xy plane, so it stays above the DB1 back bar
    at dock x 58.5-69 / frame), then down along the cradle's +y side and under the case to the barb (one zip tie)."""
    zc, zb = SYR["z"], VAC_BOT - CUP["boss_h"] - 4.0
    x0, rr = SYR["tip_x"][0] + 1.0, 10.0
    pts = [(x0 - rr * sin(radians(a)), rr - rr * cos(radians(a)), zc) for a in range(0, 181, 30)]
    pts += [(-66.0, 20.0, zc - 2), (-58.0, 20.0, zc - 10), (-50.0, 20.0, zb + 5), (-42.0, 19.0, zb + 0.5), (-34.0, 16.0, zb),
            (-26.0, 10.0, zb), (-19.0, 4.0, zb), (-14.0, 1.0, zb), (-11.0, 0.0, zb)]
    out = None
    for a, b in zip(pts, pts[1:]):
        L = sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
        mid = [(a[i] + b[i]) / 2 for i in range(3)]
        d = tuple((b[i] - a[i]) / L for i in range(3))
        seg = Plane(origin=mid, z_dir=d) * Cylinder(2.5, L + 2.4)              # 3-D segment (r1: the route leaves the xz plane)
        out = seg if out is None else out + seg
    return out


# ====================================================================== MAG1
MAG_FLOOR_TOP, MAG_BOT = -60.4, -61.4              # 1.0 mm PLA pick face
CAR = dict(hx=12.0, hy=10.5, h=10.0, lift=16.0, pin_z=5.0, pin_embed=7.6)   # r1: embed 7.6 -> dowel flush with the cam face (3.4 engaged)
# r1 (critic A1): two-slope cam. 12 deg for the first 5 mm of rack (gap 1 -> 2 mm, where the pull is 17 -> 7 N), then
# 43 deg for the rest.  Lift(s) is piecewise linear; check_tools_x.py uses the same knee.
CAM_KNEE = (5.0, 5.0 * 0.2126)                     # (s, lift) at the knee: tan 12 deg
CAR_Z0 = MAG_FLOOR_TOP                             # carrier sits ON the face when engaged
PIN_Z_ENG = CAR_Z0 + CAR["pin_z"]                  # -55.4
CAM = dict(x=(-27.0, 5.0), y=(11.5, 14.9), z_bot=-59.0, slot_w=3.4)
CHAN = dict(x=(12.3, 14.3), hy=10.9)
MAGS = [(0.0, 0.0)] + [(8.0 * cos(radians(60 * k)), 8.0 * sin(radians(60 * k))) for k in range(6)]


def slot2d(a, b, w, ext=1.0):
    """Stadium-ish slot (rectangle + round ends) in the XZ plane from a to b, as a solid 8 mm thick along y."""
    dx, dz = b[0] - a[0], b[1] - a[1]
    L = sqrt(dx * dx + dz * dz)
    ang = degrees(atan2(dz, dx))
    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    s = Pos(mid[0], 0, mid[1]) * Rot(0, -ang, 0) * Box(L + 2 * ext, 30, w)
    for p in (a, b):
        s += Pos(p[0], 0, p[1]) * Rot(90, 0, 0) * Cylinder(w / 2, 30)
    return s


def mag_cam():
    """Rack 1 bar + a cam plate under the keel with the lift slot: pin at rack x = -s rides from z -55.4 (s = 0) up to
    -39.4 (s = 21; 37 deg).  The slot's low end also runs straight down out of the plate so the carrier's pins enter
    from below when the case goes on (assembly path)."""
    r = rack1_bar()
    r += bx(CAM["x"], CAM["y"], (GT.RACK_Z[1], CAM["z_bot"]))
    a, k, b = (0.0, PIN_Z_ENG), (-CAM_KNEE[0], PIN_Z_ENG + CAM_KNEE[1]), (-S_OPEN, PIN_Z_ENG + CAR["lift"])
    cut = slot2d(a, k, CAM["slot_w"]) + slot2d(k, b, CAM["slot_w"]) + bx((-CAM["slot_w"] / 2, CAM["slot_w"] / 2), (-15, 15), (PIN_Z_ENG, CAM["z_bot"] - 1))
    r -= cut & bx((-40, 20), CAM["y"], (-80, 0))
    return r


def mag_carrier():
    c = bx((-CAR["hx"], CAR["hx"]), (-CAR["hy"], CAR["hy"]), (CAR_Z0, CAR_Z0 + CAR["h"]))
    for x, y in MAGS:                                  # 7 pockets 6.15 x 3.0, open at the face side (press + CA)
        c -= Pos(x, y, CAR_Z0 + 1.5 - 0.01) * Cylinder(3.075, 3.02)
    for sy in (1, -1):                                 # 2 blind dowel holes from the +-y faces (3.0 press, 8.3 deep)
        c -= Pos(0, sy * (CAR["hy"] - CAR["pin_embed"] / 2 + 0.01), PIN_Z_ENG) * Rot(90, 0, 0) * Cylinder(1.6, CAR["pin_embed"])   # 3.2: horizontal holes print ~0.3 small
    return c


def mag_pins(lift=0.0):
    out = None
    for sy in (1, -1):
        p = Pos(0, sy * (CAR["hy"] - CAR["pin_embed"] + 6.0), PIN_Z_ENG + lift) * Rot(90, 0, 0) * Cylinder(1.5, 12.0)
        out = p if out is None else out + p
    return out


def mag_magnets(lift=0.0):
    out = None
    for x, y in MAGS:
        m = Pos(x, y, CAR_Z0 + 1.5 + lift) * Cylinder(3.0, 3.0)
        out = m if out is None else out + m
    return out


def mag_case():
    c = case_shell(MAG_FLOOR_TOP, MAG_BOT)
    for sx in (1, -1):                                 # carrier channel walls (only |y| < 10.9: the cam plates pass outside)
        x0, x1 = sorted((sx * CHAN["x"][0], sx * CHAN["x"][1]))
        c += bx((x0, x1), (-CHAN["hy"], CHAN["hy"]), (MAG_FLOOR_TOP - 0.01, -35.0))
    # carrier y: held by its 2 dowels in the cam slots, 1.0 mm off each cam plate's inner face
    return c


# ====================================================================== PEN1
PB = dict(x=(-GT.BX, 22.0), hy=18.0, z=(-21.0, -38.0))      # pen body block under the neck (rest face x -39 = GT1's)
DOG = dict(z=(-25.0, -31.0), hub_r=6.0, arm_r=13.0, arm_w=6.0, cav_r=15.0, cav_z=-24.2,
           closed_deg=90.0, open_deg=90.0 - degrees(S_OPEN / GT.R_P))   # GT1 shaft angles: closed 0 / open -120.3 deg
RIB_W_DEG = 15.0
RIB_R0 = 9.0                                                # stop ribs from r 9 (the dog arm is +-3 wide)
# r1: bore 8.2 (slip 0.2/side), spring bores out to x 13.6 (1.6 mm wall, critic B8), spring seat at -48 = 4 mm preload on
# a 30 mm spring (critic A10: SOFT springs ~0.15 N/mm -> ~1.2-3 N pen force), carriage guided 39.5 mm (critic A11) and
# clamping the marker down to -108 (critic B10)
SLV = dict(z=(-38.0, -80.0), r=15.0, bore_r=8.2, flange=18.0, flange_t=4.0, spring_x=13.6, spring_r=3.8, spring_top=-48.0,
           boss_hy=6.0, web_hy=2.2, slot_top=-65.0, cap_t=4.0)
CARR = dict(r_out=8.0, r_in=5.6, z=(-44.5, -108.0), ear_r=3.6, ear_z=(-80.0, -74.0), web_z=(-80.0, -72.0), web_hy=2.0)
PEN = dict(back=-46.0, barrel_end=-160.0, cone_end=-176.0, tip=-184.0, r=5.45)   # Sharpie Fine Point, UNMEASURED
PEN_TRAVEL = 6.0
CLAMP_Z = (-94.5, -103.0)
CAP_SCREWS = [(8.13, 8.13), (-8.13, 8.13)]            # r1: the cap is a U open to -y (slides in sideways), screws on its +y half
#       # r 12.5 at -45 / -135 deg: clear of the clamp lug on +y (screwaccess)
PB_SCREWS = [(14.0, 14.0), (-14.0, 14.0), (14.0, -14.0), (-14.0, -14.0)]


def neck_core():
    """The GT body's neck exactly (36 sq, rotor seat, shaft bearing, 4 inserts, flush neck magnet), z -5..-21."""
    return GT.body() & bx((-20, 20), (-20, 20), (-5.0, -21.0))


def sector(r0, r1, a0, a1, z0, z1):
    return B.sector(r0, r1, a0, a1, z0, z1)


def pen_body():
    b = neck_core() + bx(PB["x"], (-PB["hy"], PB["hy"]), PB["z"])
    b -= Pos(0, 0, (GT.Z_SHOULDER + DOG["cav_z"]) / 2) * Cylinder(GT.SHAFT_BORE_D / 2, GT.Z_SHOULDER - DOG["cav_z"] + 0.2)
    zc0, zc1 = DOG["cav_z"], PB["z"][1]
    b -= Pos(0, 0, (zc0 + zc1) / 2 - 0.5) * Cylinder(DOG["cav_r"], zc0 - zc1 + 1)
    # r1 (critic A8): flat stops. Each rib's face is the plane 3 mm off the arm's centre line at GT1's stop angle, so
    # the arm's flat side lands on it (r 9.5..15 contact) exactly at the stop: no corner contact, no pad.
    hw, zz = DOG["arm_w"] / 2, (DOG["z"][1] - 1.0, zc0 + 0.01)
    for ang, sy in ((DOG["closed_deg"], 1), (DOG["open_deg"], -1)):
        a0, a1 = (0.0, 40.0) if sy > 0 else (-40.0, 0.0)
        rib = sector(RIB_R0, DOG["cav_r"] + 0.01, a0, a1, *zz) & bx((0, 20), (sy * hw, sy * 20), (zz[0] - 1, zz[1] + 1))
        b += Rot(0, 0, ang) * rib
    for x, y in PB_SCREWS:                                                   # sleeve flange pilots (M3 self-tap 2.6 x 8)
        b -= Pos(x, y, PB["z"][1] + 4.0 - 0.01) * Cylinder(GT.PILOT_D / 2, 8.0)
    return b


def pen_dog():
    z0, z1 = DOG["z"]
    d = Pos(0, 0, (z0 + z1) / 2) * Cylinder(DOG["hub_r"], z0 - z1)
    a = radians(DOG["closed_deg"])
    d += Pos(DOG["arm_r"] / 2 * cos(a), DOG["arm_r"] / 2 * sin(a), (z0 + z1) / 2) * Rot(0, 0, DOG["closed_deg"]) * Box(DOG["arm_r"], DOG["arm_w"], z0 - z1)
    d -= Pos(0, 0, GT.HEX_Z[1]) * extrude(Polygon(*GT.hexagon(GT.HEX_SOCK_AF), align=None), amount=z0 - GT.HEX_Z[1] + 0.02)
    d -= Pos(0, 0, (z1 + GT.HEX_Z[1]) / 2) * Cylinder(1.7, GT.HEX_Z[1] - z1 + 0.2)
    return d


def pen_sleeve():
    z0, z1 = SLV["z"]
    f = SLV["flange"]
    s = bx((-f, f), (-f, f), (z0, z0 - SLV["flange_t"]))
    s += Pos(0, 0, (z0 + z1) / 2) * Cylinder(SLV["r"], z0 - z1)
    sx, sr = SLV["spring_x"], SLV["spring_r"]
    s += bx((-sx - sr - 1.6, sx + sr + 1.6), (-SLV["boss_hy"], SLV["boss_hy"]), (z0, z1))
    s -= Pos(0, 0, (z0 + z1) / 2) * Cylinder(SLV["bore_r"], z0 - z1 + 2)
    for k in (1, -1):
        s -= Pos(k * sx, 0, (SLV["spring_top"] + z1) / 2 - 0.5) * Cylinder(sr, SLV["spring_top"] - z1 + 1)
        s -= bx((k * 6.0, k * sx), (-SLV["web_hy"], SLV["web_hy"]), (SLV["slot_top"], z1 - 1))
    for x, y in PB_SCREWS:                       # flange screws M3x10 button from below into the body pilots
        s -= Pos(x, y, z0 - SLV["flange_t"] / 2) * Cylinder(1.7, SLV["flange_t"] + 2)
    for x, y in CAP_SCREWS:                      # cap pilots from below
        s -= Pos(x, y, z1 + 4.0 - 0.01) * Cylinder(GT.PILOT_D / 2, 8.0)
    return s


def pen_cap():
    z1 = SLV["z"][1]
    sx, sr = SLV["spring_x"], SLV["spring_r"]
    c = Pos(0, 0, z1 - SLV["cap_t"] / 2) * Cylinder(SLV["r"], SLV["cap_t"])
    c += bx((-sx - sr - 1.6, sx + sr + 1.6), (-SLV["boss_hy"], SLV["boss_hy"]), (z1, z1 - SLV["cap_t"]))
    # r1 (critic B1): U-shaped, open to -y: it slides in sideways around the round carriage tube after the carriage is in
    c -= Pos(0, 0, z1 - SLV["cap_t"] / 2) * Cylinder(CARR["r_out"] + 0.4, SLV["cap_t"] + 2)
    c -= bx((-CARR["r_out"] - 0.4, CARR["r_out"] + 0.4), (0, -SLV["r"] - 5), (z1 + 1, z1 - SLV["cap_t"] - 1))
    c -= bx((-SLV["r"] - 1, SLV["r"] + 1), (-11.5, -SLV["r"] - 5), (z1 + 1, z1 - SLV["cap_t"] - 1))   # r2: blunt arm ends (no knife edge)
    for x, y in CAP_SCREWS:
        c -= Pos(x, y, z1 - SLV["cap_t"] / 2) * Cylinder(1.7, SLV["cap_t"] + 2)
    return c


def pen_carriage(up=0.0):
    z0, z1 = CARR["z"]
    c = Pos(0, 0, (z0 + z1) / 2) * Cylinder(CARR["r_out"], z0 - z1)
    sx = SLV["spring_x"]
    for k in (1, -1):
        c += bx((k * (CARR["r_out"] - 0.5), k * sx), (-CARR["web_hy"], CARR["web_hy"]), CARR["web_z"])
        c += Pos(k * sx, 0, sum(CARR["ear_z"]) / 2) * Cylinder(CARR["ear_r"], CARR["ear_z"][1] - CARR["ear_z"][0])
    # split clamp at the bottom: lug on +y with an M3 across (nut trap on the -x side)
    # lug top -90.5: 0.5 under the cap at the top of the 6 mm travel (the cap is the hard up-stop)
    c += bx((-4.0, 4.0), (CARR["r_out"] - 1, CARR["r_out"] + 7.2), (-90.5, z1))      # r2: +1.2 so the nut trap keeps a 1.2 wall
    c -= Pos(0, 0, (z0 + z1) / 2) * Cylinder(CARR["r_in"], z0 - z1 + 2)
    c -= bx((-0.75, 0.75), (0, CARR["r_out"] + 7), (-86.0, z1 - 1))
    for zs in CLAMP_Z:                                                                 # r1: 2 clamp screws, longer clamp
        c -= Pos(0, CARR["r_out"] + 2.8, zs) * Rot(0, 90, 0) * Cylinder(1.7, 12)
        c -= Pos(-4.0, CARR["r_out"] + 2.8, zs) * Rot(0, 90, 0) * Cylinder(3.2, 2.6)   # M3 nut trap (hex approx)
    return Pos(0, 0, up) * c


def pen_marker(up=0.0):
    r = PEN["r"]
    m = Pos(0, 0, (PEN["back"] + PEN["barrel_end"]) / 2) * Cylinder(r, PEN["back"] - PEN["barrel_end"])
    m += Pos(0, 0, (PEN["barrel_end"] + PEN["cone_end"]) / 2) * Cone(1.6, r, PEN["barrel_end"] - PEN["cone_end"])
    m += Pos(0, 0, (PEN["cone_end"] + PEN["tip"]) / 2) * Cone(0.5, 1.0, PEN["cone_end"] - PEN["tip"])
    return Pos(0, 0, up) * m


def pen_springs(up=0.0):
    out = None
    top, bot = SLV["spring_top"], CARR["ear_z"][1] + up
    for k in (1, -1):
        s = Pos(k * SLV["spring_x"], 0, (top + bot) / 2) * (Cylinder(3.5, top - bot) - Cylinder(2.8, top - bot + 1))
        out = s if out is None else out + s
    return out


# ====================================================================== export
COMMON = {"plate": GT.coupling_plate, "pins": B.pins, "rotor": V.rotor_v4, "shaft": GT.shaft}


def jaw2(shape):
    return Rot(0, 0, 180) * shape


def build():
    tools = {}
    # shared coupling half + GT body/pinion
    common = {k: f() for k, f in COMMON.items()}
    gt_body, gt_pinion = GT.body(), GT.pinion()
    # ---- VAC1 (assembled at s = 0 = closed = vacuum; the -y rack is jaw-2 style)
    vy = vac_yoke()
    tools["vac1"] = {
        "parts": {**common, "body": gt_body, "pinion": gt_pinion, "yoke": jaw2(vy), "case": vac_case(), "puck": vac_puck(),
                  "syringe": syringe(0.0), "plunger": plunger(0.0), "cup": cup(), "tube": tube()},
        "moving": {"yoke": "rack2", "plunger": "rack2", "pinion": "pinion", "shaft": "pinion", "rotor": "pinion"}, "print_shapes": {"yoke_p": vy}}
    # ---- MAG1 (assembled at s = 0 = engaged)
    mc = mag_cam()
    tools["mag1"] = {
        "parts": {**common, "body": gt_body, "pinion": gt_pinion, "cam1": mc, "cam2": jaw2(mc), "carrier": mag_carrier(),
                  "dowels": mag_pins(), "magnets": mag_magnets(), "case": mag_case()},
        "moving": {"cam1": "rack1", "cam2": "rack2", "pinion": "pinion", "shaft": "pinion", "rotor": "pinion", "carrier": "lift", "dowels": "lift", "magnets": "lift"}}
    # ---- PEN1 (assembled with the carriage down on the cap, dog at the closed stop)
    tools["pen1"] = {
        "parts": {**common, "body": pen_body(), "dog": pen_dog(), "sleeve": pen_sleeve(), "cap": pen_cap(),
                  "carriage": pen_carriage(), "marker": pen_marker(), "springs": pen_springs()},
        "moving": {"dog": "dog", "shaft": "dog", "rotor": "dog", "carriage": "pen", "marker": "pen"}}
    for name, t in tools.items():
        out = H / name / "stl"
        out.mkdir(parents=True, exist_ok=True)
        info = {"parts": {}}
        for pn, s in t["parts"].items():
            export_stl(s, str(out / f"{pn}.stl"), tolerance=0.01, angular_tolerance=0.1)
            bb = s.bounding_box()
            info["parts"][pn] = {"cm3": round(s.volume / 1000, 3), "valid": s.is_valid,
                                 "bbox": [round(v, 2) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)]}
        # alternate poses for checks / renders
        if name == "vac1":
            export_stl(jaw2(Pos(S_OPEN, 0, 0) * vy), str(out / "yoke_open.stl"))
            export_stl(plunger(S_OPEN), str(out / "plunger_open.stl"))
        if name == "mag1":
            export_stl(Pos(S_OPEN, 0, 0) * mc, str(out / "cam1_open.stl"))
            export_stl(jaw2(Pos(S_OPEN, 0, 0) * mc), str(out / "cam2_open.stl"))
            L = CAR["lift"]
            export_stl(Pos(0, 0, L) * mag_carrier(), str(out / "carrier_open.stl"))
            export_stl(mag_pins(L), str(out / "dowels_open.stl"))
            export_stl(mag_magnets(L), str(out / "magnets_open.stl"))
        if name == "pen1":
            export_stl(Rot(0, 0, DOG["open_deg"] - DOG["closed_deg"]) * pen_dog(), str(out / "dog_open.stl"))
            export_stl(pen_carriage(PEN_TRAVEL), str(out / "carriage_up.stl"))
            export_stl(pen_marker(PEN_TRAVEL), str(out / "marker_up.stl"))
        for pn, s in t.get("print_shapes", {}).items():
            export_stl(s, str(out / f"{pn}.stl"))
        info["moving"] = t["moving"]
        (H / name / "parts.json").write_text(json.dumps(info, indent=1))
        print(name, {k: v["cm3"] for k, v in info["parts"].items()}, "invalid:", [k for k, v in info["parts"].items() if not v["valid"]])
    return tools


if __name__ == "__main__":
    build()
