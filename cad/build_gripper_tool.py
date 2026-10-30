# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Changer GRIPPER TOOL v1 (the first working tool for the v4 changer).  2026-10-25, agent toolchanger-build.
Replaces the concept-level gripper_body / pinion of build_changer.py (DESIGN_v4 s5 "gripper body: still concept-level").

Run:  python toolchanger/cad/build_gripper_tool.py
Writes v4/stl/gt_*.stl (assembled pose, TOOL frame) + v4/gripper_tool.json.  Checks: gt_checks.py.

TOOL frame (build_changer.py): coupling face z = 0, tool along -z, +x = up in the dock (tool -x face rests on the dock).
DRIVE (picked over a pin-in-slot yoke and a spiral cam plate; brief toolchanger/gripper_tool/GRIPPER_TOOL.html):
  blade (master) -> rotor (v4, floats 1 mm) -> Oldham TONGUE on a shaft -> m1.25 z16 spur PINION -> two RACKS at +-y
  moving in opposite x -> two jaws, each ending in the gripper project's A1 taper stub (WRF_QC spec: 10 x 16 x 14,
  8 deg/side, 3.4 draw hole), so the existing replica / fin ray finger modules plug on.
  Work identity  T * theta = F * s  (no gearing can beat it): with ~126 deg of servo for 22 mm per jaw the ratio is fixed at
  r = 10 mm, i.e. 10 N per jaw at the 0.25 N.m changer cap (6 N at 0.15).  The red team's "gear down to r 25" would need
  55 mm of travel per jaw for the same rotation.  So this tool is for RIGID fingers (replica); fin rays need a ~0.05 N.m cap.
LAYOUT (z, tool frame):
  -5..-7.2   neck bore r 9.5 (rotor bottom 2 mm into the neck), shoulder at -7.2 carries the rotor and the tongue
  -7.2..-25  shaft r 4 in a long 8.4 bore through the solid neck + ceiling (18 mm bearing)
  -21..-24.4 body ceiling (bridges the 30.5 mm tunnel)
  -24.6..-31.4 racks (6.8) in the tunnel, a 3 mm tongue on each rack back runs in a side-wall groove (no ledges, open bottom)
  -25..-31   pinion (6 face) on the shaft's 7 AF hex, M3x8 button + washer from below into the shaft
  -31.4..-42 jaw webs (open bottom), -42..-47 stub blocks, stub to -61, replica finger tips at ~-106
STOPS (the swap routine needs a SOLID open stop, rotor range < 144 deg):
  open  = rack outer end on the tunnel end wall (s = 22.0 mm per jaw, 126 deg)
  closed= rack inner end on a post in its own lane (s = 0, replica grip faces 0.5 mm apart)
DOCK: jaw parts inside tool z -25..-41 (the tail-rest rib's span) stay at x >= -38.7 at full open (rib top -39.3).
"""
import json
import sys
from math import cos, sin, tan, acos, radians, degrees, pi, sqrt, atan2
from pathlib import Path

from build123d import (Box, Cylinder, Pos, Rot, Polygon, extrude, export_stl, Plane, Axis, chamfer, Location)

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
import build_changer as B      # noqa: E402
import build_changer_v4 as V   # noqa: E402

OUT = H / "v4" / "stl"
# ---------------------------------------------------------------- gear
M, ZT, PA, XSH = 1.25, 16, 20.0, 0.10         # x = +0.1 clears the z16 / 20 deg undercut (rack addendum 1.125 < r sin^2 1.17); z even -> both racks identical
R_P = M * ZT / 2                              # 10.0: rack travel per radian
DELTA = 0.075                                 # flank thinning per flank per part (0.3 mm total backlash, printed)
Y_PITCH = R_P + XSH * M                       # rack pitch line 10.125 from the axis
TIP_TRUNC = 0.2                               # tip topped 0.2: printable 0.86 mm tip land (contact ratio still ~1.6)
RA = R_P + M * (1 + XSH) - TIP_TRUNC          # pinion tip 11.05
RF = R_P - M * (1.25 - XSH)                   # pinion root 8.625
Y_TIP, Y_ROOT = Y_PITCH - M, Y_PITCH + 1.25 * M   # rack 8.94 / 11.75
FACE = (-25.0, -31.0)                         # pinion face z
S_OPEN = 21.0                                 # per jaw (r2: 120 deg leaves ~7 deg of the 144 deg search window for lash)
THETA_DEG = degrees(S_OPEN / R_P)             # 126 deg of rotor for the full stroke
# ---------------------------------------------------------------- body
BX, BY = 39.0, 23.0                           # body half sizes; -x face at -39 rests 0.3 over the dock's tail rib (V.BODY_HALF); 46 wide: cover screws outside the guide skirts (r1)
Z_TOP, Z_CEIL, Z_BOT = -21.0, -23.7, -32.4    # ceiling 0.9 over the racks / 1.3 over the pinion: the 30.5 mm bridge may sag (critic r1)
TUN_X, TUN_Y = 14.0 + 21.0, 15.25             # tunnel half sizes: rack outer end (14) + S_OPEN; end walls 4 mm = the open stops
RACK_Z = (-24.6, -31.4)
RACK_BACK = 15.06
TONGUE_Z, TONGUE_Y = (-26.5, -29.5), 2.0      # rack back tongue (3 x 2), top face 45 deg (faces the bed when the jaw prints)
GROOVE_CLR = 0.25                             # critic r1: 0.15 on two 45-deg printed overhangs is too tight
POST_X, POST_Y0 = (30.0, 34.0), 9.2           # closed-stop post in each rack lane
Z_SHOULDER = -7.2                             # rotor (16.9 tall, bottom -7.0) gets 0.2 axial play
NECK_BORE_R = B.R_SPIG_BORE                   # 9.5
SHAFT_R, SHAFT_BORE_D = 4.0, 8.6              # rule hole.p1s.pla.comp: vertical holes print ~0.2 small -> ~0.2 running (r1: 8.4 too tight)
HEX_AF, HEX_SOCK_AF, HEX_Z = 7.0, 7.2, (-25.0, -30.0)   # r2: 7.3 = 9 deg of lash; the M3 clamp also locks it
PILOT_D, PILOT_L = 2.6, 10.0                  # M3 self-tap (gripper project convention)
TONGUE = dict(L=14.0, W=4.0, z=(Z_SHOULDER, Z_SHOULDER + 3.0))   # rotor_v4 slot 4.2 wide x 8 deep: 0.1/side, 3 mm engaged
# ---------------------------------------------------------------- jaw (jaw 1 = +x side, rack in the +y lane), closed pose
RACK_X = (-30.0, 14.0)                        # rack bar; teeth over x <= 8 (pinion contact at s=0 spans +-6.7)
TEETH_X1 = 8.0
WEB = dict(x=(5.0, 13.8), y=(-8.4, 14.9), z=(-31.4, -47.0), relief_z=-32.0)   # r1: wider joint; face under the pinion 1.0 clear
KEEL = dict(x=(-30.0, 13.8), y=(11.7, 14.9), z=(-31.4, -32.4))  # r1: rack-to-web joint along the whole rack (>= 11.7 from the axis: clear of the 11.18 pinion tip)
# r1 BLOCKING (roll): the lower web is +-14.9 wide and runs between two SKIRTS of a screwed-on bottom COVER (0.25/side over
# z -38.5..-47; the cover is located by snug 3.2 screw holes) so roll about the slide axis is held there and at the rack, ~19 mm apart, instead of by the 3 mm rack tongue
# (~4 deg).  The web faces print as vertical walls (no support on a sliding face); the -y step is a 45-deg chamfer.
WEB_LOW = dict(hy=14.9, z_full=-38.5)
COVER = dict(z_plate=-34.9, z_skirt=-47.0, win_x=36.2, skirt_in=15.05, skirt_out=17.55,   # web 0.15/side; r2: located by snug 3.2 screw holes
             screw=((20.0, 20.8), (-20.0, 20.8), (20.0, -20.8), (-20.0, -20.8)))
BLOCK = dict(y=(-10.0, 10.0), z=(-47.0, -52.0))   # r1 BLOCKING (dock): 5 mm lower -> dock x >= 40, 6 mm past the tail rib's end (34)
# stub: gripper/tools/qc_build.py FIX in the stock Wrist_Roll_Follower frame (n = +X toward the other jaw, u = +Z to the tip)
FIX = dict(cut_z=46.0, x=(-26.5, -16.5), y=(-8.0, 8.0), L=14.0, pin=(-21.5, 53.0))
TAPER, PIN_D = 8.0, 3.7                       # horizontal holes print ~0.3 small (rule hole.p1s.horizontal): 3.7 -> ~3.4
PILOT_H = 2.9                                 # horizontal M3 self-tap pilot -> prints ~2.6
GRIP_FACE_X = -7.9                            # pad face of gt_finger (part frame); = replica_fixed_v2's most proud point
# finger (part frame): socket = the gripper project's A1 socket (modules_build.py: stub + FIT, 15 deep, mouth 1 mm off the
# shoulder, 3.4 clearance on the +Y wall, 2.6 self-tap pilot on the -Y wall, hole 0.25 toward the tip so the screw pulls it on)
FIT, MOUTH_GAP, TIP_GAP, DRAW = 0.05, 1.0, 1.0, 0.25    # r1: FIT 0 is untested and likely an interference; +0.05 clearance, the draw screw pulls it on
GT_SCREW_R = 19.0                             # r1: plate screws / neck inserts at r 19 (r 21 left a 1.15 mm insert wall in the 36 neck)
DRAW_CB = 1.0                                 # counterbore for the M3x20 draw screw head
FING = dict(back=-29.0, hw=10.5, z_tip=100.0, pad_t=6.0, root_z=62.0, taper_to=80.0)
GAP_CLOSED = 1.0                              # r1: 0.5 sat inside the tolerance stack
TX = GAP_CLOSED / 2 + GRIP_FACE_X             # part -> tool: Pos(TX, 0, TZ) * Rot(0, 180, 0)
TZ = BLOCK["z"][1] + FIX["cut_z"]             # stub shoulder at tool z -52
BLOCK["x"] = (14.0 + TX, 29.0 + TX)            # around the stub (part X -29..-14)


def part_to_jaw1():
    return Pos(TX, 0, TZ) * Rot(0, 180, 0)


def jaw_loc(side, s):
    """Location of jaw `side` (+1 / -1) opened by s mm: jaw 2 is jaw 1 turned 180 deg about the tool axis."""
    return (Pos(s, 0, 0) if side > 0 else Rot(0, 0, 180) * Pos(s, 0, 0))


# ---------------------------------------------------------------- gear profiles
def inv(a):
    return tan(a) - a


def pinion_profile(delta=DELTA):
    rb = R_P * cos(radians(PA))
    s_p = M * (pi / 2 + 2 * XSH * tan(radians(PA))) - 2 * delta     # tooth thickness on the reference circle
    psi_p = s_p / 2 / R_P
    pts = []
    n_inv = 10
    for k in range(ZT):
        c = radians(90 + k * 360 / ZT)
        r0 = max(rb, RF)
        flank = []
        for i in range(n_inv + 1):
            rho = r0 + (RA - r0) * i / n_inv
            a = acos(min(1.0, rb / rho))
            flank.append((rho, psi_p + inv(radians(PA)) - inv(a)))
        if RF < rb:
            flank.insert(0, (RF, flank[0][1]))
        left = [(r * cos(c - p), r * sin(c - p)) for r, p in flank]          # one flank up the tooth
        right = [(r * cos(c + p), r * sin(c + p)) for r, p in reversed(flank)]
        pts += left + right
        # root arc to the next tooth
        p0 = c + flank[0][1]
        p1 = radians(90 + (k + 1) * 360 / ZT) - flank[0][1]
        for j in range(1, 4):
            a = p0 + (p1 - p0) * j / 4
            pts.append((RF * cos(a), RF * sin(a)))
    return pts


def tooth_tip_thickness():
    rb = R_P * cos(radians(PA))
    s_p = M * (pi / 2 + 2 * XSH * tan(radians(PA))) - 2 * DELTA
    a = acos(rb / RA)
    return 2 * RA * (s_p / 2 / R_P + inv(radians(PA)) - inv(a))


def pinion():
    z0, z1 = FACE
    prof = pinion_profile()
    g = Pos(0, 0, z1 + 0.3) * extrude(Polygon(*prof, align=None), amount=(z0 - z1) - 0.3)
    g += Pos(0, 0, z1) * extrude(Polygon(*pinion_profile(DELTA + 0.15), align=None), amount=0.3)   # bed layer thinned (elephant foot)
    g -= Pos(0, 0, HEX_Z[1]) * extrude(Polygon(*hexagon(HEX_SOCK_AF), align=None), amount=z0 - HEX_Z[1] + 0.02)
    g -= Pos(0, 0, (z1 + HEX_Z[1]) / 2) * Cylinder(1.7, HEX_Z[1] - z1 + 0.2)
    return g


def hexagon(af):
    r = af / 2 / cos(radians(30))
    return [(r * cos(radians(30 + 60 * i)), r * sin(radians(30 + 60 * i))) for i in range(6)]


def rack_teeth_profile(x0, x1, delta=DELTA):
    """Rack 1 (+y lane, teeth pointing -y): a polygon from y = Y_ROOT (+ the backing beyond) down to the tips.
    Gaps are centred at x = 0 + k*p for s = 0 (a pinion tooth points +y there)."""
    p = pi * M
    ta = tan(radians(PA))
    half_p = (pi * M / 2 - 2 * delta) / 2
    h_tip, h_root = M, 1.25 * M
    pts = [(x0, RACK_BACK), (x0, Y_ROOT)]
    k0 = int((x0 / p) - 2)
    k1 = int((x1 / p) + 2)
    for k in range(k0, k1 + 1):
        xc = (k + 0.5) * p                                   # tooth centre
        if xc - half_p - h_root * ta < x0 or xc + half_p + h_root * ta > x1:
            continue
        pts += [(xc - half_p - h_root * ta, Y_ROOT), (xc - half_p + h_tip * ta, Y_TIP),
                (xc + half_p - h_tip * ta, Y_TIP), (xc + half_p + h_root * ta, Y_ROOT)]
    pts += [(x1, Y_ROOT), (x1, RACK_BACK)]
    return pts


def prism_yz(pts, x0, x1):
    """(y, z) polygon extruded along +x from x0 to x1."""
    return Pos(x0, 0, 0) * extrude(Plane.YZ * Polygon(*pts, align=None), amount=x1 - x0, dir=(1, 0, 0))


def tongue_profile():
    z0, z1 = TONGUE_Z
    y0, y1 = RACK_BACK - 0.01, RACK_BACK + TONGUE_Y
    return [(y0, z0), (y1, z0 - TONGUE_Y), (y1, z1), (y0, z1)]


def groove_profile():
    """The tongue + GROOVE_CLR all round (the 45-deg face offset along its normal)."""
    zs = lambda y: TONGUE_Z[0] + GROOVE_CLR * sqrt(2) - (y - RACK_BACK)
    yw, yo, zb = TUN_Y - 0.01, RACK_BACK + TONGUE_Y + GROOVE_CLR, TONGUE_Z[1] - GROOVE_CLR
    return [(yw, zs(yw)), (yo, zs(yo)), (yo, zb), (yw, zb)]


# ---------------------------------------------------------------- parts
def body():
    n = Pos(0, 0, -5 - B.NECK_H / 2) * Box(B.NECK_SQ, B.NECK_SQ, B.NECK_H)
    n = chamfer(n.edges().filter_by(Axis.Z), 1.5)
    b = n + Pos(0, 0, (Z_TOP + Z_BOT) / 2) * Box(2 * BX, 2 * BY, Z_TOP - Z_BOT)
    # tunnel (open bottom), side grooves, stop posts
    b -= Pos(0, 0, (Z_CEIL + Z_BOT - 1) / 2) * Box(2 * TUN_X, 2 * TUN_Y, Z_CEIL - Z_BOT + 1)
    g = groove_profile()
    for sy in (1, -1):    # groove only where that lane's rack tongue travels (rack 1: x -30..36), so the post corner stays solid
        x0, x1 = (-30.3, TUN_X) if sy > 0 else (-TUN_X, 30.3)
        b -= prism_yz([(y * sy, z) for y, z in g], x0, x1)
    for sx in (1, -1):    # post for rack 1 (+y lane) sits at -x; for rack 2 (-y lane) at +x
        sy = sx
        b += Pos(-sx * sum(POST_X) / 2, sy * (POST_Y0 + TUN_Y + 0.5) / 2, (Z_CEIL + Z_BOT) / 2) * Box(
            POST_X[1] - POST_X[0], TUN_Y + 0.5 - POST_Y0, Z_CEIL - Z_BOT)
    # rotor seat + shaft bearing
    b -= Pos(0, 0, (-5 + Z_SHOULDER) / 2 + 0.01) * Cylinder(NECK_BORE_R, -5 - Z_SHOULDER + 0.02)
    b -= Pos(0, 0, (Z_SHOULDER + Z_CEIL) / 2) * Cylinder(SHAFT_BORE_D / 2, Z_SHOULDER - Z_CEIL + 0.2)
    # 4 M3 heat-set inserts (rule insert.heatset.M3.bore: 4.0, 6.7 deep) for the coupling plate's countersunk screws
    for k in range(4):
        a = radians(45 + 90 * k)
        b -= Pos(GT_SCREW_R * cos(a), GT_SCREW_R * sin(a), -5 - 6.7 / 2 + 0.01) * Cylinder(2.0, 6.7 + 0.02)
        # rule insert.locating_counterbore: 5.2 dia (insert OD 5.0 + 0.2) x 0.4 deep step at the mouth
        b -= Pos(GT_SCREW_R * cos(a), GT_SCREW_R * sin(a), -5 - 0.4 / 2 + 0.01) * Cylinder(2.6, 0.4 + 0.02)
    # dock magnet (N52 6x3): CLOSED drop-in slot behind a 0.4 wall on the neck's -x face, open upward in the print pose (neck up),
    # sealed by the neck above it after the print pause (10/29, iris v2 pocket spec; see build_changer_v4.neck_magnet_slot)
    b -= V.neck_magnet_slot()
    # 4 self-tap pilots (2.6 x 8) in the side-wall undersides for the bottom cover's M3x10 screws (outside the shoe channel)
    for x, y in COVER["screw"]:
        b -= Pos(x, y, Z_BOT + 4.0 - 0.01) * Cylinder(PILOT_D / 2, 8.0)
    return b


def cover():
    """Bottom cover: a frame on the body underside (window +-36.2 x +-15.05 for the jaw webs) with a skirt down each side
    (|y| 15.05..17.55 to z -47) that guides the lower webs; 4x M3x10 button self-tapping into the side walls."""
    zp, zs, zt = COVER["z_plate"], COVER["z_skirt"], Z_BOT
    c = Pos(0, 0, (zp + zt) / 2) * Box(2 * BX, 2 * BY, zt - zp)
    c -= Pos(0, 0, (zp + zt) / 2) * Box(2 * COVER["win_x"], 2 * COVER["skirt_in"], zt - zp + 2)
    for sy in (1, -1):
        c += Pos(0, sy * (COVER["skirt_in"] + COVER["skirt_out"]) / 2, (zp + zs) / 2 + 0.01) * Box(
            2 * BX, COVER["skirt_out"] - COVER["skirt_in"], zp - zs + 0.02)
    for x, y in COVER["screw"]:   # r2: 3.2 holes (print ~3.0-3.1) locate the cover to +-0.1 so the skirts can't shift onto the webs
        c -= Pos(x, y, (zp + zt) / 2) * Cylinder(1.6, zt - zp + 2)
        c -= Pos(x, y, zp + 0.5) * Cylinder(3.3, 1.02)       # 1 mm button-head counterbore (head sits 0.65 proud)
    return c


def shaft():
    t0, t1 = TONGUE["z"]
    t = Pos(0, 0, (t0 + t1) / 2) * Box(TONGUE["L"], TONGUE["W"], t1 - t0)
    t = chamfer(t.edges().group_by(Axis.Z)[-1], 0.4)          # printed tongue-down: elephant-foot relief + slot lead-in
    s = t + Pos(0, 0, (t0 + HEX_Z[0]) / 2) * Cylinder(SHAFT_R, t0 - HEX_Z[0])
    s += Pos(0, 0, HEX_Z[0]) * extrude(Polygon(*hexagon(HEX_AF), align=None), amount=HEX_Z[1] - HEX_Z[0])
    s -= Pos(0, 0, HEX_Z[1] + PILOT_L / 2 - 0.01) * Cylinder(PILOT_D / 2, PILOT_L)
    return s


def stub_part():
    """A1 fixed stub in the part frame: rectangle at z 46, boss 14 toward +z, 8 deg/side inward, 3.4 draw hole along y."""
    x0, x1 = FIX["x"]; y0, y1 = FIX["y"]; L = FIX["L"]
    st = Pos(0, 0, FIX["cut_z"]) * extrude(Polygon((x0, y0), (x1, y0), (x1, y1), (x0, y1), align=None), amount=L, taper=TAPER)
    st -= Pos(FIX["pin"][0], 0, FIX["pin"][1]) * Rot(90, 0, 0) * Cylinder(PIN_D / 2, 30)
    return st


def jaw1():
    """Jaw 1 in its closed pose: rack bar + back tongue + web + stub block + A1 stub."""
    z0, z1 = RACK_Z
    r = Pos(0, 0, z1) * extrude(Polygon(*rack_teeth_profile(RACK_X[0], TEETH_X1), align=None), amount=z0 - z1)
    # plain rack beyond the teeth, down to the tip line (>= 12.0 from the axis at x >= 8: clears the 11.44 tip circle)
    r += Pos((TEETH_X1 + RACK_X[1]) / 2, (Y_TIP + RACK_BACK) / 2, (z0 + z1) / 2) * Box(RACK_X[1] - TEETH_X1, RACK_BACK - Y_TIP, z0 - z1)
    r += prism_yz(tongue_profile(), *RACK_X)
    # 0.4 chamfer on the rack's top back edge (printed top-down: the elephant foot would rub the side wall)
    r -= Pos(sum(RACK_X) / 2, RACK_BACK, z0) * Rot(45, 0, 0) * Box(RACK_X[1] - RACK_X[0] + 1, 0.8, 0.8)
    bx = lambda xs, ys, zs: Pos(sum(xs) / 2, sum(ys) / 2, sum(zs) / 2) * Box(xs[1] - xs[0], ys[1] - ys[0], zs[0] - zs[1])
    w = bx(WEB["x"], (Y_TIP, WEB["y"][1]), WEB["z"]) + bx(WEB["x"], WEB["y"], (WEB["relief_z"], WEB["z"][1]))
    w += bx(KEEL["x"], KEEL["y"], KEEL["z"])
    # lower web +-14.9 between the cover skirts; the -y widening is a 45-deg chamfer (prints support-free, clears keel 2)
    zr, zf = WEB["relief_z"], WEB_LOW["z_full"]
    hy = WEB_LOW["hy"]
    w += bx(WEB["x"], (-hy, WEB["y"][1]), (zf, WEB["z"][1]))
    w += prism_yz([(WEB["y"][0] + 0.01, zr), (WEB["y"][0] + 0.01, zf), (-hy, zf), (-hy, zf + 0.01)], *WEB["x"])
    bl = Pos(sum(BLOCK["x"]) / 2, 0, sum(BLOCK["z"]) / 2) * Box(BLOCK["x"][1] - BLOCK["x"][0], BLOCK["y"][1] - BLOCK["y"][0], BLOCK["z"][0] - BLOCK["z"][1])
    j = r + w + bl + part_to_jaw1() * stub_part()
    return j


def rotor():
    return V.rotor_v4()


def coupling_plate():
    """build_changer's coupling plate with its 4 countersunk screws at GT_SCREW_R (matches this tool's neck inserts)."""
    old = B.PLATE_SCREW_R
    B.PLATE_SCREW_R = GT_SCREW_R
    try:
        return B.coupling_plate()
    finally:
        B.PLATE_SCREW_R = old


def finger_part():
    """Flat parallel pad finger on the A1 stub (part frame: X toward the other jaw, Z to the tip).  The stock-shape replica
    has an inclined grip face: on a parallel gripper it only touched at the tips (render 10/25), so this one is flat.
    Side profile (XZ): full socket block to Z 62, back face tapering to a 6 mm pad plate at Z 80, tip at Z 100."""
    z0 = FIX["cut_z"] + MOUTH_GAP
    b, g, t, zt = FING["back"], GRIP_FACE_X, FING["pad_t"], FING["z_tip"]
    prof = [(b, z0), (g, z0), (g, zt - 1.0), (g - 1.0, zt), (g - t, zt), (g - t, FING["taper_to"]), (b, FING["root_z"])]
    f = Pos(0, FING["hw"], 0) * extrude(Plane.XZ * Polygon(*prof, align=None), amount=2 * FING["hw"])
    # socket: the stub (+FIT) run TIP_GAP past its tip, from the shoulder plane; 0.5 x 45 entry chamfer (elephant foot)
    x0, x1 = FIX["x"][0] - FIT, FIX["x"][1] + FIT
    y0, y1 = FIX["y"][0] - FIT, FIX["y"][1] + FIT
    f -= Pos(0, 0, FIX["cut_z"]) * extrude(Polygon((x0, y0), (x1, y0), (x1, y1), (x0, y1), align=None),
                                           amount=FIX["L"] + TIP_GAP, taper=TAPER)
    f -= Pos(0, 0, z0 - 0.01) * extrude(Polygon((x0 - 0.5, y0 - 0.5), (x1 + 0.5, y0 - 0.5), (x1 + 0.5, y1 + 0.5),
                                                 (x0 - 0.5, y1 + 0.5), align=None), amount=0.5, taper=45)
    zp = FIX["pin"][1] + DRAW
    f -= Pos(FIX["pin"][0], 0, zp) * Rot(90, 0, 0) * Cylinder(PILOT_H / 2, 2 * FING["hw"] + 2)
    f -= Pos(FIX["pin"][0], FING["hw"] / 2 + 0.5, zp) * Rot(90, 0, 0) * Cylinder(PIN_D / 2, FING["hw"] + 1)
    # 1 mm counterbore on the +Y face: an M3x20 head seats at Y 9.5, its tip reaches the far face (3.5 mm in the 2.6 pilot)
    f -= Pos(FIX["pin"][0], FING["hw"] - DRAW_CB / 2 + 0.01, zp) * Rot(90, 0, 0) * Cylinder(3.3, DRAW_CB)
    # grip texture: 0.4 deep x 0.8 grooves across the pad every 2 mm (Z 50..96)
    zz = 50.0
    while zz < 96.0:
        f -= Pos(g, 0, zz) * Box(0.8, 2 * FING["hw"] + 2, 0.8)
        zz += 2.0
    return f


def replica_module_mesh():
    """The finger the checks / renders mount on the stubs (gt_finger.stl, part frame)."""
    return OUT / "gt_finger.stl"


def report():
    return {
        "gear": {"module": M, "teeth": ZT, "pressure_angle_deg": PA, "profile_shift": XSH, "pitch_r_mm": R_P,
                 "tip_r": round(RA, 3), "root_r": round(RF, 3), "rack_pitch_y": Y_PITCH, "rack_tip_y": round(Y_TIP, 3),
                 "rack_root_y": round(Y_ROOT, 3), "backlash_total_mm": 4 * DELTA,
                 "pinion_tip_thickness_mm": round(tooth_tip_thickness(), 2)},
        "stroke": {"per_jaw_mm": S_OPEN, "rotor_deg": round(THETA_DEG, 1), "gap_closed_mm": GAP_CLOSED,
                   "gap_open_mm": GAP_CLOSED + 2 * S_OPEN},
        "grip_force_per_jaw_N": {f"{t:.2f}Nm": round(0.85 * t / (2 * R_P / 1000), 1) for t in (0.05, 0.15, 0.25)},
        "tx_tz": [TX, TZ],
    }


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    parts = {"gt_body": body(), "gt_cover": cover(), "gt_shaft": shaft(), "gt_pinion": pinion(), "gt_jaw1": jaw1(),
             "gt_rotor": rotor(), "gt_plate": coupling_plate(), "gt_pins": B.pins()}
    parts["gt_jaw2"] = Rot(0, 0, 180) * parts["gt_jaw1"]
    parts["gt_finger"] = finger_part()                     # part frame (print pose: socket mouth on the bed)
    info = {"parts": {}, **report()}
    for name, s in parts.items():
        export_stl(s, str(OUT / f"{name}.stl"), tolerance=0.005, angular_tolerance=0.05)
        bb = s.bounding_box()
        info["parts"][name] = {"cm3": round(s.volume / 1000, 3), "valid": s.is_valid,
                               "bbox": [round(v, 2) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)]}
    (H / "v4" / "gripper_tool.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()
