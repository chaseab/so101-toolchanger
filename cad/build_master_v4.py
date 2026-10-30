# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Arm-side MASTER parts v4 (hub, cage, blade carrier, TPU back pad) for the SO-101 tool changer.  2026-10-24, agent
toolchanger-master (sub-agent of toolchanger-build).  Imports build_changer (v3.1) and overrides only what had to change.

Run:  python toolchanger/cad/build_master_v4.py
Writes v4/stl/master_{hub,cage,carrier}_v4.stl, v4/stl/master_backpad_TPU_v4.stl, v4/gate_parts/sts3215_proxy_v4.stl,
v4/master_v4.json (numbers + checks).

FRAME: the v3.1 master frame (build_changer), so the printed ring v4p / plungers / skirt drop in unchanged.  The ONLY frame
change: the hub is now HUB_T_V4 = 14.5 thick instead of 6, so on the arm the roll-horn plane sits at z = +MASTER_DROP (8.5)
in this frame.  Every master/tool z below the hub is 8.5 mm farther from the wrist than v3.1 says (sim/swap_io use
Z_TOOL_FACE_LOCK etc.; add MASTER_DROP there if the sim is re-run; the real swap is taught, so it absorbs it).

WHY v3.1 could not go on the arm as drawn (all from the official meshes in dashboard/meshes + the URDF; see master_v4.json):
 1. CABLE: the STS3215 sockets are on the servo BACK (the face under the hub), 14.5 mm from the output axis toward the far
    end; the plug goes in straight down onto that face.  The stock Wrist_Roll_Follower leaves exactly that spot open.
    v3.1 covered the whole back with the gasket and put the cable exit on the +x end, so the servo could not be plugged in.
 2. WRIST LINK: the wrist_roll_pitch part reaches 4.7 mm BELOW the roll-horn plane at r >= ~17 mm, and its side plates
    reach to ~0.5 mm below it at r >= 10.4.  Anything that turns with the roll must stay inside r ~17 for the top 5.5 mm,
    and the plug + its wire must stay below that.  The v3.1 hub (24 x 42 x 6, ears at r 17.5 up to the horn plane) swept
    through that tab over ~200 deg of roll.
 3. REAR BOSS: the servo back has a boss on the output axis (mesh: r 2.6, ~4 mm proud).  v3.1 pressed the gasket onto it.
Fixes (after 2 sonnet critic rounds, design-reviews/toolchanger-master-arm-2026-10-24.md): hub = round stepped flange
(r 13 for the top 2.5 mm, r 14.6 to 5.5 mm: >= 1.1 mm from the link at every roll angle) on a 9 mm plate that carries the
2 cage inserts + 4 key pockets below the link; boss relief; plug well beside the hub; cage ears widened with 2 torque keys
each; servo modelled from the OFFICIAL mesh (it stands on its proud front strip, horn face 1.7 below the floor top); TPU
back pad (near end, 0 squeeze) + PLA STRAP with a TPU shim over the far end (2x M3x8 into cage-boss inserts) hold it
down; 3 radial floor ribs at the spring angles; ring screws moved beside each spring (ring v4q, build_ring_v4q.py);
carrier pilot boss -> clearance pocket for the horn's centre screw head; lower cable tab removed (45 mm dock rule).

HEAT INSERTS (owner 10/24): every screw that carries the spring load or the master goes into brass or a metal horn:
  ring <- cage floor: 3x M3x8 button into ring v4q top-face inserts, one 15 deg beside each spring;
  strap -> cage: 2x M3x8 button into cage-boss inserts;
  cage -> hub: 2x M3x40 SHCS up through the cage ears into 2 hub heat-set inserts (bores now L+1 deep, 2.0 mm walls);
  hub -> roll horn: 4x M3x6 button into the roll servo horn;  carrier -> gripper horn: 4x M3x5 (or x4) button.
Nothing threads into bare plastic.

=== CALIPER INPUTS (owner: measure on the gripper STS3215, change here, re-run this file, then re-slice + re-gate) ===
"""
import json
import sys
from math import cos, sin, radians, hypot
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot, Polygon, extrude, export_stl, chamfer, Axis, Plane

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
import build_changer as B  # noqa: E402
import build_changer_v4 as V  # noqa: E402
from build_ring_v4q import RING_SCREWS_V4Q  # noqa: E402  (screw beside each spring, owner 10/24)

# (owner 10/24: no caliper pass first; these are the OFFICIAL-MESH values, fits kept forgiving.)
# M1  horn outer face below the face the servo stands on.  The output face is not flat: a strip |y| < 7 is 1.1 proud of
#     the rims and the horn end is stepped 2.6 down; on a flat floor the servo stands on the strip, horn face 1.7 below
#     the floor top (critic r1).  v3.1 used 4.2 (from the stepped face) -> the carrier would hang 2.5 below the horn.
#     UNUSABLE-IF-WRONG for the carrier only (10-min reprint): too small = screws barely reach, too big = blade bottoms.
HORN_FACE_DROP = 1.7
# M2  centre screw head of the gripper horn, proud of the horn face (mesh 1.5) and its head radius (mesh 2.7).
HORN_SCREW_PROUD, HORN_SCREW_R = 1.5, 2.7
HORN_DISC_T = 2.5      # horn disc thickness (mesh 2.5); the gap behind it (mesh 1.7) takes the screw tips
# M3  servo height from the face it stands on to its FLAT BACK under the axis (not the boss, not the plug pins).
#     Sets only the TPU back pad (mesh strip -> flat back plateau 31.4; v3.1 assumed 35.35).  Wrong -> loose servo or a
#     pad too thick to close; the pad is the cheap adjustable part (TPU, ~15 min).
SERVO_BACK_D = 31.4
# M4  rear boss on the output axis: radius, height above the back (mesh 2.6 / 4.0).  If a REAR IDLER HORN (20 mm disc) is
#     fitted on the back, take it off: the hub has to bear on the case there.
REAR_BOSS_R, REAR_BOSS_H = 2.6, 4.0
# M5  plug: centre of the cable socket from the output axis (toward the far end, on the back face), plug height above the
#     back incl. the latch, and room to bend the wire flat.  Mesh pins at 14.5; plug height is a guess.
SOCKET_X, PLUG_H, WIRE_BEND = 14.5, 7.0, 2.5
# ======================================================================================================================

# ---- wrist link keep-out (URDF + wrist_roll_pitch mesh, gripper frame with the horn plane at 0) ----
LINK_TAB_Z = -4.7              # lowest point of the link at r >= ~17 (mesh); 0.8 mm margin below it
LINK_CLEAR_Z = -5.5
STOCK_TOP_HALF = 12.0          # half width of the plate's -x end (below the link keep-out)
# critic r1: the link's inner radius from the roll axis is 14.1 at real z -0.1, 15.1 at -2, 15.7 at -3, 16.8 at -4.6 (stock
# follower max r 12.6).  The flange above the plate is therefore round and stepped: r 13.0 for the top 2.5 mm, r 14.6 below.
FLANGE_R = ((2.5, 13.0), (5.5, 14.6))   # (depth below the horn plane, radius)
# ---- hub ----
GASKET_MIN = 0.6               # thinnest back pad
HUB_T_V4 = round(-LINK_CLEAR_Z + PLUG_H + WIRE_BEND - GASKET_MIN - 0.0, 2)   # 14.4 -> rounded up below
HUB_T_V4 = max(HUB_T_V4, 14.5)
MASTER_DROP = HUB_T_V4 - B.HUB_T                                             # 8.5
Z_HORN_PLANE = MASTER_DROP                                                   # horn plane in this (v3.1) frame
Z_FLANGE_BOT = Z_HORN_PLANE + LINK_CLEAR_Z                                   # +3.0: plate below, 24 sq flange above
HUB_PLATE_Y = 21.5             # insert at 17.5 + bore 2.0 -> 2.0 mm wall (rule insert.heatset.wall_min 1.6)
HUB_PLATE_XMAX = 10.5          # plug well beside the hub: plug spans ~x 12..17
HORN_SCREW_GRIP = 2.4          # hub under the M3 head (v3.1 / camarm 2.8): M3x6 -> 3.6 past the hub, through the 2.5 roll horn, tip 1.1 past it (mesh gap 1.7)
WELL_R = 3.4                   # 6.8: button head 5.7 + 0.55/side; 0.87 mm web to the r 2.8 centre hole (critic r1: 3.4/3.2 left 0.47)
CENTRE_R = 2.8                 # roll-horn centre screw head r 2.7 (mesh); = v3.1 / printed camarm mount
KEY_X, KEY_W, KEY_H, KEY_CLR = 5.0, 2.0, 2.0, 0.2   # 2 keys on each cage ear top -> pockets in the hub bottom (roll torque, critic r1 #8)
EAR_X_HALF, EAR_Y_OUT = 6.5, 21.0  # cage ears widened from 9 x 5.5 (1.0 mm walls round the screw) to 13 wide, out to y 21
INSERT_D, INSERT_DEPTH = 4.0, 6.7    # same 4.0 bore as ring v4p (the owner's FFVRVSS inserts went in there); depth L+1
BOSS_RELIEF_R, BOSS_RELIEF_H = REAR_BOSS_R + 1.9, REAR_BOSS_H + 1.0
# ---- cage ----
CB_R_V4 = 3.4                  # hub-screw counterbore: SHCS head 5.5 + 0.65/side (v3.1 3.2 = 0.45)
INS_RELIEF_R, INS_RELIEF_H = 2.75, 0.4   # rule insert.heatset.flush_ring: the ring inserts' melt ring must not lift the floor
# ---- carrier ----
CAR_FL_R4 = 11.6               # floor hole r 12.5 -> 0.9 clear if the horn face sits inside the floor (M1 small)
CAR_CB_R4 = 3.35               # button head 5.7 -> 0.5/side (v3.1 3.0 = 0.15); 1.2 mm rim at r 11.6
CAR_POCKET_R, CAR_POCKET_H = HORN_SCREW_R + 0.9, HORN_SCREW_PROUD + 0.5
Z_HORN_BOT4 = B.Z_CASE_OUT - HORN_FACE_DROP
# ---- TPU back pad ----
PAD_SQUEEZE = 0.0              # critic r2: the servo stands on its front strip (x 10..34); the near-end pad only stops lift-off
# ---- far-end hold-down (critic r2 #1): PLA strap over the servo's raised rear cover, 2x M3x8 into cage-boss inserts ----
STRAP_X, STRAP_W, STRAP_T, STRAP_Y = 25.0, 10.0, 5.0, 22.6
SOCKET_Y, CHAN_W, CHAN_D = 5.3, 5.0, 2.5     # cable channel over the strap (critic r3)
BOSS_Y, BOSS_W = 18.6, 8.0                  # insert at y 18.6: 2.0 walls in an 8 x 7.6 boss on the cage wall
COVER_TOP_Z = B.Z_CASE_OUT + 34.7           # mesh: raised rear cover (x 18.5..29.5, |y| < 9) = -7.25
FARPAD_SQUEEZE = 0.15
FARPAD_T = round(-B.HUB_T - COVER_TOP_Z + FARPAD_SQUEEZE, 2)   # 1.4 TPU, ribbed
HORN_HOLE_R = 13.5                          # critic r2 #4: floor horn hole r 12.5 -> 13.5 (1.9 clear round the carrier)
CAV_HALF_Y = 12.8                           # critic r2 #4: cage inner 25.2 -> 25.6 wide; crush rib re-added
PAD_XMAX = 5.0                 # pad stops before the raised rear cover (mesh x > 5.5 is 1.5 mm higher)
PAD_T = round(B.CAV_H - SERVO_BACK_D + PAD_SQUEEZE, 2)   # CAV_H = hub bottom to cage floor (35.95)
PAD_BASE, RIB_W, RIB_PITCH = 0.8, 2.0, 4.0                # ribbed: 50 % contact; 2 mm ribs so a ~4 mm pad does not buckle


def hub_v4():
    """Stock-size 24 x 24 flange (top 5.5 mm, inside the wrist-link keep-out) on a 22.5 x 43 plate that carries the 2 cage
    inserts below the link.  4x M3 button from below into the roll horn through deep wells (driven on the bench, before
    the servo).  Print bottom-down (horn face up)."""
    z0, z1, zt = -B.HUB_T, Z_FLANGE_BOT, Z_HORN_PLANE
    xlo = -STOCK_TOP_HALF
    plate = Pos((xlo + HUB_PLATE_XMAX) / 2, 0, (z0 + z1) / 2) * Box(HUB_PLATE_XMAX - xlo, 2 * HUB_PLATE_Y, z1 - z0)
    plate = chamfer(plate.edges().filter_by(Axis.Z), 1.0)
    b = plate
    d0 = 0.0
    for d1, r in FLANGE_R:
        b += Pos(0, 0, zt - (d0 + d1) / 2) * Cylinder(r, d1 - d0)
        d0 = d1
    z_seat = zt - HORN_SCREW_GRIP
    for sx in (-5, 5):
        for sy in (-5, 5):
            b -= Pos(sx, sy, (z_seat + zt) / 2) * Cylinder(B.HOLE_R, zt - z_seat + 2)
            b -= Pos(sx, sy, (z0 + z_seat) / 2 - 0.5) * Cylinder(WELL_R, z_seat - z0 + 1)
    b -= Pos(0, 0, (z0 + zt) / 2) * Cylinder(CENTRE_R, zt - z0 + 2)                # roll-horn centre screw head (mesh r 2.7; v3.1 2.8)
    b -= Pos(0, 0, z0 + BOSS_RELIEF_H / 2 - 0.5) * Cylinder(BOSS_RELIEF_R, BOSS_RELIEF_H + 1)
    for sy in (-17.5, 17.5):                                                       # cage-screw inserts, mouth down
        b -= Pos(0, sy, z0 + INSERT_DEPTH / 2 - 0.5) * Cylinder(INSERT_D / 2, INSERT_DEPTH + 1)
        b -= Pos(0, sy, z0 + 0.2 - 0.5) * Cylinder(INSERT_D / 2 + 0.4, 1.4)      # flush-ring relief 0.4 x 0.4 at the mouth (critic r1 #5)
        for sx in (-KEY_X, KEY_X):                                                 # key pockets for the cage-ear keys
            b -= Pos(sx, sy + (0.25 if sy > 0 else -0.25), z0 + (KEY_H + 0.4) / 2 - 0.5) * Box(KEY_W + 2 * KEY_CLR, 4.5 + 2 * KEY_CLR, KEY_H + 0.4 + 1)
    return b


def cage_v4():
    """v3.1 cage with: 6.8 counterbores for the M3x40 heads, 0.4 insert-ring reliefs under the floor at the 3 ring screws."""
    c = B.cage()
    zb, zt = B.Z_RING_TOP, -B.HUB_T
    x1, w, y1 = B.SV_L - B.SV_SHAFT_OFF + 0.2, 2.4, B.SV_W / 2 + 0.2
    c += Pos(x1 + w / 2, 0, zt - 6) * Box(w, 10.02, 6.02)                  # close the v3.1 +x cable slot (cable leaves from the back)
    for sy in (-1, 1):                                                     # wider ears (13 x out to y 21) + 2 torque keys on top
        yi = y1 + w - 0.5
        c += Pos(0, sy * (yi + EAR_Y_OUT) / 2, (zt + zb) / 2) * Box(2 * EAR_X_HALF, EAR_Y_OUT - yi, zt - zb)
        for sx in (-KEY_X, KEY_X):
            c += Pos(sx, sy * 17.75, zt + KEY_H / 2 - 0.01) * Box(KEY_W, 4.5, KEY_H)
        c -= Pos(0, sy * 17.5, (zt + zb) / 2) * Cylinder(1.75, zt - zb + 4)
    for a in V.PL_ANG4:                                                    # floor stiffeners over the 3 spring seats (critic r1 #4)
        r0 = {75.0: 15.8, 195.0: 13.6, 315.0: 21.6}[a]                       # start at the cage box's outer wall
        c += Rot(0, 0, a) * Pos((r0 + 33.0) / 2, 0, B.Z_CASE_OUT + 2.0 - 0.01) * Box(33.0 - r0, 3.0, 4.0)
    for sx in (5, 25):                     # 45-deg gussets under the 2 cable-clip tabs, beside the zip-tie slot (prints unsupported)
        zbot = zt - 14 - sx / 2 - 1.5
        c += B.radial_prism([(-(x1 + w), zbot + 0.01), (-(x1 + w + 4), zbot + 0.01), (-(x1 + w), zbot - 4)], 4.0, 6.0, 90)
        c += B.radial_prism([(x1 + w, zbot + 0.01), (x1 + w + 4, zbot + 0.01), (x1 + w, zbot - 4)], 4.0, 6.0, -90)
    zp = zt - 22                           # 45-deg gusset under the wrist-cam pad
    c += B.radial_prism([(-(y1 + w), zp + 0.01), (-(y1 + w + 4), zp + 0.01), (-(y1 + w), zp - 4)], 8.0, 34.0, 0)
    for sy in (-17.5, 17.5):
        c -= Pos(0, sy, zb + 1.6) * Cylinder(CB_R_V4, 3.2)
    # ring screws: v4q pattern (r 30 at 90/180/300, one beside each spring) + the v4p holes (r 31 at 90/215/270) so the
    # bench ring v4p can still bolt on with 2 screws (215/270; its 90 insert is 1 mm off the new r 30 hole, so that v4p hole
    # is not cut: a slot fails the 0.3 mm hole-alignment check).  0.4 relief under every insert mouth.
    c += Pos(0, B.RING_SCREW_R, zb + B.FLOOR_T / 2) * Cylinder(1.76, B.FLOOR_T - 0.02)    # refill the v3.1 r 31 / 90 hole
    holes = [(r, a) for r, a in RING_SCREWS_V4Q] + [(B.RING_SCREW_R, a) for a in B.RING_SCREW_ANG if a != 90.0]
    for r, a in holes:
        x, y = r * cos(radians(a)), r * sin(radians(a))
        c -= Pos(x, y, zb + B.FLOOR_T / 2) * Cylinder(1.75, B.FLOOR_T + 2)
        c -= Pos(x, y, zb + INS_RELIEF_H / 2 - 0.01) * Cylinder(INS_RELIEF_R, INS_RELIEF_H + 0.02)
    # critic r2: wider cavity (25.6), bigger horn hole, lower clip tab removed (it sat < 45 mm behind the dock wall face)
    c -= Pos((x1 - 0.2 + (-B.SV_SHAFT_OFF - 0.2)) / 2 + 0.1, 0, (zt + B.Z_CASE_OUT) / 2 + 0.5) * Box(B.SV_L + 0.4, 2 * CAV_HALF_Y, zt - B.Z_CASE_OUT + 1)
    c += Pos(10, CAV_HALF_Y - 0.35, (zt + B.Z_CASE_OUT) / 2) * Box(6, 0.7, B.CAV_H - 4)     # +y crush rib (0.3 into the case)
    c += Pos(x1 - 0.25, 0, (zt + B.Z_CASE_OUT) / 2) * Box(0.5, 6, B.CAV_H - 4)              # +x crush rib (v3.1, re-added)
    c -= Pos(0, 0, zb + B.FLOOR_T / 2) * Cylinder(HORN_HOLE_R, B.FLOOR_T + 2)
    c -= Pos(x1 + w + 3, 0, zt - 14 - 25 / 2 - 2.5) * Box(6.0, 14, 9.0)
    for sy in (-1, 1):                     # strap bosses + 45-deg gussets, insert from the top
        yo = sy * (y1 + w)
        c += Pos(STRAP_X, sy * (y1 + w + BOSS_Y + BOSS_W / 2 - (y1 + w)) / 1 * 0 + sy * ((y1 + w) + (STRAP_Y - (y1 + w)) / 2), zt - 4.5) * Box(BOSS_W, STRAP_Y - (y1 + w), 9.0)
        tri = [(sy * (y1 + w), zt - 9.0 + 0.01), (sy * STRAP_Y, zt - 9.0 + 0.01), (sy * (y1 + w), zt - 9.0 - (STRAP_Y - (y1 + w)))]
        c += B.radial_prism(tri, STRAP_X - BOSS_W / 2, STRAP_X + BOSS_W / 2, 0)
        c -= Pos(STRAP_X, sy * BOSS_Y, zt - INSERT_DEPTH / 2 + 0.5) * Cylinder(INSERT_D / 2, INSERT_DEPTH + 1)
        c -= Pos(STRAP_X, sy * BOSS_Y, zt - 0.2 + 0.25) * Cylinder(INSERT_D / 2 + 0.4, 0.9)
    return c


def strap_v4():
    """PLA hold-down strap across the cage top over the servo's raised rear cover (x 20..30).  2x M3x8 button into the cage
    boss inserts.  Keeps x 11..17 (the plugs) clear; the cable passes over it.  Print flat, underside down."""
    zt = -B.HUB_T
    s = Pos(STRAP_X, 0, zt + STRAP_T / 2) * Box(STRAP_W, 2 * STRAP_Y, STRAP_T)
    for sy in (-1, 1):
        s -= Pos(STRAP_X, sy * BOSS_Y, zt + STRAP_T / 2) * Cylinder(1.75, STRAP_T + 2)
        s -= Pos(STRAP_X, sy * BOSS_Y, zt + STRAP_T - 1.0 + 0.5) * Cylinder(3.4, 2.0 + 1)
    # critic r3 (c): cable channel along x over the plug socket (y 5.3): 5.0 wide x 2.5 deep, 0.8 chamfered entry edges.
    # The wire lies on its floor (real z -12.0 = just above the plug top -12.05) -> wire top ~ -10.5, 1 mm under the strap top.
    s -= Pos(STRAP_X, SOCKET_Y, zt + STRAP_T - CHAN_D / 2 + 0.5) * Box(STRAP_W + 2, CHAN_W, CHAN_D + 1)
    return s


def farpad_tpu():
    """TPU shim between the strap and the raised rear cover (ribbed, 0.15 squeeze)."""
    zt = -B.HUB_T
    x0, x1, y1 = 19.5, 28.5, 8.0
    p = Pos((x0 + x1) / 2, 0, zt - 0.3) * Box(x1 - x0, 2 * y1, 0.6)
    x = x0 + 0.75
    while x <= x1 - 0.75 + 1e-6:
        p += Pos(x, 0, zt - 0.6 - (FARPAD_T - 0.6) / 2) * Box(1.5, 2 * y1, FARPAD_T - 0.6)
        x += 3.0
    return p


def carrier_v4():
    """v3.1 carrier: flange from the measured horn face, no pilot boss (a clearance pocket for the horn's centre screw
    head instead; the 4 screws centre it), body and blade tip at the same absolute z as v3.1."""
    zf = Z_HORN_BOT4
    c = Pos(0, 0, zf - B.CAR_FL_T / 2) * Cylinder(CAR_FL_R4, B.CAR_FL_T)
    c += Pos(0, 0, (zf - B.CAR_FL_T + B.Z_CAR_BOT) / 2) * Cylinder(B.CAR_BODY_R, zf - B.CAR_FL_T - B.Z_CAR_BOT)
    c -= Pos(0, 0, zf - CAR_POCKET_H / 2 + 0.01) * Cylinder(CAR_POCKET_R, CAR_POCKET_H + 0.02)
    for sx in (-5, 5):
        for sy in (-5, 5):
            c -= Pos(sx, sy, zf - 3) * Cylinder(1.75, 8)
            c -= Pos(sx, sy, zf - B.CAR_FL_T - 3 + 1.7) * Cylinder(CAR_CB_R4, 6)
    bl = Pos(0, 0, (B.Z_CAR_BOT + B.Z_BLADE_TIP) / 2) * Box(B.BLADE_T, B.BLADE_W, B.Z_CAR_BOT - B.Z_BLADE_TIP)
    bl = chamfer(bl.edges().group_by(Axis.Z)[0].filter_by(Axis.Y), 1.1)
    return c + bl


def backpad_tpu():
    """TPU pad between the hub bottom and the servo's flat back, under the hub plate only (the plug well stays open),
    hole for the rear boss; ribs along y on a 0.4 base so a 0.1 mm squeeze stays soft.  Print base down."""
    x0, x1 = -B.SV_SHAFT_OFF + 0.5, PAD_XMAX         # mesh: the back steps 1.5 mm up beyond x ~5.5 (rear cover)
    y1 = B.SV_W / 2 - 0.5
    ztop = -B.HUB_T
    p = Pos((x0 + x1) / 2, 0, ztop - PAD_BASE / 2) * Box(x1 - x0, 2 * y1, PAD_BASE)
    if PAD_T > PAD_BASE + 0.2:
        x = x0 + RIB_W / 2
        while x <= x1 - RIB_W / 2 + 1e-6:
            p += Pos(x, 0, ztop - PAD_BASE - (PAD_T - PAD_BASE) / 2) * Box(RIB_W, 2 * y1, PAD_T - PAD_BASE)
            x += RIB_PITCH
    p -= Pos(0, 0, ztop - PAD_T / 2) * Cylinder(BOSS_RELIEF_R, PAD_T + 2)
    return p


MESH = H.parents[1] / "dashboard" / "meshes" / "sts3215_03a_no_horn_v1.stl"   # official STS3215 (front horn on, no rear horn)
MESH_AXIS_X, MESH_STRIP_Z = 12.5, 17.0                                          # output axis x, proud front strip z (mm, mesh)


def servo_proxy_mesh():
    """The OFFICIAL STS3215 mesh in the master frame (gripper frame after to_gripper): output down on the roll axis,
    standing on its front strip on the cage floor, far end toward +x; plus one plug on the back socket (cable side)."""
    import trimesh
    m = trimesh.load(MESH); m.apply_scale(1000.0)
    T = trimesh.transformations.rotation_matrix(3.141592653589793, [0, 1, 0])     # x -> -x, z -> -z (proper rotation)
    m.apply_transform(T)
    m.apply_translation([MESH_AXIS_X + B.HORN_CX, 0.0, B.Z_CASE_OUT + MESH_STRIP_Z])
    zb = B.Z_CASE_OUT + SERVO_BACK_D
    plug = trimesh.creation.box(extents=[4.6, 9.0, PLUG_H])
    plug.apply_translation([SOCKET_X + B.HORN_CX, 5.3, zb + PLUG_H / 2])
    return trimesh.util.concatenate([m, plug])


def roll_horn_proxy():
    """Gate stand-in for the ROLL servo's disc horn above the hub (same STS3215 horn): 2.5 mm disc, 4 tapped M3 holes,
    centre screw head 1.5 mm proud into the hub's centre hole."""
    z = Z_HORN_PLANE
    h = Pos(0, 0, z + HORN_DISC_T / 2) * Cylinder(B.HORN_R, HORN_DISC_T)
    for sx in (-5, 5):
        for sy in (-5, 5):
            h -= Pos(sx, sy, z + HORN_DISC_T / 2) * Cylinder(1.25, HORN_DISC_T + 0.2)
    h += Pos(0, 0, z - HORN_SCREW_PROUD / 2) * Cylinder(HORN_SCREW_R, HORN_SCREW_PROUD)
    return h


def checks(parts):
    """Overlaps between neighbours (mm3), link keep-out sweep, spring seat, screw lengths."""
    def ov(a, b):
        try:
            return round((parts[a] & parts[b]).volume, 3)
        except Exception as e:  # noqa
            return f"err {e}"
    out = {"overlap_mm3": {f"{a}|{b}": ov(a, b) for a, b in (
        ("hub", "cage"), ("hub", "pad"), ("cage", "ring"), ("strap", "cage"), ("strap", "hub"), ("farpad", "cage"), ("carrier", "cage"), ("carrier", "ring"), ("hub", "ring"))}}
    # wrist-link keep-out: everything that turns with the roll, above (real) z -5.5 must be within the stock 24 x 24 square
    zc = Z_HORN_PLANE + LINK_CLEAR_Z
    probe = Pos(0, 0, (zc + Z_HORN_PLANE + 20) / 2) * Box(200, 200, Z_HORN_PLANE + 20 - zc)
    above = parts["hub"] & probe
    bb = above.bounding_box()
    out["link_keepout"] = {"above_real_z_-5.5_bbox_local": [round(v, 2) for v in (bb.min.X, bb.min.Y, bb.max.X, bb.max.Y)],
                           "max_r_from_roll_axis": round(max(hypot(x, y) for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y)), 2),
                           "stock_follower_top_max_r": round(hypot(12.5, 12.0), 2)}
    plug_top_real = B.Z_CASE_OUT + SERVO_BACK_D + PLUG_H + WIRE_BEND - Z_HORN_PLANE
    out["plug_wire_top_real_z"] = round(plug_top_real, 2)
    out["plug_vs_hub_plate_x_gap"] = round(SOCKET_X - 2.3 - HUB_PLATE_XMAX, 2)
    out["spring"] = {"cage_floor_underside_z": round(B.Z_RING_TOP, 3), "ring_top_z": round(B.Z_RING_TOP, 3),
                     "plunger_head_top_z": round(B.Z_LIP_BOT + V.PL_GUIDE4 + B.PL_HEAD_H, 3),
                     "installed_mm": round(B.Z_RING_TOP - (B.Z_LIP_BOT + V.PL_GUIDE4 + B.PL_HEAD_H), 3)}
    # floor solid where the 3 springs bear (r 29 +- 2.5 at 75/195/315)
    sol = []
    for a in V.PL_ANG4:
        x, y = B.R_PL * cos(radians(a)), B.R_PL * sin(radians(a))
        disk = Pos(x, y, B.Z_RING_TOP + 0.25) * Cylinder(2.5, 0.5)
        sol.append(round((parts["cage"] & disk).volume / disk.volume, 3))
    out["spring_seat_solid_fraction"] = sol
    grip_cage = (B.HUB_T * 0 + (-B.HUB_T) - B.Z_RING_TOP) - 3.2
    out["screws"] = {"cage_hub_grip": round(grip_cage, 2), "M3x40_into_insert": round(40 - grip_cage, 2),
                     "hub_insert_depth": INSERT_DEPTH, "ring_screw_grip": B.FLOOR_T, "M3x8_into_ring_insert": 8 - B.FLOOR_T,
                     "horn_screw_grip_hub": HORN_SCREW_GRIP, "carrier_grip": round(B.CAR_FL_T - 1.7, 2)}
    tool_lock = B.Z_TOOL_FACE_ENTRY + B.LIFT_CREST
    out["carrier_vs_spigot_at_crest_mm"] = round((Z_HORN_BOT4 - B.CAR_FL_T) - (tool_lock + B.SPIG_TOP), 2)
    out["blade_engagement_mm"] = round(B.ROTOR_TOP + B.Z_TOOL_FACE_LOCK - B.Z_BLADE_TIP, 2)
    return out


def main():
    stl, gp = H / "v4" / "stl", H / "v4" / "gate_parts"
    gp.mkdir(exist_ok=True)
    local = {"hub": hub_v4(), "cage": cage_v4(), "carrier": carrier_v4(), "pad": backpad_tpu()}
    import build_ring_v4q as RQ
    local["ring"] = RQ.ring_v4q()
    info = {"params": {"MASTER_DROP": MASTER_DROP, "HUB_T_V4": HUB_T_V4, "Z_HORN_PLANE_in_v31_frame": Z_HORN_PLANE,
                       "HORN_FACE_DROP": HORN_FACE_DROP, "SERVO_BACK_D": SERVO_BACK_D, "PAD_T": PAD_T,
                       "SOCKET_X": SOCKET_X, "PLUG_H": PLUG_H, "REAR_BOSS": [REAR_BOSS_R, REAR_BOSS_H]},
            "parts": {}}
    local["strap"], local["farpad"] = strap_v4(), farpad_tpu()
    names = {"hub": "master_hub_v4", "cage": "master_cage_v4", "carrier": "master_carrier_v4", "pad": "master_backpad_TPU_v4",
             "strap": "master_strap_v4", "farpad": "master_farpad_TPU_v4"}
    for k, n in names.items():
        s = B.to_gripper(local[k])
        export_stl(s, str(stl / f"{n}.stl"), tolerance=0.005, angular_tolerance=0.05)
        bb = s.bounding_box()
        info["parts"][n] = {"cm3": round(s.volume / 1000, 2), "valid": s.is_valid,
                            "bbox": [round(v, 2) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)]}
    sv = servo_proxy_mesh(); sv.export(str(gp / "sts3215_proxy_v4.stl"))
    export_stl(B.to_gripper(roll_horn_proxy()), str(gp / "roll_horn_proxy_v4.stl"), tolerance=0.01, angular_tolerance=0.1)
    info["checks"] = checks(local)
    # servo (official mesh) against the printed parts: manifold booleans, one pair at a time
    import trimesh
    sv_ov = {}
    for k, n in names.items():
        try:
            pm = trimesh.load(str(stl / f"{n}.stl"))
            sv_ov[k] = round(float(trimesh.boolean.intersection([pm, sv], engine="manifold").volume), 2)
        except Exception as e:  # noqa
            sv_ov[k] = f"err {e}"
    info["checks"]["servo_mesh_overlap_mm3"] = sv_ov
    (H / "v4" / "master_v4.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()
