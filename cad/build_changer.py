# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""SO-101 automatic tool changer, CAD-as-code (build123d).  v3, 2026-10-04 (agent toolchanger-2).
v2 = v1 + critic round 1 fixes; v3 = v2 + round 2 fixes; v3.1 = R3 fixes (toolchanger/reviews/R*.md; log in DESIGN.md s6).
Run:  python toolchanger/cad/build_changer.py
Writes toolchanger/cad/stl/*.stl + step/*.step + parts.json (volumes, bboxes, derived lock numbers).

FRAME = camarm's gripper_link frame, mm: horn plane z = 0, tools point -z, +x = world-up at wrist_roll 0,
y = wrist_flex axis.  Roll axis = the z axis through (HORN_CX, 0).  Master parts are built about a local axis at
x = 0 and shifted by HORN_CX at export.  Tool parts are in the TOOL frame: coupling face z = 0, tool along -z;
when LOCKED the tool frame equals the master frame with the face at Z_TOOL_FACE_LOCK.

HOW IT WORKS (DESIGN.md has the numbers):
  MASTER (stays on the arm) = hub (screws to the roll horn like the stock Wrist_Roll_Follower) + TPU gasket + cage
  holding the gripper servo (ID 6) with its output ON the roll axis + coupling ring (+ lead-in skirt) + drive blade.
  TOOL (passive, no electronics) = coupling plate with a spigot carrying 3 radial 3x12 steel dowels + floating
  rotor, and a tool body with a square neck (dock key) bolted on with 4 countersunk M3.
  LOCK: the spigot enters the ring bore and the pins pass up through 3 windows.  Wrist roll turns the master 45 deg
  while the dock holds the tool: each pin rides up a ramp (compressing 3 spring plungers with steel-ball tips that
  push the tool plate away), over a crest, and drops between two glued steel dowels (a 120-deg steel V).  3 pins in
  3 radial steel V's = Maxwell kinematic coupling, preloaded by the plungers.  Unlock = roll back 45 deg.
  DRIVE: blade on the gripper-servo horn -> slot in a radially floating rotor -> Oldham tongue -> pinion shaft.
  DOCK: printed post per tool; the tool's square neck (flats level) drops through a 45-deg flared mouth into a
  parallel slot (0.25 mm/side): the slot walls take the twist torque directly (R2 #1), the slot floor sets height,
  the plate's top edge against the wall limits tilt.  No dock pins, no pushing.

STARRED (*) parameters are unverified: measure before printing.
"""
import json
from math import cos, sin, radians, tan, degrees, atan, asin, sqrt
from pathlib import Path
from build123d import (Box, Cylinder, Cone, Pos, Rot, Polygon, extrude, export_stl, export_step, Plane, Axis,
                       Sphere, chamfer, mirror)

OUT = Path(__file__).resolve().parent
# ---- roll horn interface (verified in camarm/build_mount.py) ----
HORN_CX = -0.5
HOLE_R, CB_R, CB_D = 1.75, 3.2, 3.2
# ---- STS3215 (case dims + axis offset measured on dashboard/meshes/sts3215_03a_v1.stl) ----
SV_L, SV_W, SV_H = 45.4, 24.8, 35.4
SV_SHAFT_OFF = 10.2                      # output axis to the near end of the case
HORN_T, HORN_R = 4.2, 10.0               # disc horn 20 mm dia, top 4.2 above the case face; * 4x M3 on a 10 mm square
# ---- z stack (gripper frame, mm) ----
HUB_T = 6.0
GASKET_T = 0.6                           # TPU gasket between hub and servo case (R1 S7)
CAV_H = SV_H + GASKET_T - 0.05           # 0.05 mm squeeze on the gasket (R2 #14); hub bottoms on the cage ears
Z_CASE_OUT = -HUB_T - CAV_H              # cage floor top = servo output face (-41.9)
FLOOR_T = 2.5
Z_RING_TOP = Z_CASE_OUT - FLOOR_T        # -44.4
Z_HORN_BOT = Z_CASE_OUT - HORN_T         # -46.1
LIP_T = 4.0
Z_LIP_TOP = -57.0
Z_LIP_BOT = Z_LIP_TOP - LIP_T            # -61
SKIRT_H = 4.0                            # lead-in skirt below the lip (R1 hobby #1: capture)
# ---- ring / lip ----
R_RING = 34.0
R_SKIRT_IN, R_SKIRT_OUT, R_SKIRT_MOUTH = 34.8, 39.5, 38.0   # separate glued skirt part (R2 #7)
R_CAV = 24.2                             # cavity above the lip (pins rotate in here)
R_BORE = 14.3                            # lip bore; spigot R_SPIG
R_SPIG = 14.0
BORE_CHAMFER = 3.0                       # lip-bottom lead-in (capture +-3 mm)
WIN = 16.0                               # window half-angle (deg); pins enter at phi = 0, 120, 240 (master frame)
R_WIN = 23.7
RAMP = (18.0, 37.0)                      # R2: longer ramp for the 1.9 crest
H_CREST = 1.9                            # >= near dowel top (1.70) + 0.2
PHI_V = 45.0                             # locked pin angle = twist
PHI_STOP = (62.0, 100.0)                 # tall wall far side
STOP_T = (3.1, 9.0)                      # R2 #5: radial-parallel stop block right behind the far dowel (tangential mm)
H_STOP = 4.5
# ---- steel V: two 3x12 dowels glued in radial grooves, parallel to the radial line at PHI_V, offset +-1.5 ----
DOWEL_D, DOWEL_L, DOWEL_RIN = 3.0, 12.0, 14.5
# ---- pins (tool side) ----
PIN_D, PIN_L = 3.0, 12.0                 # ZYLtech 3x12 steel dowel
PIN_RIN = 11.0                           # blind end of the hole (3 mm embed); tip at r 23
PIN_HOLE_D = 3.1                         # teardrop, glue with CA (R1 S6)
PIN_GAP = 1.0                            # pin bottom above the lip top at entry (R1 N1, R2 #6)
H_DOWEL = PIN_D / 2 + PIN_GAP + 0.3 - sqrt(DOWEL_D ** 2 - (DOWEL_D / 2) ** 2)   # dowel centre z (lock lift 0.3)
# ---- plungers (steel-ball tip, R1 S4) ----
R_PL = 29.0                              # R2 #15
PL_ANG = (60.0, 180.0, 300.0)
PL_HEAD_D, PL_HEAD_H, PL_SHANK_D, PL_BALL_D, PL_PROTRUDE = 6.4, 2.5, 5.0, 3.97, 1.5   # 5/32 in ball
PL_GUIDE = 6.0                           # shank guide length (R1 S4)
SPRING = dict(od=5.0, wire=0.7, free=11.0, rate_N_per_mm=5.0)   # * measure; installed length is derived below
# ---- ring <-> cage screws: outside the cage box strip (R1 B2) ----
RING_SCREW_R, RING_SCREW_ANG = 31.0, (90.0, 215.0, 270.0)   # R2 #8
# ---- tool coupling plate / spigot / rotor ----
TP_T = 5.0
R_TP = R_RING
Z_TOOL_FACE_ENTRY = Z_LIP_BOT - PL_PROTRUDE                          # -62.5
TOOL_PIN_Z = (Z_LIP_TOP + PIN_D / 2 + PIN_GAP) - Z_TOOL_FACE_ENTRY    # pin centre above the tool face (7.6)
SPIG_TOP = TOOL_PIN_Z + 3.5                                          # R1 S6
SPIG_CHAMFER = 0.6                                                   # R2 #17
R_SPIG_BORE = 9.5                        # rotor floats 1.0 mm radially (R1 S5; R2 #9: 1.5 mm pin web)
LIP_IN_R, LIP_IN_T = 7.0, 1.2            # inward retaining lip at the spigot top (rotor r 8.5 - float 1.0 > 7.0)
ROTOR_R, SLOT_W, SLOT_D = 8.5, 3.6, 5.0
SLOT_LEAD = 1.3                          # R3 #1: slot-mouth chamfer (capture +-2.7 mm > 1.0 float)
ROTOR_TOP = SPIG_TOP - LIP_IN_T
PLATE_SCREW_R = 21.0                     # 4x M3x10 countersunk, face -> inserts toward the neck corners (45 + 90k deg)
PIN_PHASE = PHI_V                        # tool frame == master frame when LOCKED: pins at 45/165/285 deg
# ---- tool body neck (dock key) ----
NECK_SQ, NECK_H = 36.0, 16.0             # square, flats on tool +-x / +-y (level in the dock) x length
# ---- blade carrier (on the gripper servo horn) ----
CAR_FL_R, CAR_FL_T = 11.0, 3.2
CAR_BODY_R = 5.5                         # R1 B4; passes the r 7 rotor lip with 1 mm float
CAR_PILOT_R, CAR_PILOT_H = 4.0, 1.5      # * into the horn centre
Z_CAR_BOT = -51.0
BLADE_T, BLADE_W = 3.2, 11.0             # R1 S5 / R2: blade + float passes the r 7 lip
# ---- derived lock kinematics ----
PIN_C_ENTRY = PIN_D / 2 + PIN_GAP                                  # pin centre above the lip top
PIN_C_LOCK = H_DOWEL + sqrt(DOWEL_D ** 2 - (DOWEL_D / 2) ** 2)     # resting on the two dowels (30-deg normals)
PIN_C_CREST = H_CREST + PIN_D / 2
LIFT_LOCK = PIN_C_LOCK - PIN_C_ENTRY
LIFT_CREST = PIN_C_CREST - PIN_C_ENTRY
Z_TOOL_FACE_LOCK = Z_TOOL_FACE_ENTRY + LIFT_LOCK
Z_BLADE_TIP = Z_TOOL_FACE_LOCK + ROTOR_TOP - 4.0                     # 4 mm into the slot when locked


def sector(r0, r1, a0, a1, z0, z1, n=None):
    """Annular sector between radii r0..r1 and angles a0..a1 (deg), z0..z1."""
    n = n or max(2, int(abs(a1 - a0) / 3) + 1)
    pts = [(r1 * cos(radians(a0 + (a1 - a0) * i / n)), r1 * sin(radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]
    pts += [(r0 * cos(radians(a1 - (a1 - a0) * i / n)), r0 * sin(radians(a1 - (a1 - a0) * i / n))) for i in range(n + 1)]
    return Pos(0, 0, z0) * extrude(Polygon(*pts, align=None), amount=z1 - z0)


def radial_prism(tri_yz, r0, r1, ang):
    """Polygon given in (tangential, z) extruded along the radial line at angle ang from r0 to r1."""
    face = Plane.YZ.offset(r0) * Polygon(*tri_yz, align=None)
    return Rot(0, 0, ang) * extrude(face, amount=r1 - r0, dir=(1, 0, 0))


def radial_rod(d, r0, r1, ang, t_off, z):
    """Cylinder of diameter d parallel to the radial line at ang, offset t_off tangentially, from r0 to r1."""
    return Rot(0, 0, ang) * Pos((r0 + r1) / 2, t_off, z) * Rot(0, 90, 0) * Cylinder(d / 2, r1 - r0)


def _horn_holes(body, z_top, t):
    for sx in (-5, 5):
        for sy in (-5, 5):
            body -= Pos(sx, sy, z_top - t / 2) * Cylinder(HOLE_R, t + 2)
            body -= Pos(sx, sy, z_top - t + CB_D / 2 - 0.01) * Cylinder(CB_R, CB_D + 0.02)
    return body


# ------------------------------------------------------------------ master
def hub():
    """Screws to the wrist-roll horn (4x M3 from below, counterbored) like the stock part; 2 M3 heat-set inserts
    at y = +-17.5 take the cage screws."""
    b = Pos(0, 0, -HUB_T / 2) * Box(24, 42, HUB_T)
    b = _horn_holes(b, 0, HUB_T)
    b -= Pos(0, 0, -HUB_T / 2) * Cylinder(2.8, HUB_T + 2)
    for sy in (-17.5, 17.5):
        b -= Pos(0, sy, -HUB_T + 2.6) * Cylinder(2.0, 5.3)
    return b


def gasket():
    """0.6 mm TPU gasket between the hub and the servo case top (print in the fin ray TPU)."""
    x0, x1 = -SV_SHAFT_OFF, SV_L - SV_SHAFT_OFF
    g = Pos((x0 + x1) / 2, 0, -HUB_T - GASKET_T / 2) * Box(x1 - x0 - 1, SV_W - 1, GASKET_T)
    return g


def cage():
    """Servo cage: the servo is pre-loaded through the TPU gasket between the hub and this floor; two 0.3 mm crush
    ribs push the case into one corner (R1 S7); output on the roll axis."""
    w = 2.4
    x0, x1 = -SV_SHAFT_OFF - 0.2, SV_L - SV_SHAFT_OFF + 0.2
    y1 = SV_W / 2 + 0.2
    zt, zb = -HUB_T, Z_RING_TOP
    outer = Pos((x0 + x1) / 2, 0, (zt + zb) / 2) * Box(x1 - x0 + 2 * w, 2 * (y1 + w), zt - zb)
    inner = Pos((x0 + x1) / 2, 0, (zt + Z_CASE_OUT) / 2 + 0.5) * Box(x1 - x0, 2 * y1, zt - Z_CASE_OUT + 1)
    b = outer - inner
    b += Pos(x1 - 0.25, 0, (zt + Z_CASE_OUT) / 2) * Box(0.5, 6, CAV_H - 4)            # crush rib, +x wall
    b += Pos(10, y1 - 0.25, (zt + Z_CASE_OUT) / 2) * Box(6, 0.5, CAV_H - 4)           # crush rib, +y wall
    for sy in (-1, 1):                                                  # full-height ears for the hub screws
        b += Pos(0, sy * (y1 + w + 2.5), (zt + zb) / 2) * Box(9, 5.5, zt - zb)
    b -= Pos(x1 + w / 2, 0, zt - 6) * Box(w + 2, 10, 6)                # * servo cable exit (check socket side)
    for sx in (5, 25):                                                  # cable clip tabs on the +x end
        b += Pos(x1 + w + 2, 0, zt - 14 - sx / 2) * Box(4, 12, 3)
        b -= Pos(x1 + w + 2, 0, zt - 14 - sx / 2) * Box(2.2, 8, 4)
    disc = Pos(0, 0, zb + FLOOR_T / 2) * Cylinder(R_RING, FLOOR_T)
    body = b + disc
    body -= Pos(0, 0, zb + FLOOR_T / 2) * Cylinder(HORN_R + 2.5, FLOOR_T + 2)          # horn clearance
    for sy in (-17.5, 17.5):                                            # hub screws: through ear AND disc (R1 B1)
        body -= Pos(0, sy, (zt + zb) / 2) * Cylinder(1.75, zt - zb + 2)
        body -= Pos(0, sy, zb + 1.6) * Cylinder(3.2, 3.2)
    for a in RING_SCREW_ANG:                                            # ring screws, heads on top of the disc
        body -= Pos(RING_SCREW_R * cos(radians(a)), RING_SCREW_R * sin(radians(a)), zb + FLOOR_T / 2) * Cylinder(1.75, FLOOR_T + 2)
    # wrist-cam pad on the -y wall: 2 M3 nut traps 16 mm apart (* match the stock cam bracket)
    pad = Pos(21, -(y1 + w + 2), zt - 14) * Box(26, 4, 16)          # x 8..34: clear of the hub screw at x 0 (R2 #3)
    for sx in (13, 29):
        pad -= Pos(sx, -(y1 + w + 2), zt - 14) * Rot(90, 0, 0) * Cylinder(1.75, 6)
        pad -= Pos(sx, -(y1 + w + 3.2), zt - 14) * Rot(90, 0, 0) * Cylinder(3.2, 2.5)   # hex drawn round
    return body + pad


def ring():
    """Coupling ring (print lip-bottom down, PETG): lip with windows, ramps, crests, steel-dowel V grooves, stop
    walls; 3 plunger pockets; lead-in skirt."""
    zt, zb = Z_RING_TOP, Z_LIP_BOT
    b = Pos(0, 0, (zt + zb) / 2) * Cylinder(R_RING, zt - zb)
    b += Pos(0, 0, (zt + zb + 3.0) / 2) * Cylinder(R_RING + 1.5, zt - zb - 3.0)       # R3: shoulder at lip_bot+3 = skirt stop
    b -= Pos(0, 0, (zt + Z_LIP_TOP) / 2 + 0.01) * Cylinder(R_CAV, zt - Z_LIP_TOP + 0.02)    # cavity
    b -= Pos(0, 0, (Z_LIP_TOP + zb) / 2) * Cylinder(R_BORE, LIP_T + 2)                      # bore
    b -= Pos(0, 0, zb + BORE_CHAMFER / 2) * Cone(R_BORE + BORE_CHAMFER, R_BORE, BORE_CHAMFER)   # 45-deg lead-in
    zd = Z_LIP_TOP + H_DOWEL
    for k in range(3):
        a0 = 120 * k
        b -= sector(R_BORE - 1, R_WIN, a0 - WIN, a0 + WIN, zb - 1, Z_LIP_TOP + 0.01)         # window
        b -= sector(R_BORE - 1, R_WIN + 2.5, a0 - WIN - 6, a0 + WIN + 6, zb - 0.01, zb + 1.0)  # pin funnel
        steps = int(RAMP[1] - RAMP[0])
        for i in range(steps):
            h = H_CREST * (i + 1) / steps
            b += sector(R_BORE, R_CAV, a0 + RAMP[0] + i, a0 + RAMP[0] + i + 1.02, Z_LIP_TOP - 0.01, Z_LIP_TOP + h)
        b += sector(R_BORE, R_CAV, a0 + RAMP[1], a0 + PHI_STOP[0], Z_LIP_TOP - 0.01, Z_LIP_TOP + H_CREST)
        b += sector(R_BORE, R_CAV, a0 + PHI_STOP[0], a0 + PHI_STOP[1], Z_LIP_TOP - 0.01, Z_LIP_TOP + H_STOP)
        # clearance trough for the pin between the dowels (120-deg V, bottom 0.3 above the lip top)
        top = H_CREST + 0.8
        hw = (top - 0.3) / tan(radians(30))
        b -= Pos(0, 0, Z_LIP_TOP) * radial_prism([(-hw, top), (hw, top), (0, 0.3)], R_BORE - 1, R_CAV + 0.5, a0 + PHI_V)
        stop = radial_prism([(STOP_T[0], -0.01), (STOP_T[1], -0.01), (STOP_T[1], H_STOP), (STOP_T[0], H_STOP)], R_BORE, R_CAV + 0.3, a0 + PHI_V)
        b += Pos(0, 0, Z_LIP_TOP) * stop
        for s in (-1, 1):                                               # dowel grooves 3.05 (ream); slide dowels outward from the bore
            b -= radial_rod(DOWEL_D + 0.05, DOWEL_RIN - 0.2, DOWEL_RIN + DOWEL_L + 0.2, a0 + PHI_V, s * DOWEL_D / 2, zd)
    for a in PL_ANG:                                                    # plunger pockets: 6 mm guide + head/spring bore
        x, y = R_PL * cos(radians(a)), R_PL * sin(radians(a))
        b -= Pos(x, y, zb + PL_GUIDE / 2 - 0.01) * Cylinder(PL_SHANK_D / 2 + 0.15, PL_GUIDE + 0.02)
        b -= Pos(x, y, (zt + zb + PL_GUIDE) / 2 + 0.01) * Cylinder(PL_HEAD_D / 2 + 0.15, zt - zb - PL_GUIDE + 0.02)
    for a in RING_SCREW_ANG:                                            # M3 insert bores from the top
        x, y = RING_SCREW_R * cos(radians(a)), RING_SCREW_R * sin(radians(a))
        b -= Pos(x, y, zt - 2.85) * Cylinder(2.0, 5.7)                            # insert
        b -= Pos(x, y, zt - 5.7 - 2.0) * Cylinder(1.3, 4.0)                       # R3: screw-tip clearance (M3x8)
    return b


def skirt():
    """Lead-in skirt (separate so the ring prints lip-down without support, R2 #7): 3 mm collar slips over the ring
    bottom (glue), funnel r 38 -> 34.8 below the lip.  Print mouth-down."""
    zb = Z_LIP_BOT
    s = Pos(0, 0, zb - SKIRT_H / 2 + 1.5) * Cylinder(R_SKIRT_OUT, SKIRT_H + 3.0)
    s -= Pos(0, 0, zb + 1.5) * Cylinder(R_RING + 0.15, 3.02)
    s -= Pos(0, 0, zb - SKIRT_H / 2) * Cone(R_SKIRT_MOUTH, R_SKIRT_IN, SKIRT_H + 0.02)
    return s


def dowels():
    zd = Z_LIP_TOP + H_DOWEL
    out = None
    for k in range(3):
        for s in (-1, 1):
            d = radial_rod(DOWEL_D, DOWEL_RIN, DOWEL_RIN + DOWEL_L, 120 * k + PHI_V, s * DOWEL_D / 2, zd)
            out = d if out is None else out + d
    return out


def plunger():
    """Printed plunger (head + shank) with a 4 mm steel ball glued in the tip socket (ball drawn)."""
    z_head_bot = Z_LIP_BOT + PL_GUIDE
    p = Pos(0, 0, z_head_bot + PL_HEAD_H / 2) * Cylinder(PL_HEAD_D / 2, PL_HEAD_H)
    zc = Z_LIP_BOT - PL_PROTRUDE + PL_BALL_D / 2                      # ball centre
    p += Pos(0, 0, (z_head_bot + zc) / 2) * Cylinder(PL_SHANK_D / 2, z_head_bot - zc)
    p -= Pos(0, 0, zc) * Sphere(PL_BALL_D / 2 + 0.05)
    return p, Pos(0, 0, zc) * Sphere(PL_BALL_D / 2)


def carrier():
    """Blade carrier on the gripper-servo horn: flange (4x M3 from below into the horn) + pilot + body + blade."""
    zf = Z_HORN_BOT
    c = Pos(0, 0, zf - CAR_FL_T / 2) * Cylinder(CAR_FL_R, CAR_FL_T)
    c += Pos(0, 0, zf + CAR_PILOT_H / 2) * Cylinder(CAR_PILOT_R, CAR_PILOT_H)
    c += Pos(0, 0, (zf - CAR_FL_T + Z_CAR_BOT) / 2) * Cylinder(CAR_BODY_R, zf - CAR_FL_T - Z_CAR_BOT)
    for sx in (-5, 5):
        for sy in (-5, 5):
            c -= Pos(sx, sy, zf - 3) * Cylinder(1.75, 8)
            c -= Pos(sx, sy, zf - CAR_FL_T - 3 + 1.7) * Cylinder(3.0, 6)       # R3 #2: M3 BUTTON head (1.65) flush
    bl = Pos(0, 0, (Z_CAR_BOT + Z_BLADE_TIP) / 2) * Box(BLADE_T, BLADE_W, Z_CAR_BOT - Z_BLADE_TIP)
    bl = chamfer(bl.edges().group_by(Axis.Z)[0].filter_by(Axis.Y), 1.1)   # R3 #1: 1.0 mm tip
    return c + bl


def master_parts():
    d = {"hub": hub(), "gasket_TPU": gasket(), "cage": cage(), "ring": ring(), "skirt": skirt(), "carrier": carrier(), "dowels": dowels()}
    for k, a in enumerate(PL_ANG):
        p, ball = plunger()
        d[f"plunger{k}"] = Pos(R_PL * cos(radians(a)), R_PL * sin(radians(a)), 0) * p
        d[f"ball{k}"] = Pos(R_PL * cos(radians(a)), R_PL * sin(radians(a)), 0) * ball
    return d


# ------------------------------------------------------------------ tool side (TOOL frame: face z = 0)
def coupling_plate():
    """Coupling plate + spigot (print face down).  Pins: 3.1 teardrop holes, glued.  4 countersunk M3 to the body."""
    p = Pos(0, 0, -TP_T / 2) * Cylinder(R_TP, TP_T)
    p += Pos(0, 0, SPIG_TOP / 2) * Cylinder(R_SPIG, SPIG_TOP)
    p = chamfer(p.edges().group_by(Axis.Z)[-1], SPIG_CHAMFER)
    p -= Pos(0, 0, (SPIG_TOP - LIP_IN_T - TP_T) / 2 - 0.01) * Cylinder(R_SPIG_BORE, SPIG_TOP - LIP_IN_T + TP_T + 0.02)
    p -= Pos(0, 0, SPIG_TOP - LIP_IN_T / 2) * Cylinder(LIP_IN_R, LIP_IN_T + 0.02)
    for k in range(3):
        a = 120 * k + PIN_PHASE
        L = R_SPIG + 1 - PIN_RIN
        h = Rot(0, 0, a) * Pos(PIN_RIN + L / 2, 0, TOOL_PIN_Z) * Rot(0, 90, 0) * Cylinder(PIN_HOLE_D / 2, L)
        tear = Rot(0, 0, a) * Pos(PIN_RIN + L / 2, 0, TOOL_PIN_Z + PIN_HOLE_D / 2 * 0.7) * Rot(45, 0, 0) * Box(L, PIN_HOLE_D * 0.62, PIN_HOLE_D * 0.62)
        p -= h + tear
    for k in range(4):
        a = radians(45 + 90 * k)
        x, y = PLATE_SCREW_R * cos(a), PLATE_SCREW_R * sin(a)
        p -= Pos(x, y, -TP_T / 2) * Cylinder(1.7, TP_T + 2)
        p -= Pos(x, y, -0.75 + 0.01) * Cone(1.7, 3.2, 1.5)                  # 90-deg countersink from the face (R2 #10)
    return p


def pins():
    out = None
    for k in range(3):
        s = Rot(0, 0, 120 * k + PIN_PHASE) * Pos(PIN_RIN + PIN_L / 2, 0, TOOL_PIN_Z) * Rot(0, 90, 0) * Cylinder(PIN_D / 2, PIN_L)
        out = s if out is None else out + s
    return out


def rotor():
    """Floating rotor: blade slot on top, Oldham tongue slot (perpendicular) below; retained by the spigot lip."""
    h = ROTOR_TOP + TP_T + 2.0
    r = Pos(0, 0, ROTOR_TOP - h / 2) * Cylinder(ROTOR_R, h)
    r -= Pos(0, 0, ROTOR_TOP - SLOT_D / 2 + 0.01) * Box(SLOT_W, 2 * ROTOR_R + 1, SLOT_D)
    a = (SLOT_W / 2 + SLOT_LEAD) * sqrt(2)                          # R3 #1: 45-deg lead-in, mouth 3.6 + 2 x 1.3
    r -= Pos(0, 0, ROTOR_TOP) * Rot(0, 45, 0) * Box(a, 2 * ROTOR_R + 1, a)
    r -= Pos(0, 0, ROTOR_TOP - h + 2.0) * Box(2 * ROTOR_R + 1, 4.2, 4.0)
    return r


def neck(z_top=-TP_T):
    """Square dock key (flats on tool +-x / +-y), 1.5 mm edge chamfers, 4 M3 inserts at r 21 toward the corners."""
    n = Pos(0, 0, z_top - NECK_H / 2) * Box(NECK_SQ, NECK_SQ, NECK_H)
    n = chamfer(n.edges().filter_by(Axis.Z), 1.5)
    n -= Pos(0, 0, z_top - NECK_H / 2) * Cylinder(R_SPIG_BORE, NECK_H + 2)
    for k in range(4):
        a = radians(45 + 90 * k)
        n -= Pos(PLATE_SCREW_R * cos(a), PLATE_SCREW_R * sin(a), z_top - 3) * Cylinder(2.0, 6.02)
    return n


Z0 = -TP_T - NECK_H                      # tool body starts here


def gripper_body():
    """Parallel gripper below the neck: m1 z20 pinion (r 10) drives two racks/jaws in opposite x; 143 deg of servo
    = 25 mm per jaw.  CONCEPT LEVEL (teeth, slides, shaft not detailed)."""
    hb = Pos(0, 0, Z0 - 8) * Box(78, 34, 16)
    hb -= Pos(0, 0, Z0 - 8) * Box(80, 24.4, 10.4)
    hb -= Pos(0, 0, Z0 - 8) * Cylinder(11.5, 18)
    hb -= Pos(0, 0, Z0 - 16) * Box(80, 12, 2.5)
    return hb + neck()


def jaw(x_inner, side, stub_w=16.0):
    """stub_w 16 = the fixed-jaw A1 stub (carrier_fixed_v2 fits), 14 = the moving-jaw stub (carrier_moving_v2 fits)."""
    xc = side * (x_inner + 12)
    j = Pos(xc, 0, Z0 - 8) * Box(24, 24, 10)
    for i in range(7):
        j += Pos(xc - 9 + i * 3.1416, side * 9.0, Z0 - 8) * Box(1.5, 2.2, 10)
    arm = Pos(side * (x_inner + 5), 0, Z0 - 22) * Box(10, 16, 18)
    hw = stub_w / 2
    stub = Pos(side * (x_inner + 5), 0, Z0 - 31) * Rot(180, 0, 0) * extrude(Polygon((-5, -hw), (5, -hw), (5, hw), (-5, hw), align=None), amount=14 if stub_w > 15 else 12, taper=8)
    return j + arm + stub


def pinion():
    pts = []
    for i in range(40):
        a = radians(i * 9); r = 11.0 if i % 2 == 0 else 9.0
        pts.append((r * cos(a), r * sin(a)))
    p = Pos(0, 0, Z0 - 13) * extrude(Polygon(*pts, align=None), amount=10)
    z_rot_bot = ROTOR_TOP - (ROTOR_TOP + TP_T + 2.0)                                   # rotor bottom (-7)
    z_sh_top = z_rot_bot - 0.4
    p += Pos(0, 0, (Z0 - 3 + z_sh_top) / 2) * Cylinder(4.0, z_sh_top - (Z0 - 3))        # shaft ends below the rotor (R2 #4)
    p += Pos(0, 0, z_sh_top + 1.6) * Box(2 * ROTOR_R - 3, 3.8, 3.2)                    # Oldham tongue in the rotor slot
    return p


def camera_tool():
    """ZV-1 tool (concept; 328 g: NOT recommended for v1, see DESIGN.md)."""
    t = neck()
    t += Pos(0, 0, Z0 - 2) * Box(64, 70, 4)
    cam = Pos(0, 0, Z0 - 4 - 21.75) * Box(60, 105.5, 43.5)
    lens = Pos(0, 0, Z0 - 4 - 43.5 - 6) * Cylinder(14, 12)
    return t, cam, lens


def scanner_tool():
    t = neck()
    body = Pos(0, 0, Z0 - 20) * Box(70, 44, 40)
    win = Pos(0, 0, Z0 - 40.5) * Box(50, 20, 1.5)
    return t, body, win


# ------------------------------------------------------------------ dock posts
# The SO-101 wrist-flex limit (~95 deg) rules out a vertical approach to a shelf tall enough for the tools to hang
# under it (analysis/reach_torque.py).  Tools park LYING DOWN, coupling facing the arm, in three posts on an arc.
# DOCK frame: origin = tool axis point on the wall's arm-side face, +x = along the tool axis away from the arm,
# +y horizontal, +z up; the table is at z = -DOCK_H.
DOCK_H = 100.0
DOCK_R = 280.0           # base axis -> wall face
DOCK_AZ = (-35.0, 0.0, 35.0)
WALL_T, WALL_HW, WALL_TOP = 12.0, 62.0, 40.0
PLATE_GAP = 1.0          # plate back to wall face when seated (tilt limit ~1 deg)
SLOT_CLR = 0.25          # per side (R2 #1)


def dock_post():
    """Wall with a parallel slot (neck width + 0.25/side) from the floor (neck bottom) up to the neck top, then a
    45-deg flare to the wall top (capture +-22 mm); 3 mm axial lead-in chamfers on both faces."""
    wall = Pos(WALL_T / 2, 0, (WALL_TOP - DOCK_H) / 2) * Box(WALL_T, 2 * WALL_HW, WALL_TOP + DOCK_H)
    hw, zf, zn = NECK_SQ / 2 + SLOT_CLR, -NECK_SQ / 2, NECK_SQ / 2
    mouth = hw + (WALL_TOP - zn)
    prof = [(-hw, zf), (hw, zf), (hw, zn), (mouth + 1, WALL_TOP + 1), (-mouth - 1, WALL_TOP + 1), (-hw, zn)]
    wall -= Pos(-1, 0, 0) * extrude(Plane.YZ * Polygon(*prof, align=None), amount=WALL_T + 2, dir=(1, 0, 0))
    big = [(-hw - 3, zf - 0.01), (hw + 3, zf - 0.01), (hw + 3, zn), (mouth + 4, WALL_TOP + 1), (-mouth - 4, WALL_TOP + 1), (-hw - 3, zn)]
    front = extrude(Plane.YZ * Polygon(*big, align=None), amount=3.0, taper=45)
    wall -= front + mirror(front, Plane.YZ.offset(WALL_T / 2))
    foot = Pos(25, 0, -DOCK_H + 3) * Box(90, 2 * WALL_HW, 6)
    gus = None
    for sv in (-1, 1):
        g = Pos(WALL_T, sv * (WALL_HW - 3) + 3, -DOCK_H + 6) * extrude(Plane.XZ * Polygon((0, 0), (45, 0), (0, 70), align=None), amount=6)
        gus = g if gus is None else gus + g
    return wall + foot + gus


# ------------------------------------------------------------------ coupon (print first)
def coupon_toolplate():
    return coupling_plate()


def to_gripper(shape):
    return Pos(HORN_CX, 0, 0) * shape


def report():
    k = SPRING["rate_N_per_mm"]
    installed = (Z_RING_TOP - (Z_LIP_BOT + PL_GUIDE + PL_HEAD_H))     # head top to the cage floor
    F0 = k * (SPRING["free"] - installed)
    f_lock, f_crest = 3 * (F0 + k * LIFT_LOCK), 3 * (F0 + k * LIFT_CREST)
    mu_ss, mu_ramp, mu_ball = 0.15, 0.3, 0.15                          # steel/steel V, steel pin on PETG ramp, ball on plate
    r_v, r_ramp = 18.5, 15.0                                           # V line centre; ramp contact at the pin root (R1 S3)
    hold = f_lock * tan(radians(30) + atan(mu_ss)) * r_v / 1000 + f_lock * mu_ball * R_PL / 1000
    slope = degrees(atan(H_CREST / (radians(RAMP[1] - RAMP[0]) * r_ramp)))
    climb = f_crest * tan(radians(slope) + atan(mu_ramp)) * r_ramp / 1000 + f_crest * mu_ball * R_PL / 1000
    r_p = 18.5
    tilt = 2 / 2 * 0 + (f_lock / 3) * 3 * r_p / 2 / 1000               # pin unloads when 2M/(3 r_p) = F_pin
    return {
        "z_stack_mm": {"hub_bottom": -HUB_T, "servo_output_face": round(Z_CASE_OUT, 2), "horn_bottom": round(Z_HORN_BOT, 2),
                       "ring_top": round(Z_RING_TOP, 2), "lip_top": Z_LIP_TOP, "lip_bottom": Z_LIP_BOT, "skirt_bottom": Z_LIP_BOT - SKIRT_H,
                       "tool_face_entry": Z_TOOL_FACE_ENTRY, "tool_face_locked": round(Z_TOOL_FACE_LOCK, 2),
                       "blade_tip": round(Z_BLADE_TIP, 2)},
        "lock": {"twist_deg": PHI_V, "pin_lift_at_crest_mm": round(LIFT_CREST, 2), "pin_lift_locked_mm": round(LIFT_LOCK, 2),
                 "tool_pin_z_mm": round(TOOL_PIN_Z, 2), "plate_gap_locked_mm": round(Z_LIP_BOT - Z_TOOL_FACE_LOCK, 2),
                 "plate_gap_crest_mm": round(Z_LIP_BOT - (Z_TOOL_FACE_ENTRY + LIFT_CREST), 2),
                 "window_capture_deg_at_r14": round(degrees(((2 * R_BORE * sin(radians(WIN))) - PIN_D) / 2 / R_BORE), 1),
                 "lateral_capture_mm": BORE_CHAMFER, "skirt_capture_mm": R_SKIRT_MOUTH - R_TP},
        "forces": {"spring_installed_mm": round(installed, 2), "preload_each_N": round(F0, 1),
                   "preload_locked_N": round(f_lock, 1), "preload_crest_N": round(f_crest, 1),
                   "unlock_or_hold_torque_Nm": round(hold, 2), "lock_climb_torque_Nm": round(climb, 2),
                   "ramp_slope_deg_at_pin_root": round(slope, 1), "tilt_moment_before_pin_lifts_Nm": round(tilt, 2),
                   "assumptions": "steel/steel V mu 0.15, pin on PETG ramp mu 0.3, steel ball on plate mu 0.15, k 5 N/mm (measure)"},
    }


def main():
    (OUT / "stl").mkdir(exist_ok=True)
    (OUT / "step").mkdir(exist_ok=True)
    for stale in ("master_v0", "toolplate_v0", "rack", "tool_plate_gripper", "camera_tool_plate", "scanner_tool_plate"):
        f = OUT / "stl" / f"{stale}.stl"
        if f.exists(): f.unlink()
    info = {"parts": {}}
    parts = {**{f"master_{k}": to_gripper(v) for k, v in master_parts().items()}}
    parts["tool_coupling_plate"] = coupling_plate()
    parts["tool_pins"] = pins()
    parts["tool_rotor"] = rotor()
    parts["gripper_body"] = gripper_body()
    parts["gripper_pinion"] = pinion()
    parts["gripper_jaw_L"] = jaw(14.0, -1)
    parts["gripper_jaw_R"] = jaw(14.0, 1, stub_w=14.0)
    ct, cam, lens = camera_tool()
    parts["camera_neck_pad"], parts["camera_zv1_box"], parts["camera_zv1_lens"] = ct, cam, lens
    st, sb, sw = scanner_tool()
    parts["scanner_neck"], parts["scanner_body"], parts["scanner_window"] = st, sb, sw
    parts["dock_post"] = dock_post()
    for name, s in parts.items():
        export_stl(s, str(OUT / "stl" / f"{name}.stl"))
        bb = s.bounding_box()
        info["parts"][name] = {"cm3": round(s.volume / 1000, 2),
                               "bbox": [round(v, 1) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)]}
    for name in ("master_hub", "master_cage", "master_ring", "master_carrier", "tool_coupling_plate", "gripper_body", "dock_post"):
        export_step(parts[name], str(OUT / "step" / f"{name}.step"))
    info.update(report())
    (OUT / "parts.json").write_text(json.dumps(info, indent=1))
    print(json.dumps({k: v for k, v in info.items() if k != "parts"}, indent=1))


if __name__ == "__main__":
    main()
