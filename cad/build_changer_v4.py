# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""SO-101 tool changer v4 = v3.1 + the changes the docking sim (toolchanger/sim) showed were needed.  2026-10-04,
agent toolchanger-3.  Imports build_changer (v3.1) and overrides only what changed; v3.1 files stay untouched.

Run:  python toolchanger/cad/build_changer_v4.py
Writes toolchanger/cad/v4/stl/*.stl + v4/step/*.step + v4/parts_v4.json

v4 geometry changes (sim evidence in toolchanger/sim/out, DESIGN_v4.md):
  RING   lip bore R 14.3 -> 14.8 (0.8 mm radial clearance: a 0.3 mm bore dragged the parked tool out of the dock
         while the arm retracted with its normal 0.5-1 mm friction lag); windows R 23.7 -> 24.5; ramps / crest /
         stop wall start at r 15.6 (a relief ring around the spigot; v3.1 started them at the bore radius, so a
         0.5 mm offset made the spigot rub them); V dowels slide in from r 15.8 (was 14.5: dowel ends touched the
         spigot).  The cavity stays R 24.2 (pin tips reach 23 + 0.8 offset = 23.8).
  ROTOR  v3.1 rotor, NO spring (critic R2: no room for one beside the Oldham tongue).  A mis-parked rotor now just
         stops the insertion ~4 mm short with the arm still pressing (the arm's 1.3 N/mm axial compliance is the
         spring); the routine sweeps the blade toward the rotor's open stop right there until it drops in.
  DOCK   plate gap 1.0 -> 2.0; slot clearance 0.25 -> 0.4 per side; the slot sides AND the floor are an 8 mm land
         (v3.1: the full 12 mm wall) so a neck pitched by the arm's droop cannot jam across the wall; 5 mm x 45-deg
         bevel on the wall's arm-side top edge (v3.1 plates hung on the flat top); a tail-rest rib 0.3 mm below the
         tool body (v3.1 tools tipped 3.7 deg nose-down: the CoM is 38 mm past the wall); one N52 6x3 magnet in the
         land (1 mm deep) facing one flush in the tool's neck (~3-4 N: holds the parked tool while the master backs out).
  SKIRT  (r3) 5.5 mm tall, mouth r 40 (capture +-4 -> +-6 mm).
Not geometry: PTFE dry lube on the ramps / V / ball track; roll Torque_Limit 85 % during the 1.5 s twist (sim lock
torque 1.5 N.m nominal, 2.3 worst corner vs the v3.1 55 % = 1.6 N.m); see swap routine.
"""
import json
import sys
from math import cos, sin, radians, tan, sqrt
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot, Polygon, Sphere, extrude, export_stl, export_step, Plane, mirror

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
import build_changer as B  # noqa: E402

OUT = H / "v4"
# ---- v4 parameters (mm) ----
R_BORE4, R_WIN4, RELIEF_RIN, DOWEL_RIN4 = 14.8, 24.5, 15.6, 15.8
PLATE_GAP4, SLOT_CLR4, LAND, TOP_BEVEL = 2.0, 0.4, 8.0, 5.0    # R2: land 6 -> 8 (holds the magnets, less crushing)
TAIL_X = (B.NECK_H - PLATE_GAP4 + 4.0, B.NECK_H - PLATE_GAP4 + 20.0)    # dock frame, under the body beyond the CoM
# R2: magnets in the FLOOR LAND under the neck (the body's -x face is the open jaw channel: no room for a washer)
# R3: ONE magnet pair (tool magnet flush in the neck underside, dock magnet 1 mm deep in the land, opposite poles):
# an M8 washer's hole sat under half of each magnet (R3 note).  x 4.0: 0.9 mm wall front and back in the 8 mm land.
MAG_XY = ((4.0, 0.0),)                                                 # dock frame
# 10/29 (owner: "make it so the magnets are built in mid print and match the iris pocket size since those were perfect"):
# CLOSED, SEALED pockets, magnet dropped in at ONE print pause per plate.  Iris v2 spec (the one owner called perfect), from
# internal design rules rules `press.fit.general` ("iris v2 uses 6.15 drop-in pockets sealed by a pause") and
# `print.pause.magnet` (6.15 CAD pocket for a 6x3 magnet; bottom skin = first layer 0.20; pocket top = pause-layer top; magnet top
# 0.12 = ONE layer below the pause-layer top; sealed by a bridge; steel bed holds it down) and an internal wire-bender design
# iris_test_v2/iris_cad2.py (MAG_Z0, MAG_Z1 = 0.20, 3.32 -> 3.12 deep for a 3.0 magnet; iris_geom2.MAG_POCKET_D = 6.15).
# This plate slices at 0.20 layers, so "one layer clearance" = 0.20 (pocket 3.2 / slot 6.2); pocket tops sit on a layer boundary.
LAYER = 0.20
MAG_CLR_Z = LAYER                                                      # magnet top one layer under the pause-layer top (iris: 0.12)
MAG_SKIN_DOCK = 0.60                                                   # roof over the dock magnet (3 layers) = the land top
MAG_SKIN_NECK = 0.40                                                   # one-line wall over the tool magnet's outer face (-x)
MAG_W = 3.15                                                           # neck slot width in x = 3.0 magnet + 0.15 (iris: 6.15 - 6.0)
MAG_RECESS = MAG_SKIN_DOCK                                             # dock magnet pressed UP against its roof: gap = skin (+ clearance if it sits low)
WASHER_Z = -(MAG_XY[0][0] + PLATE_GAP4 + B.TP_T)                         # tool z of the neck magnet (= -11)
TAIL_DROP = 0.3                                                        # tail rest 0.3 below level: neck land carries, no hyperstatic seat
BODY_HALF = 39.0                                                       # gripper body is 78 tall in the dock
MAG_D_NOM = 6.0
MAG_D, MAG_H = 6.15, 3.0                                               # 6x3 N52 in a 6.15 pocket (iris v2 drop-in), sealed at a print pause
ROTOR_TRAVEL, ROTOR_TONGUE = 4.5, 8.0
PL_ANG4 = (75.0, 195.0, 315.0)          # R1 v4 critic: 60/180/300 left 1.1 mm between a plunger pocket and a dowel groove
PL_GUIDE4 = 6.0                         # R2: guide back to 6 (L/D 1.2); preload ~10 N comes from a 10 mm free-length spring
SHAFT_TOP4 = -12.0                      # pinion shaft top (tool z) moved down so the rotor can travel 4.5 mm


def ring_v4():
    """v3.1 ring with the wider bore / windows, the relief ring and the moved dowel grooves."""
    zt, zb = B.Z_RING_TOP, B.Z_LIP_BOT
    b = Pos(0, 0, (zt + zb) / 2) * Cylinder(B.R_RING, zt - zb)
    b += Pos(0, 0, (zt + zb + 3.0) / 2) * Cylinder(B.R_RING + 1.5, zt - zb - 3.0)
    b -= Pos(0, 0, (zt + B.Z_LIP_TOP) / 2 + 0.01) * Cylinder(B.R_CAV, zt - B.Z_LIP_TOP + 0.02)
    b -= Pos(0, 0, (B.Z_LIP_TOP + zb) / 2) * Cylinder(R_BORE4, B.LIP_T + 2)
    from build123d import Cone
    b -= Pos(0, 0, zb + B.BORE_CHAMFER / 2) * Cone(R_BORE4 + B.BORE_CHAMFER, R_BORE4, B.BORE_CHAMFER)
    zd = B.Z_LIP_TOP + B.H_DOWEL
    for k in range(3):
        a0 = 120 * k
        b -= B.sector(R_BORE4 - 1, R_WIN4, a0 - B.WIN, a0 + B.WIN, zb - 1, B.Z_LIP_TOP + 0.01)
        b -= B.sector(R_BORE4 - 1, R_WIN4 + 1.7, a0 - B.WIN - 6, a0 + B.WIN + 6, zb - 0.01, zb + 1.0)
        steps = int(B.RAMP[1] - B.RAMP[0])
        for i in range(steps):
            h = B.H_CREST * (i + 1) / steps
            b += B.sector(RELIEF_RIN, B.R_CAV, a0 + B.RAMP[0] + i, a0 + B.RAMP[0] + i + 1.02, B.Z_LIP_TOP - 0.01, B.Z_LIP_TOP + h)
        b += B.sector(RELIEF_RIN, B.R_CAV, a0 + B.RAMP[1], a0 + B.PHI_STOP[0], B.Z_LIP_TOP - 0.01, B.Z_LIP_TOP + B.H_CREST)
        b += B.sector(RELIEF_RIN, B.R_CAV, a0 + B.PHI_STOP[0], a0 + B.PHI_STOP[1], B.Z_LIP_TOP - 0.01, B.Z_LIP_TOP + B.H_STOP)
        top = B.H_CREST + 0.8
        hw = (top - 0.3) / tan(radians(30))
        b -= Pos(0, 0, B.Z_LIP_TOP) * B.radial_prism([(-hw, top), (hw, top), (0, 0.3)], RELIEF_RIN - 0.3, B.R_CAV + 0.5, a0 + B.PHI_V)  # R1: no sliver at the bore
        stop = B.radial_prism([(B.STOP_T[0], -0.01), (B.STOP_T[1], -0.01), (B.STOP_T[1], B.H_STOP), (B.STOP_T[0], B.H_STOP)],
                              RELIEF_RIN, B.R_CAV + 0.3, a0 + B.PHI_V)
        b += Pos(0, 0, B.Z_LIP_TOP) * stop
        for s in (-1, 1):
            b -= B.radial_rod(B.DOWEL_D + 0.05, DOWEL_RIN4 - 0.2, DOWEL_RIN4 + B.DOWEL_L + 0.2, a0 + B.PHI_V, s * B.DOWEL_D / 2, zd)
    for a in PL_ANG4:
        x, y = B.R_PL * cos(radians(a)), B.R_PL * sin(radians(a))
        b -= Pos(x, y, zb + PL_GUIDE4 / 2 - 0.01) * Cylinder(B.PL_SHANK_D / 2 + 0.15, PL_GUIDE4 + 0.02)
        b -= Pos(x, y, (zt + zb + PL_GUIDE4) / 2 + 0.01) * Cylinder(B.PL_HEAD_D / 2 + 0.15, zt - zb - PL_GUIDE4 + 0.02)
    for a in B.RING_SCREW_ANG:
        x, y = B.RING_SCREW_R * cos(radians(a)), B.RING_SCREW_R * sin(radians(a))
        b -= Pos(x, y, zt - 2.85) * Cylinder(2.0, 5.7)
        b -= Pos(x, y, zt - 5.7 - 2.0) * Cylinder(1.3, 4.0)
    return b


def dowels_v4():
    zd = B.Z_LIP_TOP + B.H_DOWEL
    out = None
    for k in range(3):
        for s in (-1, 1):
            d = B.radial_rod(B.DOWEL_D, DOWEL_RIN4, DOWEL_RIN4 + B.DOWEL_L, 120 * k + B.PHI_V, s * B.DOWEL_D / 2, zd)
            out = d if out is None else out + d
    return out


def rotor_v4():
    """Rotor with a long Oldham tongue slot (8 mm) so it can slide 4.5 mm down against its spring and still drive."""
    h = B.ROTOR_TOP + B.TP_T + 2.0
    r = Pos(0, 0, B.ROTOR_TOP - h / 2) * Cylinder(B.ROTOR_R, h)
    r -= Pos(0, 0, B.ROTOR_TOP - B.SLOT_D / 2 + 0.01) * Box(B.SLOT_W, 2 * B.ROTOR_R + 1, B.SLOT_D)
    a = (B.SLOT_W / 2 + B.SLOT_LEAD) * sqrt(2)
    r -= Pos(0, 0, B.ROTOR_TOP) * Rot(0, 45, 0) * Box(a, 2 * B.ROTOR_R + 1, a)
    r -= Pos(0, 0, B.ROTOR_TOP - h + ROTOR_TONGUE / 2 - 0.01) * Box(2 * B.ROTOR_R + 1, 4.2, ROTOR_TONGUE)
    return r


def dock_post_v4():
    """v3.1 post with the 6 mm land (sides + floor), wider slot, wider plate gap, top bevel, tail rest + magnets."""
    T, Hh, top = B.WALL_T, B.DOCK_H, B.WALL_TOP
    hw, zf, zn = B.NECK_SQ / 2 + SLOT_CLR4, -B.NECK_SQ / 2, B.NECK_SQ / 2
    mouth = hw + (top - zn)
    wall = Pos(T / 2, 0, (top - Hh) / 2) * Box(T, 2 * B.WALL_HW, top + Hh)
    prof = [(-hw, zf), (hw, zf), (hw, zn), (mouth + 1, top + 1), (-mouth - 1, top + 1), (-hw, zn)]
    wall -= Pos(-1, 0, 0) * extrude(Plane.YZ * Polygon(*prof, align=None), amount=T + 2, dir=(1, 0, 0))
    # relief behind the 6 mm land: slot sides +1.5 mm, floor -1.5 mm, under the neck the floor ends at the land
    relief = [(-hw - 1.5, zf - 40), (hw + 1.5, zf - 40), (hw + 1.5, zn), (mouth + 2.5, top + 1), (-mouth - 2.5, top + 1), (-hw - 1.5, zn)]
    wall -= Pos(LAND, 0, 0) * extrude(Plane.YZ * Polygon(*relief, align=None), amount=T - LAND + 1, dir=(1, 0, 0))
    # arm-side chamfers around the slot outline (3 mm x 45)
    big = [(-hw - 3, zf - 0.01), (hw + 3, zf - 0.01), (hw + 3, zn), (mouth + 4, top + 1), (-mouth - 4, top + 1), (-hw - 3, zn)]
    wall -= extrude(Plane.YZ * Polygon(*big, align=None), amount=3.0, taper=45)
    # 45-deg bevel on the arm-side top edge
    bev = Pos(0, 0, top) * Rot(0, 45, 0) * Box(TOP_BEVEL * sqrt(2), 2 * B.WALL_HW + 2, TOP_BEVEL * sqrt(2))
    wall -= bev
    foot = Pos(25, 0, -Hh + 3) * Box(90, 2 * B.WALL_HW, 6)
    gus = None
    for sv in (-1, 1):
        g = Pos(T, sv * (B.WALL_HW - 3) + 3, -Hh + 6) * extrude(Plane.XZ * Polygon((0, 0), (45, 0), (0, 70), align=None), amount=6)
        gus = g if gus is None else gus + g
    # tail rest: rib under the tool body, 0.3 mm below level (the neck on the land carries; the rib stops tipping)
    tz = -BODY_HALF - TAIL_DROP
    rib = Pos((TAIL_X[0] + TAIL_X[1]) / 2, 0, (tz - Hh) / 2) * Box(TAIL_X[1] - TAIL_X[0], 20, tz + Hh)
    post = wall + foot + gus + rib
    for xm, ym in MAG_XY:   # CLOSED pocket under a 0.6 roof (= the land top); opens UPWARD in the print pose (foot down), magnet dropped in at the pause
        ztop = -B.NECK_SQ / 2 - MAG_SKIN_DOCK                          # pocket top = pause-layer top
        post -= Pos(xm, ym, ztop - (MAG_H + MAG_CLR_Z) / 2) * Cylinder(MAG_D / 2, MAG_H + MAG_CLR_Z)
    return post


SKIRT_H4, R_SKIRT_MOUTH4, R_SKIRT_OUT4 = 5.5, 40.0, 41.5   # r3: capture +-4 -> +-6 mm; bottom 0.7 mm short of the plate back


def skirt_v4():
    """Taller lead-in skirt (r3).  Same 3 mm glued collar as v3.1."""
    from build123d import Cone
    zb = B.Z_LIP_BOT
    s = Pos(0, 0, zb - SKIRT_H4 / 2 + 1.5) * Cylinder(R_SKIRT_OUT4, SKIRT_H4 + 3.0)
    s -= Pos(0, 0, zb + 1.5) * Cylinder(B.R_RING + 0.15, 3.02)
    s -= Pos(0, 0, zb - SKIRT_H4 / 2) * Cone(R_SKIRT_MOUTH4, B.R_SKIRT_IN, SKIRT_H4 + 0.02)
    return s


def plunger_v4():
    """v3.1 plunger (guide 6 mm, R2).  Preload ~10 N comes from a 10 mm free-length spring (was 11)."""
    z_head_bot = B.Z_LIP_BOT + PL_GUIDE4
    p = Pos(0, 0, z_head_bot + B.PL_HEAD_H / 2) * Cylinder(B.PL_HEAD_D / 2, B.PL_HEAD_H)
    zc = B.Z_LIP_BOT - B.PL_PROTRUDE + B.PL_BALL_D / 2
    p += Pos(0, 0, (z_head_bot + zc) / 2) * Cylinder(B.PL_SHANK_D / 2, z_head_bot - zc)
    p -= Pos(0, 0, zc) * Sphere(B.PL_BALL_D / 2 + 0.05)
    return p


def pinion_v4():
    """UNUSED since critic R2 (the rotor spring was impossible).  Kept only as a record of the r1 attempt."""
    pts = []
    for i in range(40):
        a = radians(i * 9); r = 11.0 if i % 2 == 0 else 9.0
        pts.append((r * cos(a), r * sin(a)))
    p = Pos(0, 0, B.Z0 - 13) * extrude(Polygon(*pts, align=None), amount=10)
    p += Pos(0, 0, (B.Z0 - 3 + SHAFT_TOP4) / 2) * Cylinder(4.0, SHAFT_TOP4 - (B.Z0 - 3))
    p += Pos(0, 0, SHAFT_TOP4 - 0.75) * Cylinder(8.0, 1.5)                               # spring seat collar
    p += Pos(0, 0, SHAFT_TOP4 + ROTOR_TONGUE / 2) * Box(2 * B.ROTOR_R - 3, 3.8, ROTOR_TONGUE)
    return p


def neck_magnet_slot():
    """Tool-neck magnet pocket, CLOSED: a 3.15 x 6.15 slot in the YZ plane (disc stands on edge, marked face toward -x = the dock),
    0.4 mm wall to the -x face, open UPWARD in the print pose (tool +z up, neck on top) so the magnet drops in at the pause;
    the neck above the slot seals it.  Floor = magnet bottom (centre z WASHER_Z), top = floor + 6.0 + one layer."""
    zf = WASHER_Z - MAG_D_NOM / 2
    h = MAG_D_NOM + MAG_CLR_Z
    x0 = -B.NECK_SQ / 2 + MAG_SKIN_NECK
    return Pos(x0 + MAG_W / 2, 0, zf + h / 2) * Box(MAG_W, MAG_D, h)


def neck_slot_top_z():
    return WASHER_Z - MAG_D_NOM / 2 + MAG_D_NOM + MAG_CLR_Z            # tool z of the slot top (= pause-layer top)


def gripper_body_v4():
    """v3.1 concept body + a 6x3 magnet pocket (flush) in the NECK underside (-x face; +x is up at the lock roll), right
    above the dock magnet.  (R2: the body's -x face is the open jaw channel; R3: magnet pair instead of a washer.)"""
    gb = B.gripper_body()
    gb -= neck_magnet_slot()                                           # sealed neck magnet (drop-in slot, open upward in the print pose)
    return gb


def main():
    (OUT / "stl").mkdir(parents=True, exist_ok=True)
    (OUT / "step").mkdir(parents=True, exist_ok=True)
    parts = {
        "master_ring_v4": B.to_gripper(ring_v4()),
        "master_dowels_v4": B.to_gripper(dowels_v4()),
        "dock_post_v4": dock_post_v4(),
        "gripper_body_v4": gripper_body_v4(),
        "master_skirt_v4": B.to_gripper(skirt_v4()),
    }
    for k, a in enumerate(PL_ANG4):
        parts[f"master_plunger{k}_v4"] = B.to_gripper(Pos(B.R_PL * cos(radians(a)), B.R_PL * sin(radians(a)), 0) * plunger_v4())
    info = {"parts": {}, "params": dict(R_BORE=R_BORE4, R_WIN=R_WIN4, RELIEF_RIN=RELIEF_RIN, DOWEL_RIN=DOWEL_RIN4,
                                         PLATE_GAP=PLATE_GAP4, SLOT_CLR=SLOT_CLR4, LAND=LAND, TOP_BEVEL=TOP_BEVEL,
                                         TAIL_X=TAIL_X, TAIL_DROP=TAIL_DROP, MAGNETS="2x N52 6x3 in the floor land", MAG_XY=MAG_XY,
                                         NECK_MAGNET="N52 6x3 flush in the neck underside (opposite pole to the dock magnet)", NECK_MAGNET_Z=WASHER_Z, MAG_RECESS=MAG_RECESS, ROTOR="v3.1 rotor (no spring, R2)",
                                         PL_ANG=PL_ANG4, PL_GUIDE=PL_GUIDE4, SPRING_INSTALLED=round(B.Z_RING_TOP - (B.Z_LIP_BOT + PL_GUIDE4 + B.PL_HEAD_H), 2),
                                         SPRING_FREE=10.0,
                                         PRELOAD_EACH_N_at_k5=round(5 * (10 - (B.Z_RING_TOP - (B.Z_LIP_BOT + PL_GUIDE4 + B.PL_HEAD_H))), 1))}
    for name, s in parts.items():
        export_stl(s, str(OUT / "stl" / f"{name}.stl"))
        bb = s.bounding_box()
        info["parts"][name] = {"cm3": round(s.volume / 1000, 2),
                               "bbox": [round(v, 1) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)]}
    for name in ("master_ring_v4", "dock_post_v4"):
        export_step(parts[name], str(OUT / "step" / f"{name}.step"))
    (OUT / "parts_v4.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()
