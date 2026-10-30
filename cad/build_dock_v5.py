# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Dock v5 retrofit kit (pm-eng / dock-cradle, 2026-10-29).  Dock frame: origin on the wall's arm-side face at the tool axis,
+x away from the arm, +z up, table z=-100, foot top z=-94.  Does NOT touch build_changer_v4.py / build_gripper_tool.py.
R1 kit (print now, tray 0 white, no cutting, no inserts):  2 stop pads (tape on the wall face) + 2 cradle columns (tape on the foot).
R2 option (needs ONE jigsaw cut of the old wall at z=-24): new slot head, land 8 mm, 1.0 mm/side, 4 mm lead-in chamfers.
Run: python toolchanger/cad/dock_v5/build_dock_v5.py"""
import guard  # noqa: memory watchdog
import json
from math import atan, degrees, sqrt, sin, cos, radians
from pathlib import Path
import numpy as np, trimesh
from build123d import Box, Pos, Polygon, Plane, extrude, export_stl, export_step, Compound

H = Path(__file__).resolve().parent
(H / "stl").mkdir(exist_ok=True); (H / "step").mkdir(exist_ok=True)
FOOT_TOP, TAPE = -94.0, 0.4
BODY_BOT = -39.0
# ---------- R1 stop pad: plate back face is at x=-2 (2 mm gap).  pad 1.6 + tape 0.4 = 2.0
PAD_T, PAD_Y, PAD_Z = 1.58, (23.0, 31.0), (-8.0, 8.0)
def stop_pad():
    from build123d import Cylinder, Rot
    return Pos(-TAPE - PAD_T / 2, 27.0, 0) * Rot(0, 90, 0) * Cylinder(5.0, PAD_T)     # round tab D10: rotation-proof on the bed
# ---------- R1 cradle column (+y side; mirror for -y).  profile in (y,z), extruded along x 12.4 .. 36
COL_X = (12.4, 36.0)
Z0 = FOOT_TOP + TAPE
def column(sign=1):
    # ridge 15..18 at -39.0 (body flat bottom / cover rails at |y| 15.5..17.5); floor -41 between; guide cheek inner face 23.8, top -31, 3 mm lead-in
    pts = [(11.5, Z0), (26.3, Z0), (26.3, -31.0), (23.8, -31.0), (23.8 - 0.0, -34.0), (23.8, -41.0),   # cheek inner face (vertical below -34), lead-in is on the top
           (18.0, -41.0), (18.0, BODY_BOT), (15.0, BODY_BOT), (15.0, -41.0), (11.5, -41.0)]
    # lead-in: chamfer the cheek top inner corner 3 x 3 (outward)
    pts = [(11.5, Z0), (26.3, Z0), (26.3, -31.0), (26.3 - 0.0, -31.0), (23.8 + 0.0, -31.0 - 0.0)]
    pts = [(11.5, Z0), (26.3, Z0), (26.3, -31.0), (23.8, -31.0), (23.8, -41.0), (18.0, -41.0), (18.0, BODY_BOT),
           (15.0, BODY_BOT), (15.0, -41.0), (11.5, -41.0)]
    # cheek lead-in: slope the cheek top from (26.3,-31) down to (23.8,-34) is inward; use a 45-deg inner chamfer instead
    pts = [(11.5, Z0), (26.3, Z0), (26.3, -31.0), (25.0, -31.0), (23.8, -32.2), (23.8, -41.0), (18.0, -41.0), (18.0, BODY_BOT),
           (15.0, BODY_BOT), (15.0, -41.0), (11.5, -41.0)]
    pts = [(sign * y, z) for y, z in pts]
    prof = Polygon(*pts, align=None)
    s = extrude(Plane.YZ.offset(COL_X[0]) * prof, amount=COL_X[1] - COL_X[0])
    return s
# ---------- R2 head
def head(c=1.0, land=8.0, chamf=4.0):
    NECK, T, ZC, TOP = 36.0, 12.0, -24.0, 40.0
    hw, zf, zn = NECK / 2 + c, -NECK / 2, NECK / 2
    mouth = hw + (TOP - zn)
    hy = 62.0
    wall = Pos(T / 2, 0, (TOP + ZC) / 2) * Box(T, 2 * hy, TOP - ZC)
    prof = [(-hw, zf), (hw, zf), (hw, zn), (mouth + 1, TOP + 1), (-mouth - 1, TOP + 1), (-hw, zn)]
    wall -= Pos(-1, 0, 0) * extrude(Plane.YZ * Polygon(*prof, align=None), amount=T + 2)
    rel = [(-hw - 1.5, zf - 40), (hw + 1.5, zf - 40), (hw + 1.5, zn), (mouth + 2.5, TOP + 1), (-mouth - 2.5, TOP + 1), (-hw - 1.5, zn)]
    wall -= Pos(land, 0, 0) * extrude(Plane.YZ * Polygon(*rel, align=None), amount=T - land + 1)
    big = [(-hw - chamf, zf - 0.01), (hw + chamf, zf - 0.01), (hw + chamf, zn), (mouth + chamf + 1, TOP + 1), (-mouth - chamf - 1, TOP + 1), (-hw - chamf, zn)]
    wall -= extrude(Plane.YZ * Polygon(*big, align=None), amount=chamf, taper=45)
    # back skin 1.5 over the stump top (x 12..13.5, z -32..-24) and two front flanges outside the plate radius (|y| 40..58, z -42..-24, x -3..0)
    wall += Pos(12 + 0.75, 0, -28) * Box(1.5, 2 * hy, 8)
    for s in (-1, 1):
        wall += Pos(-1.5, s * 49, -33) * Box(3, 18, 18)
    return wall
# ---------- (a) radial T-gauge: crossbar flush on DB1's front face (x=-35, y +-80), V nose in the front V-notch, stem points at the pan axis
def t_gauge():
    """Own frame = DB1 frame coords, z up from the table (z=-100).  Crossbar x -41..-35 (6), y +-80, z 8 tall; stem x -185..-41 (to the arm), 14 wide, 4 thick;
    V nose 90 deg, 2.6 deep, on the crossbar's +x face at y=0 (DB1 notch = 4x4 square at 45 deg); two 2.2 mm thread holes in the stem at 30 and 140 mm from the crossbar."""
    zc = -100.0
    g = Pos(-38.0, 0, zc + 4) * Box(6, 160, 8)
    g += Pos(-(41 + 144 / 2), 0, zc + 2) * Box(144, 14, 4)
    nose = Pos(-35 + 1.3, 0, zc + 4) * extrude(Plane.XY * Polygon((-1.3, -2.6), (1.3, 0), (-1.3, 2.6), align=None), amount=8, dir=(0, 0, 1))
    g += Pos(0, 0, -4) * nose
    from build123d import Cylinder
    for d in (30, 140):
        g -= Pos(-(35 + d), 0, zc + 2) * Cylinder(1.1, 6)
    return g
def to_mesh(shape, name):
    export_stl(shape, str(H / "stl" / f"{name}.stl")); export_step(shape, str(H / "step" / f"{name}.step"))
    return trimesh.load(str(H / "stl" / f"{name}.stl"))
from build123d import mirror
pad = stop_pad(); colp = column(1); colm = mirror(colp, Plane.XZ)
parts = {"t_gauge": t_gauge(), "stop_pad": pad, "cradle_col_R": colp, "cradle_col_L": colm, "head_R2": head()}
meshes = {k: to_mesh(v, k) for k, v in parts.items()}
# ---- numbers
c0, L0 = 0.4, 12.0
res = {"jam_bench_c0.4_L12": degrees(atan(2 * c0 / L0)), "jam_CAD_c0.4_flat5": degrees(atan(2 * c0 / 5.0)),
       "R2_c1.0_L8": degrees(atan(2 * 1.0 / 8.0)), "R2_c1.0_flat4": degrees(atan(2 * 1.0 / 4.0)),
       "c1.0_L12": degrees(atan(2 * 1.0 / 12.0)), "c1.0_L6": degrees(atan(2 * 1.0 / 6.0))}
def roll_slack(s, W):   # square s in slot W: sin a + cos a = W/s
    from scipy.optimize import brentq
    return degrees(brentq(lambda a: sin(a) + cos(a) - W / s, 1e-6, 0.7))
res["roll_slack_c0.4_deg"] = roll_slack(36, 36.8); res["roll_slack_c1.0_deg"] = roll_slack(36, 38.0)
res["mass_cm3"] = {k: round(m.volume / 1000, 2) for k, m in meshes.items()}
# mass estimate: DK1S 194.4 g / 263.17 cm3 = 0.739 g/cm3 at 4 walls + 40 % gyroid
res["g_per_cm3_40pct"] = 194.4 / 263.17
json.dump(res, open(H / "numbers.json", "w"), indent=1)
print(json.dumps(res, indent=1))
# ---- reference meshes for the viewer / gate (cheap: no contains(), no booleans on meshes)
def dock(m):
    V = m.vertices; return trimesh.Trimesh(np.c_[-V[:, 2] - 7, V[:, 1], V[:, 0]], m.faces[:, ::-1])
GP = H.parent / "v4" / "gate_parts"
tool = trimesh.util.concatenate([dock(trimesh.load(str(GP / f"{n}_asm.stl"))) for n in
        ["gt_body", "gt_cover", "gt_plate", "gt_jaw1", "gt_jaw2", "gt_finger1", "gt_finger2", "gt_pinion", "gt_shaft", "gt_rotor", "gt_pins"]])
tool.export(str(H / "stl" / "ref_tool_in_dock.stl"))
trimesh.load(str(H.parent / "v4" / "stl_open_pockets_bak" / "dock_post_v4.stl")).export(str(H / "stl" / "ref_dock_post_asbuilt.stl"))
