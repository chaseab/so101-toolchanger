# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""v4 geometry checks on the real CAD solids (exact OCC booleans) + the lock path on the exported ring mesh.

1. Master (ring_v4 + dowels_v4 + skirt + plungers/balls + carrier) vs tool (plate + pins + rotor_v4) at ENTRY,
   mid-ramp, CREST and LOCK, and with the tool offset 0.8 mm sideways at entry (the new bore clearance).
2. Spigot vs the cavity features: the relief ring must clear the spigot by >= 0.8 mm (the reason for v4).
3. Tool (plate + neck + body) vs dock_post_v4 along the ASSEMBLY PATH: lowered from 35 mm above the seat to the seat,
   seated, pitched +-2 deg about the land (arm droop), slid 2 mm toward the arm; body underside vs the tail rest.
4. Lock path (pin-centre height needed vs twist angle) on the v4 ring mesh.
Run: python toolchanger/cad/v4_checks.py  -> v4/checks_v4.json
"""
import json
from pathlib import Path

import numpy as np
import trimesh
from build123d import Pos, Rot

import build_changer as B
import build_changer_v4 as V

H = Path(__file__).resolve().parent
out = {}
mp = B.master_parts()
M = {k: B.to_gripper(v) for k, v in mp.items() if k in ("carrier", "ball0", "ball1", "ball2", "plunger0", "plunger1", "plunger2")}
M = {k: v for k, v in M.items() if not k.startswith("plunger")}
for k, a in enumerate(V.PL_ANG4):
    M[f"plunger{k}_v4"] = B.to_gripper(Pos(B.R_PL * __import__("math").cos(__import__("math").radians(a)), B.R_PL * __import__("math").sin(__import__("math").radians(a)), 0) * V.plunger_v4())
    M[f"ball{k}"] = B.to_gripper(Pos(B.R_PL * __import__("math").cos(__import__("math").radians(a)), B.R_PL * __import__("math").sin(__import__("math").radians(a)), B.Z_LIP_BOT - B.PL_PROTRUDE + B.PL_BALL_D / 2) * __import__("build123d").Sphere(B.PL_BALL_D / 2))
M["ring_v4"] = B.to_gripper(V.ring_v4())
M["skirt_v4"] = B.to_gripper(V.skirt_v4())
out_skirt = {"skirt_bottom_z": round(B.Z_LIP_BOT - V.SKIRT_H4, 2), "plate_back_z_locked": round(B.Z_TOOL_FACE_LOCK - B.TP_T, 2),
             "clearance_to_wall_when_plate_on_wall_mm": round((B.Z_LIP_BOT - V.SKIRT_H4) - (B.Z_TOOL_FACE_LOCK - B.TP_T), 2)}
M["dowels_v4"] = B.to_gripper(V.dowels_v4())
T = {"plate": B.coupling_plate(), "pins": B.pins(), "rotor": B.rotor()}      # R2: v3.1 rotor (no spring)
states = {"entry": (-45, B.Z_TOOL_FACE_ENTRY, 0.0), "entry_offset_0.8": (-45, B.Z_TOOL_FACE_ENTRY, 0.8),
          "ramp_27": (-18, B.Z_TOOL_FACE_ENTRY + 0.45, 0.0), "crest_39": (-6, B.Z_TOOL_FACE_ENTRY + B.LIFT_CREST, 0.0),
          "lock": (0, B.Z_TOOL_FACE_LOCK, 0.0)}
ov = {}
for s, (rot, zf, dy) in states.items():
    for tn, tm in T.items():
        if tn == "rotor" and s != "lock":
            continue
        t = Pos(B.HORN_CX, dy, zf) * Rot(0, 0, rot) * tm
        for mn, mm in M.items():
            v = (t & mm).volume
            if v > 0.01:
                ov[f"{s}: tool {tn} x master {mn}"] = round(v, 3)
out["coupling_overlaps_mm3"] = ov or "none > 0.01 mm3"
out["skirt_v4"] = out_skirt

# 2. spigot clearance to the relief ring / dowel ends (radial, design numbers)
out["spigot_clearance_mm"] = {"bore": round(V.R_BORE4 - B.R_SPIG, 2), "relief_ring": round(V.RELIEF_RIN - B.R_SPIG, 2),
                              "dowel_ends": round(V.DOWEL_RIN4 - B.R_SPIG, 2),
                              "v3.1_bore/ramps/dowels": [round(B.R_BORE - B.R_SPIG, 2), round(B.R_BORE - B.R_SPIG, 2), round(B.DOWEL_RIN - B.R_SPIG, 2)]}

# 2b. (R2) magnet target: the washer pocket must REMOVE material (a pocket cut into the open jaw channel removes
# nothing - the R2 critic found 11.6 of 98 mm3) and sit right above both land magnets; magnet pockets must not merge
import math as _m
from build123d import Cylinder as _Cyl
gb3, gb4 = B.gripper_body(), V.gripper_body_v4()
pocket = _m.pi * (V.MAG_D / 2) ** 2 * V.MAG_H
out["neck_magnet_pocket_removed_mm3"] = {"removed": round(gb3.volume - gb4.volume, 1), "pocket": round(pocket, 1)}
xm = V.MAG_XY[0][0]
out["dock_magnet_pocket"] = {"x": [round(xm - V.MAG_D / 2, 2), round(xm + V.MAG_D / 2, 2)], "land_x": [0, V.LAND],
                             "wall_front": round(xm - V.MAG_D / 2, 2), "wall_back": round(V.LAND - xm - V.MAG_D / 2, 2)}
dock_solid = V.dock_post_v4()
zt = -B.NECK_SQ / 2 - V.MAG_RECESS
mag_vol = sum((Pos(x, y, zt - V.MAG_H / 2 + 0.02) * _Cyl(3.0, V.MAG_H - 0.04) & dock_solid).volume for x, y in V.MAG_XY)
out["dock_magnet_x_dock_mm3 (0 = fits)"] = round(mag_vol, 3)
neck_x = -V.WASHER_Z - V.PLATE_GAP4 - B.TP_T
out["neck_magnet_over_dock_magnet"] = {"neck_x": round(neck_x, 2), "dock_x": xm, "aligned": abs(neck_x - xm) < 0.01,
                                       "gap_mm": V.MAG_RECESS}

# 2c. plunger pockets vs dowel grooves / ring screws: plunger_v4 parts must not overlap the ring, balls clear
out["plunger_v4_x_ring_mm3"] = round(sum((M[f"plunger{k}_v4"] & M["ring_v4"]).volume for k in range(3)), 3)

# 3. tool vs dock along the assembly path (dock frame: x along the tool axis into the wall, z up)
dock = V.dock_post_v4()
tool = B.coupling_plate() + B.gripper_body()
face_x = -(V.PLATE_GAP4 + B.TP_T)


def tool_in_dock(dx=0.0, dz=0.0, pitch=0.0):
    # tool frame -> dock frame: tool z = -x_dock, tool x = +z_dock (+x is UP at the lock roll, R1 fix), tool y = y
    # pitch: about the land's far top edge (x = LAND, z = -18)
    t = Pos(face_x + dx, 0, dz) * Rot(0, -90, 0) * tool
    if pitch:
        t = Pos(V.LAND, 0, -B.NECK_SQ / 2) * Rot(0, pitch, 0) * Pos(-V.LAND, 0, B.NECK_SQ / 2) * t
    return t


path = {}
for dz in (35, 25, 18, 10, 5, 2, 0.5, 0.0):
    path[f"lower dz={dz}"] = round((tool_in_dock(0, dz) & dock).volume, 3)
for p in (-2, -1, 1, 2):
    path[f"seated pitch {p:+d} deg (tool 0.3 mm up)"] = round((tool_in_dock(0, 0.3, p) & dock).volume, 3)
    path[f"seated pitch {p:+d} deg (on the land)"] = round((tool_in_dock(0, 0.0, p) & dock).volume, 3)
for dx in (-2, 1.5):
    path[f"seated slid dx={dx}"] = round((tool_in_dock(dx, 0) & dock).volume, 3)
for dz in (2, 8, 20):
    path[f"lift pitched 2 deg dz={dz}"] = round((tool_in_dock(0, dz, 2) & dock).volume, 3)
out["dock_path_overlap_mm3"] = path
bb = tool_in_dock().bounding_box()
out["seated_tool_bbox_dock"] = [round(v, 1) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)]
out["tail_rest_top_vs_body_underside_mm"] = round(-V.BODY_HALF - bb.min.Z, 2)
# 4. lock path on the v4 ring + dowel mesh
from build123d import export_stl
tmp = H / "v4" / "stl"
ring = trimesh.util.concatenate([trimesh.load(tmp / "master_ring_v4.stl"), trimesh.load(tmp / "master_dowels_v4.stl")])
ring.apply_translation([-B.HORN_CX, 0, 0])
rmax, rpin = B.PIN_RIN + B.PIN_L, B.PIN_D / 2
rs = np.linspace(V.R_BORE4 + 0.2, rmax, 16)
ts = np.linspace(-rpin * 0.98, rpin * 0.98, 13)
rows = []
for phi in np.arange(-20, 66.5, 1.0):
    a = np.radians(phi)
    ur, ut = np.array([np.cos(a), np.sin(a), 0]), np.array([-np.sin(a), np.cos(a), 0])
    O = np.array([r * ur + t * ut + [0, 0, B.Z_LIP_TOP + 9.0] for r in rs for t in ts])
    D = np.tile([0, 0, -1.0], (len(O), 1))
    loc, idx, _ = ring.ray.intersects_location(O, D, multiple_hits=False)
    need = -1e9
    for (pp, i) in zip(loc, idx):
        t = ts[i % len(ts)]
        need = max(need, pp[2] - B.Z_LIP_TOP + np.sqrt(rpin ** 2 - t ** 2))
    rows.append((float(phi), None if need < -1e8 else round(float(need), 3)))
out["lockpath_pin_centre_needed"] = rows
out["lockpath_design"] = {"entry": B.PIN_C_ENTRY, "crest": B.PIN_C_CREST, "lock": round(B.PIN_C_LOCK, 3)}
(H / "v4" / "checks_v4.json").write_text(json.dumps(out, indent=1))
print(json.dumps({k: v for k, v in out.items() if k != "lockpath_pin_centre_needed"}, indent=1))
print("lockpath (deg: need):", [r for r in rows if r[0] % 5 == 0])
