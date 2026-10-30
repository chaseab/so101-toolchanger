# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""STAMP2: single-servo FLIP stamp (Printy-style reveal) for the SO-101 v4 tool changer.  2026-10-30, heads-cool for pm-eng.
Run (Idle priority, 1.5 GB watchdog inside): python toolchanger/heads_x3/build_stamp2.py -> heads_x3/stamp2/
Nothing here is printed, bought or run on the arm.

WHAT IT DOES  Jaw servo -> rotor -> GT1 shaft -> HELIX DRIVE (helix_drive.py, 22.0 mm stroke) -> RACK hanging off the sleeve -> 14 t m1 PINION
(PD 14) on the DIE ROTOR.  22.0 / 7 = 3.14 rad: the die swings exactly 180 deg about a cross axle (y).  OPEN stop (swap/dock pose): die tucked UP
inside the yoke, face hidden.  CLOSED stop: die swung out and face DOWN, 25 mm below the axle.  The arm then just positions and presses: the press
load goes die -> axle dowels -> cheeks -> cap, not through the gears.  The servo does the one precise move (the flip); the arm only moves down.
Face: TPU cap (95A) with a raised ring + check mark, snapped over the PLA die block.  Ink: none included; dab on
a desk pad, or run dry.  Spring-free.
FRAME: TOOL frame (coupling face z = 0, tool along -z, +x = UP in the dock).  STL pose = OPEN (die up); *_closed.stl = die down.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import memwd
memwd.start(1.5e9)
import json
from math import radians, degrees, pi, cos, sin, atan2, hypot

import trimesh
from build123d import Cylinder, Pos, Rot, Box, Axis, fillet

from common3 import *   # noqa
from helix_drive import Drive, THETA_DEG, CAP, RAIL_X, RAIL_Y, PIN_HOLE_R
import jg2_gears as G

NT = 14
RP = NT * G.M / 2                        # 7.0 pitch radius
D = Drive(pi * RP)                       # 21.99 mm = half a pinion turn
S = D.s
ZA = -97.5                               # die axle
RACK = dict(x=RP, y=(12.0, 18.0), back=11.25)                     # pitch line x 7, tips at 6, root 8.25, back 11.25
TEETH_K = range(-8, 3)                                            # last unused tooth trimmed (critic R2 nit): rack tail 3 mm shorter                                            # tooth centres ZA + k*pi (OPEN pose)
BRK = dict(x=(6.0, RACK["back"]), y=(8.0, RACK["y"][0]), h=4.0, scr_x=8.6, scr_dz=(6.0, -1.0))   # bracket under the sleeve (r >= 10 clears the drum); the RACK bolts to its +y face (critic R1 #6: sleeve prints upright, rack on its side)
ARM = dict(D=18.0, hub_r=5.0, hub_y=(-11.0, RACK["y"][0]), x=(-3.0, 3.0), y=(-9.0, 9.0))
DIE = dict(x=(-13.0, 13.0), y=(-11.0, 11.0), t=8.0)
CAPT = dict(face=3.0, glyph=1.5, scr=[(sx * 10.5, sy * 8.5) for sx in (1, -1) for sy in (1, -1)], head_pocket=1.8)   # TPU face plate, 4 recessed M3x8
CHEEK = dict(t=3.0, ym=(-15.4, -12.4), yp=(18.5, 21.5), x=(-8.0, 8.0), bot=ZA - 8.0)
GUIDE = dict(x=(RACK["back"] + 0.35, RACK["back"] + 3.2), z=(-118.0, ZA + 14.0))       # bottom stays 6 mm above the closed stamp face (-124)
AXLE_SLIP_R, AXLE_PRESS_R = 3.4 / 2, PIN_HOLE_R          # slip 0.2/side (fit.slip.pla; critic R1 #7)


def cyl_y(r, y0, y1, x=0.0, z=ZA):
    return Pos(x, (y0 + y1) / 2, z) * Cylinder(r, abs(y1 - y0), rotation=(90, 0, 0))


def frame():
    f = D.cap() + D.rails()
    zr = D.rail_bot
    # -y: rail continues straight down as the -y cheek
    f += bx(RAIL_X, CHEEK["ym"], (zr + 0.01, ZA + 12.0))
    f += bx(CHEEK["x"], CHEEK["ym"], (ZA + 12.0 + 0.01, CHEEK["bot"]))
    # +y: rail jogs out over the rack to the +y cheek
    f += bx(RAIL_X, (RAIL_Y[0], CHEEK["yp"][1]), (zr + 0.01, zr - 3.0))
    f += bx(RAIL_X, CHEEK["yp"], (zr - 2.99, ZA + 14.0))
    f += bx((CHEEK["x"][0], GUIDE["x"][1]), CHEEK["yp"], (ZA + 14.01, CHEEK["bot"]))
    f += bx(GUIDE["x"], (RACK["y"][0], CHEEK["yp"][0] + 0.01), GUIDE["z"])                 # rack back-up guide (keeps the mesh)
    f -= cyl_y(AXLE_PRESS_R, CHEEK["ym"][0] - 1, CHEEK["ym"][1] + 0.01)
    f -= cyl_y(AXLE_PRESS_R, CHEEK["yp"][0] - 0.01, CHEEK["yp"][1] + 1)
    return f


def carriage():
    c = D.sleeve()
    c += bx(BRK["x"], BRK["y"], (D.bot - BRK["h"], D.top)) - Pos(0, 0, (D.top + D.bot) / 2) * Cylinder(8.3, 20)   # bracket up the sleeve's side; 4 below it (0.8 over the stowed die)
    for dz in BRK["scr_dz"]:
        c -= cyl_y(1.35, BRK["y"][0] - 0.1, BRK["y"][1] + 0.1, BRK["scr_x"], D.bot + dz)      # horizontal pilot 2.7 (carriage prints flipped)
    return c


def rack():
    centres = [pi * k for k in TEETH_K]
    r = G.rack(centres, RACK["y"][1] - RACK["y"][0], RACK["back"] - (RACK["x"] + 1.25))
    r = Pos(RACK["x"], RACK["y"][1], ZA) * Rot(90, 0, 0) * r
    z_hi = ZA + pi * max(TEETH_K) + pi / 2
    z_lo = ZA + pi * min(TEETH_K) - pi / 2
    r += bx((RACK["x"] + 1.25, RACK["back"]), RACK["y"], (z_hi - 0.01, D.bot - BRK["h"] + 0.01))
    r += bx(BRK["x"], RACK["y"], (D.bot - BRK["h"], D.top))                            # bolting pad beside the bracket
    r += bx((RACK["x"] + 1.25, RACK["back"]), RACK["y"], (z_lo + 0.01, z_lo - 2.0))
    for dz in BRK["scr_dz"]:
        r -= cyl_y(1.7, RACK["y"][0] - 0.1, RACK["y"][1] + 0.1, BRK["scr_x"], D.bot + dz)
    return r


def glyph_2d_paper(h=CAPT["glyph"]):
    """ring + check mark as seen on the paper (top view, x right, y up), raised h (solids z 0..h)."""
    g = Pos(0, 0, h / 2) * (Cylinder(9.6, h) - Cylinder(8.2, h + 0.2))
    pts = [(-5.5, 0.5), (-1.8, -3.6), (6.0, 4.6)]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        L = hypot(x1 - x0, y1 - y0)
        g += Pos((x0 + x1) / 2, (y0 + y1) / 2, h / 2) * Rot(0, 0, degrees(atan2(y1 - y0, x1 - x0))) * Box(L + 2.4, 2.4, h)
    return g


def die_cap():
    """TPU face plate, UP pose (face +z), printed flat glyph-up.  The flip maps (x, y, z) -> (-x, y, -z), so the glyph is drawn mirrored in x here."""
    z0 = ZA + ARM["D"] + DIE["t"] / 2                    # die block top
    c = rbox(DIE["x"], DIE["y"], (z0, z0 + CAPT["face"]), r_v=2.0)
    g = glyph_2d_paper().mirror(Plane.YZ)
    c += Pos(0, 0, z0 + CAPT["face"] - 0.01) * g
    for x, y in CAPT["scr"]:
        c -= Pos(x, y, z0 + CAPT["face"] / 2) * Cylinder(1.7, CAPT["face"] + 1)
        c -= Pos(x, y, z0 + CAPT["face"] - CAPT["head_pocket"] / 2 + 0.01) * Cylinder(3.4, CAPT["head_pocket"])
    return c


def die_rotor():
    """hub along y + 14 t pinion + arm + die block (UP pose).  Two slip bores for the axle dowels, 4 self-tap pilots for the face plate."""
    u = cyl_y(ARM["hub_r"], ARM["hub_y"][0], ARM["hub_y"][1])
    p = Pos(0, RACK["y"][0], ZA) * Rot(-90, 0, 0) * Rot(0, 0, 180.0 / NT) * G.spur(NT, RACK["y"][1] - RACK["y"][0])
    u += p
    zt = ZA + ARM["D"] - DIE["t"] / 2
    u += bx(ARM["x"], ARM["y"], (ZA, zt + 0.01))
    u += rbox(DIE["x"], DIE["y"], (zt, zt + DIE["t"]), r_v=2.0)
    for x, y in CAPT["scr"]:
        u -= Pos(x, y, zt + DIE["t"] - 3.5) * Cylinder(1.35, 7.02)      # horizontal pilot 2.7 (rotor prints on its -y face)
    u -= cyl_y(AXLE_SLIP_R, ARM["hub_y"][0] - 1, -3.0)
    u -= cyl_y(AXLE_SLIP_R, 9.0, RACK["y"][1] + 1)
    return u


def flip(shape, s):
    """pose of the rotating die parts for a carriage rise s (0 = OPEN)."""
    b = -degrees(s / RP)
    return Pos(0, 0, ZA) * Rot(0, b, 0) * Pos(0, 0, -ZA) * shape


def main():
    hd = Head("STAMP2", "stamp2")
    hd.coupling_parts()
    OPEN = -THETA_DEG
    for n in ("rotor", "shaft"):
        hd.parts[n]["shape"] = Rot(0, 0, OPEN) * hd.parts[n]["shape"]
    FR, CA0, RK, HUB, DR, DC = frame(), carriage(), rack(), D.hub(OPEN), die_rotor(), die_cap()
    CA = CA0 + RK
    hd.printed("housing", housing(), color="pla_white", label="light housing + dock keel (GT1 neck)")
    hd.printed("hub", HUB, color="pla_black", label="hub: stop dog + helix drum (2 grooves)")
    hd.printed("frame", FR, color="pla_white", label="frame: cap + pin rails + yoke cheeks + rack guide")
    hd.printed("carriage", CA0, color="pla_black", label="carriage: sleeve + rack bracket (moves 22.0 mm)")
    hd.printed("rack", RK, color="pla_black", label="rack m1, 2x M3x10 to the carriage bracket")
    hd.printed("die_rotor", DR, color="pla_white", label="die rotor: 14 t pinion + arm + die block (flips 180 deg)")
    hd.printed("die_cap", DC, mat="tpu", color="tpu", label="TPU stamp face 26 x 22 x 3: ring + check mark raised 1.5")
    pins = D.pins()
    hd.bought("drive_pins", pins, 2 * 0.67, "steel", "2x 3x12 steel dowel: groove followers + anti-rotation keys (pressed 3.05 in the sleeve)")
    ax = cyl_y(1.5, CHEEK["ym"][0], CHEEK["ym"][0] + 12.0) + cyl_y(1.5, CHEEK["yp"][1] - 12.0, CHEEK["yp"][1])
    hd.bought("axle_dowels", ax, 2 * 0.67, "steel", "2x 3x12 steel dowel: die axle (pressed 3.05 in the cheeks, slip 3.2 in the rotor)")
    ax_dn = (0, 0, -1)
    hd.add_hw("shaft_screw", "M3x8 button", (0.0, 0.0, -33.5), ax_dn, 8.0, "bolt", "hub")
    hd.add_hw("shaft_washer", "M3 washer", (0.0, 0.0, -33.0), ax_dn, 0.5, "washer", "hub")
    for i, (x, y) in enumerate(INS):
        hd.add_hw(f"housing_insert{i + 1}", "M3 heat-set insert 4.0 bore x 6.7", (x, y, HOUS_Z[1]), ax_dn, 5.7, "insert", "housing")
        hd.add_hw(f"cap_screw{i + 1}", "M3x6 button", (x, y, CAP[1] + 1.2), ax_dn, 6.0, "bolt", "frame")
    z0 = ZA + ARM["D"] + DIE["t"] / 2
    for i, (x, y) in enumerate(CAPT["scr"]):
        hd.add_hw(f"face_screw{i + 1}", "M3x8 button (self-taps into the die block)", (x, y, z0 + CAPT["face"] - CAPT["head_pocket"]), (0, 0, 1), 8.0, "bolt", "die_cap")
    for i, dz in enumerate(BRK["scr_dz"]):
        hd.add_hw(f"rack_screw{i + 1}", "M3x10 button (self-taps into the carriage bracket)", (BRK["scr_x"], RACK["y"][1], D.bot + dz), (0, 1, 0), 10.0, "bolt", "rack")
    face_open = ZA + ARM["D"] + DIE["t"] / 2 + CAPT["face"] + CAPT["glyph"]
    ext = {"drive": D.forces(), "pinion": {"teeth": NT, "module": G.M, "pitch_radius_mm": RP, "flip_deg": round(degrees(S / RP), 1),
                                           "pinion_torque_est_Nm": round(D.forces()["force_est_N"] * RP / 1000, 3)},
           "stamp_face_z_closed": round(2 * ZA - face_open, 1), "die_top_z_open": round(face_open, 1),
           "poses": "STL = OPEN (die up, hidden); *_closed.stl = CLOSED (hub +120.3 deg, carriage +22.0, die rotor -180 deg about y)"}
    out = hd.finish(extra=ext)
    print({k: out[k] for k in ("mass_g", "com_dist_from_coupling_face_mm", "coupling_moment_Nmm", "envelope_mm")}, ext["stamp_face_z_closed"])
    for n, p in out["parts"].items():
        print(n, p["g"], p["watertight"], p["bbox"])
    st = hd.dir / "stl"
    Tz = trimesh.transformations.rotation_matrix(radians(THETA_DEG), [0, 0, 1])
    Tc = trimesh.transformations.translation_matrix([0, 0, S])
    Tf = trimesh.transformations.rotation_matrix(-S / RP, [0, 1, 0], point=[0, 0, ZA])
    for n, T in (("hub", Tz), ("rotor", Tz), ("shaft", Tz), ("carriage", Tc), ("rack", Tc), ("drive_pins", Tc), ("die_rotor", Tf), ("die_cap", Tf)):
        m = trimesh.load(str(st / f"{n}.stl")); m.apply_transform(T); m.export(str(st / f"{n}_closed.stl"))
    for hid in ("shaft_screw", "shaft_washer", "rack_screw1", "rack_screw2") + tuple(f"face_screw{i + 1}" for i in range(4)):
        m = trimesh.load(str(st / f"hw_{hid}.stl")); m.apply_transform(Tz if hid.startswith("shaft") else Tc if hid.startswith("rack") else Tf); m.export(str(st / f"hw_{hid}_closed.stl"))
    # ---- interference through the stroke
    V = lambda s: 0.0 if s is None else round(float(s.volume), 3)
    chk = {}
    DIEU = DR + DC
    for f in (0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0):
        s = f * S
        ph = OPEN + f * THETA_DEG
        C, Rr = Pos(0, 0, s) * CA, flip(DIEU, s)
        tag = f"{int(f * 100)}%"
        chk[f"pinion&rack {tag}"] = V(flip(DR, s) & C)
        chk[f"die&frame {tag}"] = V(Rr & FR)
        chk[f"carriage&frame {tag}"] = V(C & FR)
        chk[f"pins&hub {tag}"] = V((Pos(0, 0, s) * pins) & D.hub(ph))
        chk[f"hub&carriage {tag}"] = V(D.hub(ph) & C)
    for f in (0.0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0):          # critic R2 #6: die (with face plate) vs carriage + rack through the stroke
        s = f * S
        chk[f"die+face&carriage+rack {int(f * 100)}%"] = V(flip(DIEU, s) & (Pos(0, 0, s) * CA))
    chk["die rotor&face plate"] = V(DR & DC)
    chk["axle&frame"] = V(ax & FR)
    chk["axle&die rotor"] = V(ax & DR)
    chk["rack&carriage (bolted, touching)"] = V(RK & CA0)
    chk["housing&carriage closed"] = V(housing() & (Pos(0, 0, S) * CA))
    print("INTERFERENCE", json.dumps(chk, indent=1))
    pj = json.loads((hd.dir / "parts.json").read_text())
    pj["interference_mm3"] = chk
    (hd.dir / "parts.json").write_text(json.dumps(pj, indent=1))


if __name__ == "__main__":
    main()
