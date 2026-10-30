# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""JG2: rack-and-pinion Jenga poker for the SO-101 v4 tool changer.  2026-10-30, pm-eng CAD worker.
Run (Idle priority, watchdog inside): python toolchanger/heads_x3/build_jg2.py -> heads_x3/jg2/
Nothing here is printed, bought or run on the arm.

WHAT IT DOES  owner 10/30: "does the poker extend when the last servo moves".  JG1 did not (passive magnet carriage, shaft locked by the stop dog).
JG2 drives the poker from the jaw servo's rotary input:  rotor -> GT1 shaft -> HUB (= PEN1 stop dog + spindle) -> miter bevel A (16 t, m1)
-> miter bevel B (16 t) on a cross shaft -> spur pinion (16 t, m1, PD 16) -> rack on the CARRIAGE -> 7 x 5 rod + 14 x 8 pad (JG1's).
The stop dog no longer locks anything: it swings the full 120.3 deg between its two housing ribs, and those two ribs ARE the hard end stops
of the stroke (same as SCOOP1).  Stroke s = r*theta = 8 mm * 2.1 rad = 16.8 mm.  Jaw CLOSING (rotor toward the closed rib, shaft angle 0)
EXTENDS; jaw OPENING retracts, so the head parks retracted at the open stop where the swap routine leaves it.
Pose of every STL = EXTENDED (hub at the closed rib).  *_ret.stl = retracted (hub 120.3 deg back, cross shaft the same angle, carriage +16.8).
FRAME: TOOL frame (coupling face z = 0, tool along -z, +x = UP in the dock).

Gear axes: A on the tool axis (apex at z = ZA), B on a cross axis along -y through the same apex, pinion beside B, rack at x = +8.05 (pitch line).
Kinematic check (right-hand rule): A turns +w about +z  ->  B and pinion turn +w about +y  ->  the rack at +x moves -z (extends) for w > 0 (closing).
The STS3215 caps the swap torque at 0.25 N.m: F = T/r = 31 N before friction.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import memwd
    memwd.start(1.5e9)
except Exception as _e:   # watchdog is mandatory per the task; fail loudly
    raise
import json
from math import pi, radians
from pathlib import Path

import trimesh
from build123d import Box, Cylinder, Pos, Rot, Axis, Plane, extrude, RegularPolygon, export_stl

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common3 import *   # noqa
import jg2_gears as G   # noqa

ZA = -58.5                       # apex height = cross-shaft axis
RP = 8.0                         # pitch radius of A, B and the pinion (16 teeth, module 1)
NT = 16
H_OUT, H_IN = 8.0, 5.2           # bevel back / inner end (distance from the apex along the axis)
S_OPEN, R_P_GT = 21.0, 10.0
THETA = S_OPEN / R_P_GT          # rad: GT1 jaw range (120.3 deg)
STROKE = RP * THETA              # 16.8 mm
CAP = (-31.5, -34.5)
TUB = dict(x=(-10.4, 14.5), y=(-16.5, 10.5), z=(-34.5, -95.6))
CAVITY = dict(x=(-9.4, 13.0), y=(-16.8, 9.5), z=(-44.0, -93.6))
RACK_X = 8.0                     # pitch line of the rack = pinion pitch radius (0.2 mm backlash per gear is cut into the teeth)
RACK_Y = (-15.0, -9.5)           # = the pinion's tooth span
BAR_X_BACK = 12.7                # 0.3 mm to the cavity wall at 13.0
SLOT = dict(x=(6.9, 13.0), y=(-15.3, -9.2), z_top=-35.4)
BAR_TOP = -53.0                  # extended pose; retracted = -36.2
BRIDGE = dict(x=(-3.35, BAR_X_BACK), y=(RACK_Y[0], 3.7), z=(-92.8, -85.8))
ROD = dict(w=7.0, h=5.0, z1=-126.0 - STROKE, clr=0.25, top=-87.8)    # z1 = extended pad plane (retracted -126 as JG1)
PAD = dict(w=14.0, h=8.0, t=3.0)
GUIDE = dict(x=(-5.0, 5.0), y=(-7.5, 7.5), z=(-110.0, TUB["z"][1]))
LID = dict(y=(-18.5, -16.5), x=(-10.4, 18.5), z=(-44.0, -95.6))
LID_SCREWS = [(-6.6, -47.6), (-6.6, -90.5), (14.6, -47.6), (14.6, -90.5)]    # (x, z), axis +y into the tub
SPN = dict(r=5.0, z1=-50.0, bore_r=3.4, web=(-31.0, -33.0))
COLLAR = dict(r=6.2, z=(-46.5, -50.5), bore_r=5.1, pin_z=-48.5)
CAP_INS = [p for p in INS if p != (14.0, -14.0)]     # (14,-14) is under the rack column: no screw there
BOSS = dict(r=3.5, y1=-3.0, bore_r=1.65, bore_y=(-3.0, 0.0))
TEETH_K = range(-7, 2)           # rack tooth centres at z = ZA + k*pi (extended pose)


def cyl_y(r, y0, y1, x, z):
    return Pos(x, (y0 + y1) / 2, z) * Cylinder(r, abs(y1 - y0), rotation=(90, 0, 0))


def cyl_x(r, x0, x1, y, z):
    return Pos((x0 + x1) / 2, y, z) * Cylinder(r, abs(x1 - x0), rotation=(0, 90, 0))


def hex_prism(r, z0, z1):
    """hexagon with vertices on +-y (the shaft end's vertex-to-vertex is along y), circumradius r, from z1 up to z0 (z0 > z1)."""
    h = extrude(RegularPolygon(radius=r, side_count=6), amount=abs(z0 - z1))
    return Pos(0, 0, z1) * Rot(0, 0, 90) * h


def hub():
    h = X.pen_dog()
    z_d0, z_d1 = X.DOG["z"]
    zs0, zs1 = SPN["web"][0], SPN["z1"]
    h += Pos(0, 0, (zs0 + zs1) / 2 + 0.01) * Cylinder(SPN["r"], abs(zs0 - zs1) + 0.02)
    # hex socket for the GT1 shaft's hex end (circumradius 4.04 + 0.2), open to the top of the dog
    h -= hex_prism(4.25, z_d0 + 0.5, -30.3)
    # spindle bore from below up to the web (M3 head 5.7 in 6.0), M3 clearance through the web
    h -= Pos(0, 0, (SPN["web"][1] + zs1) / 2 - 0.01) * Cylinder(SPN["bore_r"], abs(SPN["web"][1] - zs1) + 0.02)
    h -= Pos(0, 0, (SPN["web"][0] + SPN["web"][1]) / 2) * Cylinder(1.7, abs(SPN["web"][0] - SPN["web"][1]) + 1.0)
    # cross hole for the A dowel (3.05)
    h -= cyl_y(1.525, -8.0, 8.0, 0.0, COLLAR["pin_z"])
    return h


def bevel_a():
    b = Pos(0, 0, ZA) * Rot(0, 0, 270) * G.bevel(NT, H_OUT, H_IN)
    b += Pos(0, 0, (COLLAR["z"][0] + COLLAR["z"][1]) / 2 + 0.01) * Cylinder(COLLAR["r"], abs(COLLAR["z"][0] - COLLAR["z"][1]) + 0.02)
    b -= Pos(0, 0, (COLLAR["z"][0] + SPN["z1"]) / 2 + 0.5) * Cylinder(COLLAR["bore_r"], abs(COLLAR["z"][0] - SPN["z1"]) + 1.0)
    b -= cyl_y(1.525, -8.0, 8.0, 0.0, COLLAR["pin_z"])
    return b


def pinion_unit():
    """Cross shaft along -y through the apex: stub, bevel B, back collar, neck, pinion, journal.  Local +z = axis away from the apex."""
    u = G.bevel(NT, H_OUT, H_IN)
    u += Pos(0, 0, 8.75) * Cylinder(6.6, 1.52)                    # collar h 8.0..9.5 (r 6.6 ~ the pinion root 6.75: tooth tips overhang only 2.4 mm when printed pinion-down)
    u += Pos(0, 0, 9.5) * G.spur(NT, abs(RACK_Y[1] - RACK_Y[0]))          # pinion h 9.5..15.0
    u += Pos(0, 0, 16.6) * Cylinder(2.5, 3.2)                     # journal h 15.0..18.2
    u += Pos(0, 0, 2.85) * Cylinder(1.5, 4.7)                     # stub h 0.5..5.2
    return Pos(0, 0, ZA) * Rot(90, 0, 0) * Rot(0, 0, 11.25) * u


def carriage():
    centres = [pi * k for k in TEETH_K]
    r = G.rack(centres, abs(RACK_Y[1] - RACK_Y[0]), BAR_X_BACK - (RACK_X + 1.25))      # local x depth, y along, z width
    r = Pos(RACK_X, RACK_Y[1], ZA) * Rot(90, 0, 0) * r                                 # local z -> global -y (width 9.5..15), local y -> z
    z_lo_teeth = ZA + pi * min(TEETH_K) - pi / 2
    z_hi_teeth = ZA + pi * max(TEETH_K) + pi / 2
    r += bx((RACK_X + 1.25, BAR_X_BACK), RACK_Y, (z_hi_teeth - 0.01, BAR_TOP))
    r += bx((RACK_X - 1.0, BAR_X_BACK), RACK_Y, (z_lo_teeth + 0.01, BRIDGE["z"][1]))   # plain bar under the teeth, tips-level in x
    r += bx(BRIDGE["x"], BRIDGE["y"], BRIDGE["z"])
    c = 0.1
    r -= bx((-ROD["h"] / 2 - c, ROD["h"] / 2 + c), (-ROD["w"] / 2 - c, ROD["w"] / 2 + c), (BRIDGE["z"][0] - 0.1, ROD["top"]))     # rod socket from below
    r -= Pos(0, 0, (BRIDGE["z"][1] + ROD["top"]) / 2) * Cylinder(1.7, abs(BRIDGE["z"][1] - ROD["top"]) + 0.5)                       # M3 clearance
    return r


def rod():
    r = bx((-ROD["h"] / 2, ROD["h"] / 2), (-ROD["w"] / 2, ROD["w"] / 2), (ROD["top"], ROD["z1"]))
    r += rbox((-ROD["h"] / 2, -ROD["h"] / 2 + PAD["h"]), (-PAD["w"] / 2, PAD["w"] / 2), (ROD["z1"] + 0.01, ROD["z1"] - PAD["t"]), r_v=1.5)
    r -= Pos(0, 0, ROD["top"] - 3.0 + 0.01) * Cylinder(1.3, 6.0)                         # self-tap pilot for the M3x8 from the bridge top
    try:
        r = fillet(r.edges().filter_by(Axis.Z).filter_by(lambda e: e.center().Z < ROD["z1"] + 0.5), 0.6)
    except Exception:
        pass
    return r


def nose():
    """Cap plate + tub open at the front (-y): spindle bores, the web with the rack slot, the cross-shaft boss, the floor and the rod guide."""
    foot = bx(TUB["x"], TUB["y"], CAP)
    cap = (outline(CAP[0] + 0.01, CAP[1]) + foot) & (Pos(0, 0, sum(CAP) / 2) * chamfer(Box(36, 36, abs(CAP[0] - CAP[1])).edges().filter_by(Axis.Z), 1.5))
    n = cap - dock_keel()          # the keel web (x -39..-15.5, y +-1.2) passes the cap's edge: notch it (STAMP1's cap has the same 1.5 mm overlap)
    n -= bx((-40.0, -15.2), (-5.3, 5.3), (CAP[0] + 1, CAP[1] - 1))   # the cap slides UP past the keel's bottom flange (y +-5): notch it full width (path check 10/30)
    n += bx(TUB["x"], TUB["y"], (CAP[1] + 0.01, TUB["z"][1]))
    n += cyl_y(BOSS["r"], BOSS["y1"], CAVITY["y"][1] + 1.0, 0.0, ZA)                       # cross-shaft stub boss off the back wall
    n += rbox(GUIDE["x"], GUIDE["y"], (GUIDE["z"][0], GUIDE["z"][1] + 0.01), r_v=2.0)
    # cuts
    n -= bx(CAVITY["x"], CAVITY["y"], CAVITY["z"])
    # re-add the boss (the cavity cut removed it): it hangs from the back wall through the cavity
    n += cyl_y(BOSS["r"], BOSS["y1"], CAVITY["y"][1] + 1.0, 0.0, ZA)
    for zs in ((-50.1, -45.1), (-93.0, -88.0)):
        n += bx((CAVITY["x"][1] - 0.1, LID["x"][1]), (LID["y"][1], -8.5), zs)      # lid screw ears on the +x side
        n += bx((TUB["x"][0], -5.0), (LID["y"][1], -8.5), zs)                      # lid screw blocks on the -x wall
    n -= cyl_y(BOSS["bore_r"], BOSS["bore_y"][0], BOSS["bore_y"][1], 0.0, ZA)
    n -= Pos(0, 0, (CAP[0] + CAVITY["z"][0]) / 2) * Cylinder(5.3, abs(CAP[0] - CAVITY["z"][0]) + 0.2)            # spindle bearing bore through cap + web
    n -= bx(SLOT["x"], SLOT["y"], (SLOT["z_top"], CAVITY["z"][0] - 0.1))                                          # rack slot in the web
    n -= bx((-ROD["h"] / 2 - ROD["clr"], ROD["h"] / 2 + ROD["clr"]), (-ROD["w"] / 2 - ROD["clr"], ROD["w"] / 2 + ROD["clr"]), (GUIDE["z"][0] - 0.1, CAVITY["z"][1] + 0.1))
    for x, y in CAP_INS:
        n -= Pos(x, y, (CAP[0] + CAP[1]) / 2) * Cylinder(1.7, abs(CAP[0] - CAP[1]) + 0.2)
        n -= Pos(x, y, CAP[1] + 0.6 - 0.01) * Cylinder(3.4, 1.22)
    for x, z in LID_SCREWS:
        n -= cyl_y(1.3, LID["y"][1] - 0.1, LID["y"][1] + 7.0, x, z)
    return n


def lid():
    l = rbox(LID["x"], LID["y"], LID["z"], r_v=0.0)
    l = fillet(l.edges().filter_by(Axis.Y), 2.0)                                         # round the 4 plate corners first, then add features
    l += cyl_y(3.5, LID["y"][1] - 0.01, -15.1, 0.0, ZA)                                  # thrust ring for the pinion face (0.1 mm gap)
    l -= cyl_y(2.7, LID["y"][0] - 0.1, -15.0, 0.0, ZA)                                   # journal bore (shaft 5.0)
    for x, z in LID_SCREWS:
        l -= cyl_y(1.7, LID["y"][0] - 0.1, LID["y"][1] + 0.1, x, z)
        l -= cyl_y(3.4, LID["y"][0] - 0.1, LID["y"][0] + 1.2, x, z)
    return l


def main():
    hd = Head("JG2", "jg2")
    hd.coupling_parts()
    hd.printed("housing", housing(), color="pla_white", label="light housing + dock keel (GT1's neck)")
    HUB, NOSE, LID_, CARR, ROD_, A_, B_ = hub(), nose(), lid(), carriage(), rod(), bevel_a(), pinion_unit()
    hd.printed("hub", HUB, color="pla_black", label="hub: stop dog + spindle")
    hd.printed("nose", NOSE, color="pla_white", label="cap + gearbox tub + rod guide")
    hd.printed("bevel_a", A_, color="pla_black", label="miter bevel A, 16 t m1 (on the spindle)")
    hd.printed("pinion_unit", B_, color="pla_black", label="cross shaft: bevel B 16 t + spur pinion 16 t (PD 16)")
    hd.printed("carriage", CARR, color="pla_white", label="carriage: rack m1 + bridge")
    hd.printed("rod", ROD_, color="pla_white", label="rod 7 x 5 + 14 x 8 pad (JG1's)")
    hd.printed("lid", LID_, color="pla_white", label="front lid + pinion journal")
    hd.bought("a_dowel", Pos(0, 0, COLLAR["pin_z"]) * cyl_y(1.5, -6.0, 6.0, 0.0, 0.0), 0.67, "steel", "3x12 steel dowel (A to spindle cross pin)")
    ax_dn = (0, 0, -1)
    hd.add_hw("shaft_screw", "M3x8 button", (0.0, 0.0, SPN["web"][1] - 0.5), ax_dn, 8.0, "bolt", "hub")
    hd.add_hw("shaft_washer", "M3 washer", (0.0, 0.0, SPN["web"][1]), ax_dn, 0.5, "washer", "hub")
    for i, (x, y) in enumerate(INS):
        hd.add_hw(f"housing_insert{i + 1}", "M3 heat-set insert 4.0 bore x 6.7", (x, y, HOUS_Z[1]), ax_dn, 5.7, "insert", "housing")
    for i, (x, y) in enumerate(CAP_INS):
        hd.add_hw(f"cap_screw{i + 1}", "M3x6 button", (x, y, CAP[1] + 1.2), ax_dn, 6.0, "bolt", "nose")
    for i, (x, z) in enumerate(LID_SCREWS):
        hd.add_hw(f"lid_screw{i + 1}", "M3x8 button (self-taps)", (x, LID["y"][0] + 1.2, z), (0, -1, 0), 8.0, "bolt", "lid")
    hd.add_hw("rod_screw", "M3x8 button (self-taps into the rod)", (0.0, 0.0, BRIDGE["z"][1]), (0, 0, 1), 8.0, "bolt", "carriage")
    ext = {"stroke_mm": round(STROKE, 2), "input_deg": round(THETA * 180 / pi, 1), "pitch_radius_mm": RP, "teeth": NT, "module": G.M,
           "force_ideal_N_at_0p25Nm": round(0.25 / (RP / 1000), 1), "poses": "STL = extended; *_ret = retracted",
           "pad_plane_z_extended": ROD["z1"] - PAD["t"], "pad_plane_z_retracted": ROD["z1"] - PAD["t"] + STROKE}
    out = hd.finish(extra=ext)
    print({k: out[k] for k in ("mass_g", "com_dist_from_coupling_face_mm", "coupling_moment_Nmm", "envelope_mm")})
    for n, p in out["parts"].items():
        print(n, p["g"], p["watertight"], p["bbox"])

    # ---- retracted pose + interference checks (volumes via OCC booleans on the small gear solids only)
    import math
    w = -THETA
    T_z = trimesh.transformations.rotation_matrix(w, [0, 0, 1])
    T_y = trimesh.transformations.rotation_matrix(w, [0, 1, 0], point=[0, 0, ZA])
    T_c = trimesh.transformations.translation_matrix([0, 0, STROKE])
    st = hd.dir / "stl"
    for n, T in (("hub", T_z), ("bevel_a", T_z), ("a_dowel", T_z), ("shaft", T_z), ("rotor", T_z), ("pinion_unit", T_y), ("carriage", T_c), ("rod", T_c)):
        m = trimesh.load(str(st / f"{n}.stl")); m.apply_transform(T); m.export(str(st / f"{n}_ret.stl"))
    for hid in ("shaft_screw", "shaft_washer", "rod_screw"):
        m = trimesh.load(str(st / f"hw_{hid}.stl"))
        m.apply_transform(T_c if hid == "rod_screw" else T_z)
        m.export(str(st / f"hw_{hid}_ret.stl"))
    # shaft rotation not needed (GT1 parts shown as is)
    V = lambda s: 0.0 if s is None else float(s.volume)

    def I(a, b):   # OCC raises 'Null TopoDS_Shape' for an empty common: that is 0 mm3
        try:
            return V(a & b)
        except ValueError:
            return 0.0
    A = A_
    mesh = {"A&B extended mm3": round(I(A, B_), 3)}
    from build123d import Location
    Ar = Rot(0, 0, math.degrees(w)) * A
    Br = Location((0, 0, ZA)) * Rot(0, math.degrees(w), 0) * Location((0, 0, -ZA)) * B_
    mesh["A&B retracted mm3"] = round(I(Ar, Br), 3)
    C = CARR
    mesh["pinion&rack extended mm3"] = round(I(B_, C), 3)
    mesh["pinion&rack retracted mm3"] = round(I(Br, (Pos(0, 0, STROKE) * C)), 3)
    mesh["carriage&nose mm3"] = round(I(C, NOSE), 3)
    mesh["carriage retracted&nose mm3"] = round(I((Pos(0, 0, STROKE) * C), NOSE), 3)
    mesh["rod&nose ext mm3"] = round(I(ROD_, NOSE), 3)
    mesh["rod&nose ret mm3"] = round(I((Pos(0, 0, STROKE) * ROD_), NOSE), 3)
    mesh["bevelB unit&nose mm3"] = round(I(B_, NOSE), 3)
    mesh["A&nose mm3"] = round(I(A, NOSE), 3)
    mesh["hub&nose mm3"] = round(I(HUB, NOSE), 3)
    mesh["Br&nose ret mm3"] = round(I(Br, NOSE), 3)
    mesh["lid&nose mm3"] = round(I(LID_, NOSE), 3)
    mesh["unit&lid mm3"] = round(I(B_, LID_), 3)
    mesh["rod&carriage mm3"] = round(I(ROD_, C), 3)
    print("INTERFERENCE", json.dumps(mesh, indent=1))
    pj = json.loads((hd.dir / "parts.json").read_text())
    pj["interference_mm3"] = mesh
    (hd.dir / "parts.json").write_text(json.dumps(pj, indent=1))


if __name__ == "__main__":
    main()


def debug():
    N, C = nose(), carriage()
    for name, shp in (("carr ret & nose", (Pos(0, 0, STROKE) * C) & N),):
        if shp is None:
            print(name, None); continue
        for s in shp.solids():
            bb = s.bounding_box(); print(name, round(s.volume, 3), bb.min, bb.max)
    L = lid(); print("lid valid", L.is_valid, len(L.solids()), L.volume)
