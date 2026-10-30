# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""HI5F: single-servo fin-ray hand for the SO-101 v4 tool changer (replaces the flat HI5 slab).  2026-10-30, heads-cool for pm-eng.
Run (Idle priority, 1.5 GB watchdog inside): python toolchanger/heads_x3/build_hi5f.py -> heads_x3/hi5f/
Nothing here is printed, bought or run on the arm.

WHAT IT DOES  Jaw servo -> rotor -> GT1 shaft -> HELIX DRIVE (helix_drive.py: drum with 2 helical grooves, sleeve climbs 16.8 mm).  The sleeve's two
legs carry the FLEXOR PLATE down the palm.  Each finger is a TPU fin ray whose BACK flank is screwed to the frame's knuckle bar and whose PALM flank is
screwed to the flexor plate.  Pulling the palm flank 16.8 mm toward the wrist curls the finger toward the palm (Fin Ray actuated by shifting one base).
OPEN stop (where the swap routine parks) = fingers straight, flat open hand ("high five").  CLOSED stop = fingers curled; the thumb is a passive fin ray
angled out 20 deg, so the closed pose reads as a curled fist with the thumb out.  Curl angle ~ s / w_root = 16.8 / 15.7 = 1.07 rad ~ 60 deg (ESTIMATE, not FEA).
FRAME: TOOL frame (coupling face z = 0, tool along -z, +x = UP in the dock = palm face).  STL pose = OPEN; *_closed.stl = closed.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import memwd
memwd.start(1.5e9)
import json
from math import radians, cos, sin

import numpy as np
import trimesh
from build123d import Polygon, extrude, Cylinder, Pos, Rot, Box, Axis, fillet

from common3 import *   # noqa
from helix_drive import Drive, THETA_DEG, CAP, RAIL_X, RAIL_Y, PSI, PIN_R_IN, PIN_L

D = Drive(16.8)
S = D.s
YC = -8.0                                  # palm centre y (thumb side +y)
PH = 31.5                                  # flexor / dorsal plate half width
ZK = -88.5                                 # knuckle line (finger roots, OPEN pose) = flexor top -78.5 - 10 (tab zone): hand as short as the drive allows (CoM)
DORS = dict(x=(-4.0, -1.1), top=-62.0)                                  # HAND plate (separate part, 0.3 to the flexor); top 1 under the lowest sleeve
FOOT = dict(x=(-12.0, -4.2), y=(12.6, 16.5), z=(-73.0, -50.0), scr_x=-9.0, scr_z=-54.0)   # frame feet behind the hand plate; the hand joint blocks clamp to their outer faces (critic R2 #1)
JOINT = dict(x=(-12.0, -2.0), y=(16.6, 21.0), z=(-73.0, -50.0))        # beside the feet (0.1), 0.5 behind the pins, above the finger back tabs; M3x8 along y from outside, above the thumb boss
PILOT_H = 2.7 / 2                                                          # horizontal self-tap pilot: 2.6 + hole.p1s.horizontal (UNTESTED)
KBAR = dict(x=(-7.0, -1.1), z=(ZK, ZK + 10.0))
FLEX = dict(x=(-0.8, 3.2), top=-78.5, leg_y=(8.7, 11.6))       # closed top -61.7: 1.6 under the rails (rail bottom -60.1)
LIP = dict(y=(PH + 0.3, PH + 2.8), hook_y0=PH - 1.2, x=(-4.0, 4.9), hook_x=(3.5, 4.9), z=(ZK, -62.0))   # 0.3 sliding gaps (critic R1 #13)
FW, PITCH = 13.0, 15.5
FINGERS = [("index", YC + 1.5 * PITCH, 42.0), ("middle", YC + 0.5 * PITCH, 45.0), ("ring", YC - 0.5 * PITCH, 43.0), ("pinky", YC - 1.5 * PITCH, 35.0)]   # tip z -145 (JG2 -145.8)
THUMB = dict(deg=20.0, L=26.0, y=36.4, z=-72.5)
TAB = dict(dx=(-10.0, -7.0), px=(3.2, 5.7), h=10.0, scr_v=5.0)      # palm tab 2.5: M3x6 gets 3.5 mm of thread in the 4 mm flexor
T_BACK, T_FRONT, T_RIB, RIB_PITCH = 1.6, 1.2, 1.2, 6.0     # tpu.finray.flank_thickness (front 1.2 / back 1.6) - UNTESTED, coupon T1 (tpu-iter)
CURL_DEG = 60.0


def finray(L, W=FW):
    """TPU fin-ray finger, root at the origin, along -z, width along y (centred), back flank -x, palm flank +x, two root tabs."""
    xb0, xp0 = TAB["dx"][0], TAB["px"][1]                 # outer lines at the root
    xbt, xpt = -3.5, 0.5                                   # outer lines at the tip
    tr = (xpt - xbt) / 2
    outer = Polygon((xb0, 0), (xp0, 0), (xpt, -L + tr), (xbt, -L + tr), align=None)
    o = extrude(outer, amount=W, dir=(0, 0, 1))
    o += Pos((xbt + xpt) / 2, -L + tr, W / 2) * Cylinder(tr, W)
    tip_in = -L + tr + 1.2
    inner = Polygon((xb0 + T_BACK, 1.0), (xp0 - T_FRONT, 1.0), (xpt - T_FRONT, tip_in), (xbt + T_BACK, tip_in), align=None)
    i = extrude(inner, amount=W + 2, dir=(0, 0, 1))
    i = Pos(0, 0, -1) * i
    f = o - i
    ribs = None
    v = -RIB_PITCH
    while v > tip_in + 2.5:
        r = bx((xb0, xp0), (v - T_RIB / 2, v + T_RIB / 2), (0, W))
        ribs = r if ribs is None else ribs + r
        v -= RIB_PITCH
    f += ribs & i
    f += bx(TAB["dx"], (0, TAB["h"]), (0, W)) + bx(TAB["px"], (0, TAB["h"]), (0, W))
    for xs in (TAB["dx"], TAB["px"]):
        f -= Pos(sum(xs) / 2, TAB["scr_v"], W / 2) * Cylinder(1.7, 5, rotation=(0, 90, 0))
    # local (x, v, w) -> tool (x, -w, v), centred in y
    return Pos(0, W / 2, 0) * Rot(90, 0, 0) * f


def thumb_place(shape):
    return Pos(0, THUMB["y"], THUMB["z"]) * Rot(THUMB["deg"], 0, 0) * shape


def thumb_pt(x, y, z):
    a = radians(THUMB["deg"])
    return (x, THUMB["y"] + y * cos(a) - z * sin(a), THUMB["z"] + y * sin(a) + z * cos(a))


def frame():
    """cap + slotted rails + two feet behind the hand plate.  Goes on BEFORE the carriage (the cap bore cannot pass the sleeve)."""
    f = D.cap() + D.rails()
    for s in (1, -1):
        f += bx(FOOT["x"], tuple(sorted((s * FOOT["y"][0], s * FOOT["y"][1]))), FOOT["z"])
        f -= Pos(FOOT["scr_x"], s * sum(FOOT["y"]) / 2, FOOT["scr_z"]) * Cylinder(PILOT_H, abs(FOOT["y"][1] - FOOT["y"][0]) + 0.2, rotation=(90, 0, 0))   # pilot along y (horizontal in print)
    return f


def hand():
    """back-of-hand plate + knuckle bar + side lips + thumb boss + 2 joint blocks.  Slides UP after the carriage, 2x M3x12 into the frame feet."""
    y0, y1 = YC - PH, YC + PH
    f = bx(DORS["x"], (y0, y1), (DORS["top"], KBAR["z"][1] - 0.01))
    for yc in (YC - 1.0 * PITCH, YC, YC + 1.0 * PITCH):                        # metacarpal windows (show the flexor sliding behind)
        f -= rbox((DORS["x"][0] - 1, DORS["x"][1] + 1), (yc - 4.5, yc + 4.5), (-76.0, -65.0), r_v=4.0, axis="x")
    f += bx(KBAR["x"], (y0, y1), KBAR["z"])
    for s in (1, -1):                                                         # side lips: hold the flexor plate against the hand plate
        ys = (YC + s * LIP["y"][0], YC + s * LIP["y"][1])
        f += bx(LIP["x"], tuple(sorted(ys)), LIP["z"])
        f += bx(LIP["hook_x"], tuple(sorted((YC + s * LIP["hook_y0"], YC + s * LIP["y"][1]))), LIP["z"])
        f += bx(DORS["x"], tuple(sorted((YC + s * (PH - 1.0), YC + s * LIP["y"][0] + s * 0.01))), LIP["z"])
        f += bx(JOINT["x"], tuple(sorted((s * JOINT["y"][0], s * JOINT["y"][1]))), JOINT["z"])
        f -= Pos(FOOT["scr_x"], s * sum(JOINT["y"]) / 2, FOOT["scr_z"]) * Cylinder(1.8, abs(JOINT["y"][1] - JOINT["y"][0]) + 0.2, rotation=(90, 0, 0))   # 3.6 clearance (horizontal)
    boss = thumb_place(bx((TAB["dx"][1], TAB["px"][0]), (-FW / 2, FW / 2), (0.0, TAB["h"])))
    f += boss
    f += bx((LIP["x"][0], TAB["px"][0]), (YC + LIP["y"][1] - 0.5, THUMB["y"] - 3.0), (THUMB["z"] - 3.0, THUMB["z"] + 11.0))
    for _, yc, _L in FINGERS:
        f -= Pos(-4.25, yc, ZK + TAB["scr_v"]) * Cylinder(PILOT_H, 5.5, rotation=(0, 90, 0))
    for xs in ((TAB["dx"][1] - 0.01, -2.0), (-1.0, TAB["px"][0] + 0.01)):
        c = Pos(sum(xs) / 2, 0, TAB["scr_v"]) * Cylinder(PILOT_H, abs(xs[1] - xs[0]), rotation=(0, 90, 0))
        f -= thumb_place(c)
    return f


def carriage():
    c = D.sleeve()
    for s in (1, -1):
        c += bx(FLEX["x"], tuple(sorted((s * FLEX["leg_y"][0], s * FLEX["leg_y"][1]))), (D.bot + 0.5, FLEX["top"] - 0.01))
    p = bx(FLEX["x"], (YC - PH, YC + PH), (FLEX["top"], ZK))
    p = fillet(p.edges().filter_by(Axis.X).filter_by(lambda e: e.center().Z < ZK + 1), 3.0)
    for _, yc, _L in FINGERS:
        p -= Pos(sum(FLEX["x"]) / 2, yc, ZK + TAB["scr_v"]) * Cylinder(PILOT_H, 5, rotation=(0, 90, 0))
    return c + p


def bend(mesh, curl_deg, L, x0=TAB["dx"][0]):
    """Visual estimate of the closed pose: constant curvature about the back flank (x0), palm tab lifted by S."""
    m = mesh.copy()
    v = m.vertices.copy()
    u = ZK - v[:, 2]
    R = L / radians(curl_deg)
    out = v.copy()
    a = np.clip(u, 0, None) / R
    rho = R - (v[:, 0] - x0)
    bent = u > 0
    out[bent, 0] = x0 + R - rho[bent] * np.cos(a[bent])
    out[bent, 2] = ZK - rho[bent] * np.sin(a[bent])
    tab = (~bent) & (v[:, 0] > 0)
    out[tab, 2] += S
    m.vertices = out
    return m


def main():
    hd = Head("HI5F", "hi5f")
    hd.coupling_parts()
    OPEN = -THETA_DEG
    for n in ("rotor", "shaft"):
        hd.parts[n]["shape"] = Rot(0, 0, OPEN) * hd.parts[n]["shape"]
    FR0, HA, CA, HUB = frame(), hand(), carriage(), D.hub(OPEN)
    FR = FR0 + HA                                                  # static parts together for the sweeps
    hd.printed("housing", housing(), color="pla_white", label="light housing + dock keel (GT1 neck)")
    hd.printed("hub", HUB, color="pla_black", label="hub: stop dog + helix drum (2 grooves)")
    hd.printed("frame", FR0, color="pla_white", label="frame: cap + pin rails + 2 feet")
    hd.printed("hand", HA, color="pla_white", label="hand: back plate + knuckle bar + side lips + thumb boss (bolts under the feet)")
    hd.printed("carriage", CA, color="pla_black", label="carriage: sleeve + legs + flexor plate (moves 16.8 mm)")
    fingers = {}
    for name, yc, L in FINGERS:
        fingers[name] = (Pos(0, yc, ZK) * finray(L), L)
        hd.printed(f"f_{name}", fingers[name][0], mat="tpu", color="tpu", label=f"TPU fin-ray {name} finger, {L:.0f} long")
    TH = thumb_place(finray(THUMB["L"]))
    hd.printed("f_thumb", TH, mat="tpu", color="tpu", label=f"TPU fin-ray thumb, {THUMB['L']:.0f} long (passive, 20 deg out)")
    pins = D.pins()
    hd.bought("drive_pins", pins, 2 * 0.67, "steel", "2x 3x12 steel dowel: groove followers + anti-rotation keys (pressed 3.05 in the sleeve)")
    ax_dn = (0, 0, -1)
    from common3 import X as _X
    hd.add_hw("shaft_screw", "M3x8 button", (0.0, 0.0, -33.5), (0, 0, -1), 8.0, "bolt", "hub")
    hd.add_hw("shaft_washer", "M3 washer", (0.0, 0.0, -33.0), ax_dn, 0.5, "washer", "hub")
    for i, (x, y) in enumerate(INS):
        hd.add_hw(f"housing_insert{i + 1}", "M3 heat-set insert 4.0 bore x 6.7", (x, y, HOUS_Z[1]), ax_dn, 5.7, "insert", "housing")
        hd.add_hw(f"cap_screw{i + 1}", "M3x6 button", (x, y, CAP[1] + 1.2), ax_dn, 6.0, "bolt", "frame")
    for name, yc, L in FINGERS:
        hd.add_hw(f"back_screw_{name}", "M3x8 button (self-taps into the knuckle bar)", (TAB["dx"][0], yc, ZK + TAB["scr_v"]), (-1, 0, 0), 8.0, "bolt", "hand")
        hd.add_hw(f"palm_screw_{name}", "M3x6 button (self-taps into the flexor plate)", (TAB["px"][1], yc, ZK + TAB["scr_v"]), (1, 0, 0), 6.0, "bolt", "carriage")
    hd.add_hw("back_screw_thumb", "M3x8 button (self-taps into the thumb boss)", thumb_pt(TAB["dx"][0], 0, TAB["scr_v"]), (-1, 0, 0), 8.0, "bolt", "hand")
    hd.add_hw("palm_screw_thumb", "M3x6 button (self-taps into the thumb boss)", thumb_pt(TAB["px"][1], 0, TAB["scr_v"]), (1, 0, 0), 6.0, "bolt", "hand")
    for i, s in enumerate((1, -1)):
        hd.add_hw(f"joint_screw{i + 1}", "M3x8 button (self-taps into the frame foot)", (FOOT["scr_x"], s * JOINT["y"][1], FOOT["scr_z"]), (0, s, 0), 8.0, "bolt", "hand")
    ext = {"drive": D.forces(), "curl_deg_estimate": CURL_DEG, "poses": "STL = OPEN (fingers straight); *_closed.stl = CLOSED (hub +120.3 deg, carriage +16.8, fingers bent: visual estimate)",
           "finger_lengths_mm": {n: L for n, _, L in FINGERS} | {"thumb": THUMB["L"]}}
    import common3
    common3.RHO_TPU = 1.21e-3              # slicer: HI5F_t1 18.1 g / 14.37 cm3 (critic R2 #5); PLA stays 0.95 (weigh the first print)
    out = hd.finish(extra=ext)
    print({k: out[k] for k in ("mass_g", "com_dist_from_coupling_face_mm", "coupling_moment_Nmm", "envelope_mm")})
    for n, p in out["parts"].items():
        print(n, p["g"], p["watertight"], p["bbox"])

    # ---- closed pose
    st = hd.dir / "stl"
    Tz = trimesh.transformations.rotation_matrix(radians(THETA_DEG), [0, 0, 1])
    Tc = trimesh.transformations.translation_matrix([0, 0, S])
    for n, T in (("hub", Tz), ("rotor", Tz), ("shaft", Tz), ("carriage", Tc), ("drive_pins", Tc)):
        m = trimesh.load(str(st / f"{n}.stl")); m.apply_transform(T); m.export(str(st / f"{n}_closed.stl"))
    for hid in ["shaft_screw", "shaft_washer"] + [f"palm_screw_{n}" for n, _, _ in FINGERS]:
        m = trimesh.load(str(st / f"hw_{hid}.stl")); m.apply_transform(Tz if hid.startswith("shaft") else Tc); m.export(str(st / f"hw_{hid}_closed.stl"))
    for name, yc, L in FINGERS:
        m = trimesh.load(str(st / f"f_{name}.stl"))
        v, f = trimesh.remesh.subdivide_to_size(m.vertices, m.faces, max_edge=3.0)
        bend(trimesh.Trimesh(v, f), CURL_DEG, L).export(str(st / f"f_{name}_closed.stl"))

    # ---- interference (OCC booleans, small solids only)
    V = lambda s: 0.0 if s is None else round(float(s.volume), 3)
    HUBc = Rot(0, 0, THETA_DEG) * HUB
    CAc, PINc = Pos(0, 0, S) * CA, Pos(0, 0, S) * pins
    chk = {"frame&hand (bolted, touching)": V(FR0 & HA), "frame&carriage open": V(FR & CA), "frame&carriage closed": V(FR & CAc), "frame&carriage mid": V(FR & (Pos(0, 0, S / 2) * CA)),
           "hub&carriage open": V(HUB & CA), "hub&carriage closed": V(HUBc & CAc), "hub&frame open": V(HUB & FR), "hub&frame closed": V(HUBc & FR),
           "pins&frame open": V(pins & FR), "pins&frame closed": V(PINc & FR)}
    for ph in (-120.3, -90.0, -60.0, -30.0, 0.0):
        dz = D.zpin(ph) - D.zp
        chk[f"pins&hub phi {ph}"] = V((Pos(0, 0, dz) * pins) & D.hub(ph))
    for name, (fs, L) in fingers.items():
        chk[f"{name}&frame"] = V(fs & FR)
        chk[f"{name}&carriage"] = V(fs & CA)
    chk["thumb&frame"] = V(TH & FR)
    chk["thumb&carriage open"] = V(TH & CA)
    chk["thumb&carriage closed"] = V(TH & CAc)
    chk["housing&carriage closed"] = V(housing() & CAc)
    print("INTERFERENCE", json.dumps(chk, indent=1))
    pj = json.loads((hd.dir / "parts.json").read_text())
    pj["interference_mm3"] = chk
    (hd.dir / "parts.json").write_text(json.dumps(pj, indent=1))


if __name__ == "__main__":
    main()
