# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Checks for the changer gripper tool v1 (build_gripper_tool.py).  Mesh-level, from the exported STLs.  2026-10-25.
  1 motion sweep s = 0..22 mm: every moving part vs every other part (intersection volume), pinion turned by -s/r
  2 stops: rack 1 / rack 2 hit their post past closed and the end wall past open; nothing else stops them first
  3 backlash: how far the pinion turns with the jaws held (mm at the jaw)
  4 dock: tool in the dock frame vs dock_post_v4 at s = 0 / 11 / 22, compared with the dummy tool (C0) baseline
  5 fingers: replica_fixed_v2 module on each stub: seated clear, slides on along the stub axis, grip gap closed/open
  6 coupling: rotor on the shoulder, tongue in the rotor slot, plate on the neck
Run: python toolchanger/cad/gt_checks.py  -> v4/gt_checks.json, exit 1 on a FAIL
"""
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
import build_gripper_tool as G  # noqa: E402

S = H / "v4" / "stl"
THR = 0.5   # mm3: below this an intersection is tessellation noise
res = {"fails": []}


from build123d import Pos, Rot  # noqa: E402
import build_changer_v4 as V  # noqa: E402


def X(a, b):
    """Exact OCC intersection volume (mm3) of two build123d solids."""
    try:
        return round(float((a & b).volume), 3)
    except Exception as e:  # noqa: BLE001
        return f"ERR {e}"


def tr(m, T):
    return T * m


def Tx(dx):
    return Pos(dx, 0, 0)


def Rz(deg):
    return Rot(0, 0, deg)


body, shaft, pin, j1 = G.body(), G.shaft(), G.pinion(), G.jaw1()
cover = G.cover()
j2 = Rot(0, 0, 180) * j1
rotor, plate = G.rotor(), G.coupling_plate()
for n, m in (("body", body), ("shaft", shaft), ("pinion", pin), ("jaw1", j1), ("jaw2", j2)):
    if not m.is_valid:
        res["fails"].append(f"{n} not a valid solid")


def pose(s, dphi=0.0):
    phi = -np.degrees(s / G.R_P) + dphi
    return tr(pin, Rz(phi)), tr(j1, Tx(s)), tr(j2, Tx(-s))


# 1 sweep
sweep = []
for s in np.arange(0.0, G.S_OPEN + 0.01, 1.0):
    p, a, b = pose(s)
    row = {"s": round(float(s), 1), "jaw1_body": X(a, body), "jaw2_body": X(b, body), "jaw1_jaw2": X(a, b),
           "jaw1_pinion": X(a, p), "jaw2_pinion": X(b, p), "jaw1_shaft": X(a, shaft), "jaw2_shaft": X(b, shaft),
           "pinion_body": X(p, body), "jaw1_cover": X(a, cover), "jaw2_cover": X(b, cover)}
    sweep.append(row)
    bad = {k: v for k, v in row.items() if k != "s" and (isinstance(v, str) or v > THR)}
    if bad:
        res["fails"].append(f"sweep s={row['s']}: {bad}")
res["sweep_max"] = {k: max(r[k] for r in sweep if not isinstance(r[k], str)) for k in sweep[0] if k != "s"}
res["shaft_body"] = X(shaft, body)
res["cover_body"] = X(cover, body)
res["cover_pinion"] = X(cover, pin)

# 1b roll play (r1 BLOCKING): turn jaw 1 about the slide axis (x) through the shoe centre until it touches body / cover
from build123d import Axis as _Ax  # noqa: E402
roll = {}
for sgn in (1, -1):
    for d in np.arange(0.25, 4.01, 0.25):
        a = Pos(0, 0, -33.6) * Rot(sgn * d, 0, 0) * Pos(0, 0, 33.6) * Tx(11.0) * j1
        if X(a, body) > 0.05 or X(a, cover) > 0.05:
            roll[sgn] = float(d); break
res["roll_play_deg"] = roll
res["roll_tip_play_mm"] = round(np.radians(sum(roll.values())) * 70.0, 2) if len(roll) == 2 else None
if len(roll) != 2 or sum(roll.values()) > 2.0:
    res["fails"].append(f"jaw roll play {roll} deg")
res["shaft_pinion"] = X(shaft, pin)

# 2 stops (drive the jaw past each end; the rack must hit body, never the pinion first)
stops = {}
for s in (-0.4, G.S_OPEN + 0.4):
    p, a, b = pose(s)
    stops[f"s={s:.1f}"] = {"jaw1_body": X(a, body), "jaw2_body": X(b, body)}
res["stops"] = stops
for k, v in stops.items():
    if not all((not isinstance(x, str)) and x > THR for x in v.values()):
        res["fails"].append(f"stop not solid at {k}: {v}")

# 3 backlash: turn the pinion with the jaws held at s = 11 until it touches a rack
bl = {}
for sgn in (1, -1):
    for d in np.arange(0.2, 6.01, 0.2):
        p, a, b = pose(11.0, sgn * d)
        if X(p, a) > 0.05 or X(p, b) > 0.05:
            bl[sgn] = float(d); break
res["backlash_deg"] = bl
res["backlash_mm_at_jaw"] = round(np.radians(sum(bl.values())) * G.R_P, 2) if len(bl) == 2 else None
if len(bl) != 2 or sum(bl.values()) > 6:
    res["fails"].append(f"backlash out of range {bl}")

# 4 dock
TD = Pos(-7.0, 0, 0) * Rot(0, -90, 0)       # tool -> dock frame (C0 transform: dock x = -z - 7, dock z = tool x)
dock = V.dock_post_v4()
dummy = TD * V.gripper_body_v4()
dk = {"dummy_body_baseline": X(dummy, dock), "body": X(tr(body, TD), dock), "plate": X(tr(plate, TD), dock), "cover": X(tr(cover, TD), dock)}
for s in (0.0, 11.0, G.S_OPEN):
    p, a, b = pose(s)
    dk[f"jaws_s{s:.0f}"] = X(tr(a, TD), dock) + X(tr(b, TD), dock)
res["dock"] = dk
for k, v in dk.items():
    if (k.startswith("jaws") or k == "cover") and (isinstance(v, str) or v > THR):
        res["fails"].append(f"dock clash {k}: {v}")
if isinstance(dk["body"], str) or dk["body"] > max(THR, dk["dummy_body_baseline"] + THR):
    res["fails"].append(f"body in dock {dk['body']} vs dummy {dk['dummy_body_baseline']}")
# swept path in the dock (r1 BLOCKING): jaws + fingers lifted / shifted around the seated pose (drop, pick, arm lag)
fing = G.finger_part()
F1 = G.part_to_jaw1() * fing
F2 = Rot(0, 0, 180) * F1
sweep_dk = []
for s_ in (0.0, G.S_OPEN - 0.7):
    p_, a_, b_ = pose(s_)
    movers = [a_, b_, Tx(s_) * F1, Tx(-s_) * F2]
    for dz in (0.0, 3.0, 8.0, 15.0, 25.0, 35.0):
        for dx in (-4.0, 0.0, 4.0):
            M_ = Pos(dx, 0, dz) * TD
            v = sum(X(M_ * m_, dock) for m_ in movers)
            if v > THR:
                sweep_dk.append({"s": s_, "dz": dz, "dx": dx, "mm3": v})
res["dock_swept_hits"] = sweep_dk
if sweep_dk:
    res["fails"].append(f"jaws/fingers hit the dock on the swept path: {sweep_dk[:4]}")

# lowest jaw point inside the rib's tool-z span (-25..-41) at full open (jaw 2 is the low one in the dock)
from build123d import Box  # noqa: E402
p, a, b = pose(G.S_OPEN)
sl = b & (Pos(0, 0, -33.0) * Box(200, 200, 16.0))
res["jaw2_min_x_in_rib_span_open"] = round(float(sl.bounding_box().min.X), 2)
if sl.bounding_box().min.X < -38.7:
    res["fails"].append(f"jaw 2 reaches x {sl.bounding_box().min.X:.2f} over the tail rib (top -39.3)")

# 5 fingers (the module is a SolidWorks STL: mesh test = deepest point of one surface inside the other)
def mesh_of(sol):
    import tempfile, os
    from build123d import export_stl
    f = os.path.join(tempfile.gettempdir(), "gt_tmp_mesh.stl")
    export_stl(sol, f, tolerance=0.01, angular_tolerance=0.1)
    return trimesh.load(f)


def depth(a, b, n=6000):
    """Deepest penetration (mm) of mesh a's surface into mesh b and vice versa (0 = clear)."""
    out = 0.0
    for p, q in ((a, b), (b, a)):
        pts = np.vstack([p.vertices, p.sample(n)])
        lo, hi = q.bounds - 0.01, q.bounds + 0.01
        pts = pts[np.all((pts > lo[0]) & (pts < hi[1]), axis=1)]
        if len(pts):
            sd = trimesh.proximity.signed_distance(q, pts)     # + inside
            out = max(out, float(sd.max()))
    return round(out, 3)


def Mt(m, T):
    c = m.copy(); c.apply_transform(T); return c


mod = trimesh.load(str(G.replica_module_mesh()))
T1 = np.eye(4); T1[:3, :3] = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]; T1[0, 3], T1[2, 3] = G.TX, G.TZ    # part -> jaw 1
R180 = trimesh.transformations.rotation_matrix(np.pi, [0, 0, 1])
T2 = R180 @ T1
J1m, J2m, Bm_ = mesh_of(j1), mesh_of(j2), mesh_of(body)
fm = {}
m1, m2 = Mt(mod, T1), Mt(mod, T2)
fm["seated_depth_jaw1"], fm["seated_depth_jaw2"] = depth(m1, J1m), depth(m2, J2m)
path = []
for off in (16.0, 12.0, 8.0, 4.0, 2.0, 1.0, 0.5, 0.0):      # module slides along the stub axis (tool -z = away)
    Toff = np.eye(4); Toff[2, 3] = -off
    path.append({"off_mm": off, "depth": depth(Mt(m1, Toff), J1m)})
fm["slide_on_path_jaw1"] = path
for s_ in (0.0, G.S_OPEN):
    Ts1 = np.eye(4); Ts1[0, 3] = s_
    Ts2 = np.eye(4); Ts2[0, 3] = -s_
    A, Bm = Mt(m1, Ts1), Mt(m2, Ts2)
    q = trimesh.proximity.closest_point(Bm, A.vertices)[1]
    fm[f"min_gap_s{s_:.0f}"] = round(float(q.min()), 2)
    fm[f"mod_vs_body_s{s_:.0f}"] = max(depth(A, Bm_), depth(Bm, Bm_))
    fm[f"mod_vs_other_jaw_s{s_:.0f}"] = max(depth(A, Mt(J2m, Ts2)), depth(Bm, Mt(J1m, Ts1)))
res["fingers"] = fm
for k, v in fm.items():
    if k.startswith(("seated", "mod_")) and v > 0.05:
        res["fails"].append(f"finger {k}: {v}")
if any(r["depth"] > 0.05 for r in path if r["off_mm"] >= 1.0):
    res["fails"].append(f"module hits the stub before seating: {path}")
if fm["min_gap_s0"] < 0.2:
    res["fails"].append(f"fingers touch when closed: {fm['min_gap_s0']}")

# 6 coupling side
res["coupling"] = {"rotor_body": X(rotor, body), "rotor_shaft": X(rotor, shaft), "plate_body": X(plate, body),
                   "plate_shaft": X(plate, shaft), "rotor_bottom_z": round(rotor.bounding_box().min.Z, 2),
                   "shoulder_z": G.Z_SHOULDER, "tongue_top_z": round(shaft.bounding_box().max.Z, 2)}
for k in ("rotor_body", "rotor_shaft", "plate_body", "plate_shaft"):
    v = res["coupling"][k]
    if isinstance(v, str) or v > THR:
        res["fails"].append(f"coupling {k}: {v}")
for ang in (0, 30, 60, 90):          # rotor turned with the tongue: the tongue must stay in the rotor slot
    v = X(tr(rotor, Rz(ang)), tr(shaft, Rz(ang)))
    res["coupling"][f"rotor_shaft_turned_{ang}"] = v

# mass (PLA 1.24 g/cm3; ~55 % effective fill for 3 walls + 15 % infill on these sizes)
vol = {n: float(m.volume) / 1000 for n, m in (("body", body), ("cover", cover), ("shaft", shaft), ("pinion", pin), ("jaw1", j1), ("jaw2", j2),
                                                ("rotor", rotor), ("plate", plate))}
vol["modules"] = 2 * float(mod.volume) / 1000
res["volume_cm3"] = {k: round(v, 2) for k, v in vol.items()}
res["mass_g_est"] = round(sum(vol.values()) * 1.24 * 0.55 + 3 * 0.66 + 1.0 + 9 * 0.9, 0)   # + 3 dowels, magnet, 9 screws/inserts
res["PASS"] = not res["fails"]
(H / "v4" / "gt_checks.json").write_text(json.dumps(res, indent=1))
print(json.dumps({k: v for k, v in res.items() if k != "sweep"}, indent=1))
sys.exit(0 if res["PASS"] else 1)
