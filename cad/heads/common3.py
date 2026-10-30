# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Shared bits for the heads_x3 reel heads (HI5, STAMP1, SCOOP1).  2026-10-29, heads-x3 for pm-eng.
Run with python.  Nothing here is printed, bought or run on the arm.

TOOL frame = GT1 / tools_x2: coupling face z = 0, tool along -z, +x = UP in the dock.  d = -z is the depth below the face.
The coupling half is GT1's, built by importing tools_x2/common.py (plate, 3 dowels, rotor_v4, gt_shaft, neck_core) - not edited.
LIGHT HOUSING: CAM1's head housing (40 sq, solid) is 10.5 g; this one is a 2 mm tube R17 + 4 insert bosses clipped to the neck's
36 sq outline (about 4 g), same stop-dog cavity R15, same two stop ribs, same 4 M3 insert pattern (+-14, +-14) underneath.
"""
import json
import sys
from math import cos, sin, radians
from pathlib import Path

import numpy as np
import trimesh
from build123d import (Box, Cylinder, Cone, Plane, Pos, Rot, Axis, Compound, export_stl, export_step, extrude, fillet,
                       chamfer, RegularPolygon, Location)

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H.parent / "tools_x2"))
from common import B, V, GT, bx, rbox, coupling, coupling_hardware, neck_core, insert_leads  # noqa: E402
import build_tools_x as X  # noqa: E402

RHO_PLA, RHO_TPU = 0.95e-3, 1.0e-3            # g/mm3 effective printed (PLA: GT1 sliced 109.6 g / 116.8 cm3; TPU hand: walls + pockets, ~87 % of 1.15)
HOUS_Z = (-21.0, -31.5)
CAV_R = X.DOG["cav_r"]
INS = [(14.0, 14.0), (-14.0, 14.0), (14.0, -14.0), (-14.0, -14.0)]
KEEL = dict(hy=5.0, x0=-39.0, z=(-25.0, -41.0))     # dock rest: x -39 over the tail rib (same as JG1's keel)

# colours (rgb)
COL = {"pla_white": (233, 233, 236), "pla_black": (58, 61, 66), "tpu": (29, 30, 33), "steel": (184, 188, 194),
       "brass": (201, 161, 59), "magnet": (138, 143, 153), "foam": (214, 120, 40), "spring": (150, 155, 162)}


def outline(z0, z1, boss_r=5.0, tube_r=17.0):
    """Tube disc R17 + 4 insert bosses, clipped to the neck's 36 sq with 1.5 chamfers (solid, no cavity)."""
    h = abs(z1 - z0)
    zc = (z0 + z1) / 2
    s = Pos(0, 0, zc) * Cylinder(tube_r, h)
    for x, y in INS:
        s += Pos(x, y, zc) * Cylinder(boss_r, h)
    clip = Pos(0, 0, zc) * Box(36, 36, h + 2)
    clip = chamfer(clip.edges().filter_by(Axis.Z), 1.5)
    return s & clip


def housing(keel=True):
    """Light stop-dog housing z -21..-31.5 on GT1's neck core, with the dock keel."""
    h = neck_core()
    shell = outline(HOUS_Z[0] + 0.01, HOUS_Z[1])
    cav_top = X.DOG["cav_z"]
    shell -= Pos(0, 0, (cav_top + HOUS_Z[1]) / 2 - 0.5) * Cylinder(CAV_R, cav_top - HOUS_Z[1] + 1.0)
    h += shell
    h -= Pos(0, 0, (HOUS_Z[0] + cav_top) / 2) * Cylinder(GT.SHAFT_BORE_D / 2, HOUS_Z[0] - cav_top + 0.4)
    # PEN1's flat stop ribs at GT1's stop angles
    hw = X.DOG["arm_w"] / 2
    zz = (HOUS_Z[1] + 0.01, cav_top + 0.01)
    for ang, sy in ((X.DOG["closed_deg"], 1), (X.DOG["open_deg"], -1)):
        a0, a1 = (0.0, 40.0) if sy > 0 else (-40.0, 0.0)
        rib = B.sector(X.RIB_R0, CAV_R + 0.01, a0, a1, *zz) & bx((0, 20), (sy * hw, sy * 20), (zz[0] - 1, zz[1] + 1))
        h += Rot(0, 0, ang) * rib
    for x, y in INS:
        h -= Pos(x, y, HOUS_Z[1] + 6.7 / 2 - 0.01) * Cylinder(2.0, 6.7)
    h = insert_leads(h, [(x, y, HOUS_Z[1], 1) for x, y in INS])
    if keel:
        h += dock_keel()
    return h


def dock_keel():
    """I-beam rest foot: end plate at x -39 (the dock rest face), web, two flanges; welded into the housing wall."""
    x0, x1 = KEEL["x0"], -15.5
    z0, z1 = KEEL["z"]
    hy = KEEL["hy"]
    k = bx((x0, x0 + 2.0), (-hy, hy), (z0, z1))
    k += bx((x0, x1), (-1.2, 1.2), (z0, z1))
    k += bx((x0, x1), (-hy, hy), (z0, z0 - 2.0))
    k += bx((x0, x1), (-hy, hy), (z1 + 2.0, z1))
    return k


# ---------------------------------------------------------------- hardware solids
def _at(pos, ax, shape):
    return Plane(origin=tuple(pos), z_dir=tuple(ax)) * shape


def hw_solid(spec, kind, pos, ax, L):
    """pos = seat point, ax = out of the host toward the head, L = length into the host (tools_x2 convention)."""
    if kind == "bolt":
        if "countersunk" in spec:
            s = Pos(0, 0, -0.85) * Cone(1.5 + 1.4, 1.5, 1.7) if False else Pos(0, 0, -0.85) * Cone(2.9, 1.5, 1.7)
            s += Pos(0, 0, -1.7 - (L - 1.7) / 2) * Cylinder(1.5, L - 1.7)
        else:
            s = Pos(0, 0, 0.825) * Cylinder(2.85, 1.65) + Pos(0, 0, -L / 2) * Cylinder(1.5, L)
        return _at(pos, ax, s)
    if kind == "insert":
        return _at(pos, ax, Pos(0, 0, -L / 2) * Cylinder(2.3, L))
    if kind == "nut":
        n = extrude(RegularPolygon(radius=3.175, side_count=6), amount=2.4)
        n -= Cylinder(1.4, 6)
        return _at(pos, ax, Pos(0, 0, -2.4) * n)
    if kind == "pin":
        return _at(pos, ax, Pos(0, 0, -L / 2) * Cylinder(1.5, L))
    if kind == "magnet":
        return _at(pos, ax, Pos(0, 0, -L / 2) * Cylinder(3.0, L))
    if kind == "washer":
        w = Cylinder(3.5, 0.5) - Cylinder(1.7, 1)
        return _at(pos, ax, Pos(0, 0, 0.25) * w)
    raise ValueError(kind)


# (mass g) per hardware piece
HW_G = {"bolt": 0.5, "insert": 0.25, "nut": 0.2, "pin": 0.67, "magnet": 0.64, "washer": 0.1}


def spring_mesh(od, wire, free_len, coils_total, base=(0, 0, 0), direction=(0, 0, -1)):
    """Helical compression spring as a tube mesh, axis from `base` along `direction` (free length)."""
    rm = (od - wire) / 2
    n = int(coils_total * 36)
    t = np.linspace(0, coils_total * 2 * np.pi, n)
    z = wire / 2 + (free_len - wire) * t / t[-1]
    path = np.stack([rm * np.cos(t), rm * np.sin(t), z], 1)
    tang = np.gradient(path, axis=0); tang /= np.linalg.norm(tang, axis=1)[:, None]
    ref = np.array([0, 0, 1.0])
    nrm = np.cross(tang, ref); nrm /= np.linalg.norm(nrm, axis=1)[:, None]
    bn = np.cross(tang, nrm)
    m_ = 8
    ang = np.linspace(0, 2 * np.pi, m_, endpoint=False)
    verts = np.concatenate([path + wire / 2 * (np.cos(a) * nrm + np.sin(a) * bn) for a in ang])
    faces = []
    for j in range(m_):
        for i in range(n - 1):
            a, b = j * n + i, ((j + 1) % m_) * n + i
            faces += [(a, b, a + 1), (b, b + 1, a + 1)]
    m = trimesh.Trimesh(verts, np.array(faces), process=False)
    d = np.asarray(direction, float); d /= np.linalg.norm(d)
    T = np.eye(4)
    z_ax = np.array([0, 0, 1.0])
    v = np.cross(z_ax, d)
    if np.linalg.norm(v) > 1e-9:
        T[:3, :3] = trimesh.transformations.rotation_matrix(np.arccos(np.clip(z_ax @ d, -1, 1)), v)[:3, :3]
    elif d[2] < 0:
        T[:3, :3] = np.diag([1, -1, -1])
    T[:3, 3] = base
    m.apply_transform(T)
    return m


# ---------------------------------------------------------------- export + mass
class Head:
    """Collects printed parts / bought bodies / hardware, then writes STL, STEP, GLB and the mass sheet."""

    def __init__(self, name, folder):
        self.name = name
        self.dir = H / folder
        (self.dir / "stl").mkdir(parents=True, exist_ok=True)
        self.parts = {}      # name -> dict(shape|mesh, mat, rho|mass, color, kind, label)
        self.hw = []         # (id, spec, pos, ax, L, kind, host)

    def printed(self, name, shape, mat="pla", color="pla_white", label=None):
        self.parts[name] = dict(shape=shape, mat=mat, color=color, kind="printed", label=label or name)

    def bought(self, name, mesh_or_shape, mass_g, color, label, com=None):
        d = dict(mass=mass_g, color=color, kind="bought", label=label)
        if isinstance(mesh_or_shape, trimesh.Trimesh):
            d["mesh"] = mesh_or_shape
        else:
            d["shape"] = mesh_or_shape
        if com is not None:
            d["com"] = com
        self.parts[name] = d

    def add_hw(self, hid, spec, pos, ax, L, kind, host):
        self.hw.append((hid, spec, tuple(map(float, pos)), tuple(map(float, ax)), float(L), kind, host))

    def coupling_parts(self, rotor_shaft_extra=None):
        c = coupling()
        self.printed("plate", c["plate"], color="pla_black", label="coupling plate (GT1's)")
        self.printed("rotor", c["rotor"], color="pla_black", label="rotor (GT1's)")
        self.printed("shaft", c["shaft"], color="pla_black", label="Oldham shaft (GT1's)")
        self.bought("dowels", c["pins"], 3 * 0.67, "steel", "3x 3x12 steel dowel")
        for hid, spec, pos, ax, L, kind in coupling_hardware():
            if kind == "pin" or kind == "magnet":
                if kind == "magnet":
                    self.add_hw(hid, spec, pos, ax, L, kind, "housing")
                else:
                    self.hw.append((hid, spec, pos, ax, L, kind, "plate"))
                continue
            host = "plate" if hid.startswith("plate_screw") else "housing"
            self.add_hw(hid, spec, pos, ax, L, kind, host)

    def finish(self, extra=None, notes=None):
        out = {"name": self.name, "parts": {}, "hardware": []}
        meshes = {}
        for n, p in self.parts.items():
            if "shape" in p:
                s = p["shape"]
                export_stl(s, str(self.dir / "stl" / f"{n}.stl"), tolerance=0.01, angular_tolerance=0.1)
                m = trimesh.load(str(self.dir / "stl" / f"{n}.stl"))
            else:
                m = p["mesh"]
                m.export(str(self.dir / "stl" / f"{n}.stl"))
            meshes[n] = m
        mass = []   # (name, g, com xyz)
        for n, p in self.parts.items():
            m = meshes[n]
            if p["kind"] == "printed":
                rho = RHO_TPU if p["mat"] == "tpu" else RHO_PLA
                g = m.volume * rho
                c = m.center_mass
            else:
                g = p["mass"]
                c = np.array(p.get("com", m.bounds.mean(axis=0)))
            mass.append((n, g, c))
            bb = m.bounds
            out["parts"][n] = {"label": p["label"], "kind": p["kind"], "g": round(g, 2), "cm3": round(m.volume / 1000, 3),
                               "watertight": bool(m.is_watertight), "bbox": [round(float(v), 2) for v in (*bb[0], *bb[1])],
                               "color": p["color"]}
        hw_meshes = {}
        for hid, spec, pos, ax, L, kind, host in self.hw:
            s = hw_solid(spec, kind, pos, ax, L)
            f = self.dir / "stl" / f"hw_{hid}.stl"
            export_stl(s, str(f), tolerance=0.02, angular_tolerance=0.2)
            hw_meshes[hid] = (trimesh.load(str(f)), kind, spec)
            g = HW_G["pin" if kind == "pin" else kind] if not (kind == "bolt" and L >= 12) else 0.5 + 0.03 * (L - 8)
            c = np.array(pos) - np.array(ax) / np.linalg.norm(ax) * L / 2
            if kind in ("pin", "magnet"):
                pass
            if kind == "pin" and any(n == "dowels" for n in self.parts):
                g = 0.0                        # counted in "dowels"
            mass.append((f"hw:{hid}", g, c))
            out["hardware"].append({"id": hid, "spec": spec, "pos": pos, "axis": ax, "len": L, "kind": kind, "host": host})
        tot = sum(g for _, g, _ in mass)
        com = sum(g * c for _, g, c in mass) / tot
        d = float(-com[2])
        out["mass_g"] = round(tot, 1)
        out["com_mm"] = [round(float(v), 1) for v in com]
        out["com_dist_from_coupling_face_mm"] = round(d, 1)
        out["coupling_moment_Nmm"] = round(tot / 1000 * 9.81 * d, 1)
        allm = [m for m in meshes.values()]
        bb = np.array([m.bounds for m in allm])
        lo, hi = bb[:, 0].min(0), bb[:, 1].max(0)
        out["envelope_mm"] = {"x": [round(float(lo[0]), 1), round(float(hi[0]), 1)], "y": [round(float(lo[1]), 1), round(float(hi[1]), 1)],
                              "z": [round(float(lo[2]), 1), round(float(hi[2]), 1)],
                              "size_xyz": [round(float(hi[i] - lo[i]), 1) for i in range(3)]}
        out["mass_rows"] = [(n, round(g, 2)) for n, g, _ in mass if g > 0]
        out["rho"] = {"pla_g_per_cm3_effective": RHO_PLA * 1000, "tpu_g_per_cm3_effective": RHO_TPU * 1000}
        if extra:
            out.update(extra)
        # GLB: printed + bought + hardware as separate nodes
        sc = trimesh.Scene()
        for n, m in meshes.items():
            mm = m.copy(); mm.visual.face_colors = COL[self.parts[n]["color"]] + (255,)
            sc.add_geometry(mm, node_name=n)
        for hid, (m, kind, spec) in hw_meshes.items():
            mm = m.copy()
            mm.visual.face_colors = COL["brass" if kind == "insert" else "magnet" if kind == "magnet" else "steel"] + (255,)
            sc.add_geometry(mm, node_name=f"hw_{hid}")
        sc.export(str(self.dir / f"{self.name.lower()}.glb"))
        # STL of everything merged (assembled, TOOL frame)
        allmesh = trimesh.util.concatenate([m for m in meshes.values()] + [m for m, _, _ in hw_meshes.values()])
        allmesh.export(str(self.dir / f"{self.name.lower()}.stl"))
        # STEP: printed + bought + hardware solids
        shapes = []
        for n, p in self.parts.items():
            if "shape" in p:
                shapes.append(p["shape"])
        for hid, spec, pos, ax, L, kind, host in self.hw:
            shapes.append(hw_solid(spec, kind, pos, ax, L))
        export_step(Compound(children=shapes), str(self.dir / f"{self.name.lower()}.step"))
        (self.dir / "parts.json").write_text(json.dumps(out, indent=1))
        return out
