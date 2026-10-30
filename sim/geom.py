# SPDX-License-Identifier: GPL-3.0-or-later
"""Collision geometry for the tool-changer sim (MuJoCo), generated from the CAD parameters.

Every dimension comes from toolchanger/cad/build_changer.py (imported as B) through ``params()``; v4 changes are
overrides in that dict, so the sim and the CAD share one source of numbers.  Non-convex CAD features are split into
small convex hulls (MuJoCo collides mesh geoms as convex hulls): the lip and skirt in 3-6 deg bins, the ramps and
crests in 1 deg bins (two radial bands where the dowel trough cuts them), the dock wall in 5 convex blocks.

Frames (mm here, metres in the XML):
  MASTER  = MJCF ``gripper`` body: roll axis = z through (0, 0); the coupling points -z (CAD gripper frame with
            HORN_CX removed).  Window k is centred on phi = 120 k.
  TOOL    = coupling face z = 0, tool along -z, pins at 120 k + PIN_PHASE.  LOCKED: tool frame == master frame
            shifted to z = Z_TOOL_FACE_LOCK.
  DOCK    = origin on the wall's arm-side face at the tool axis; +x along the tool axis away from the arm, +z up.
"""
from __future__ import annotations

import sys
from math import cos, sin, radians, sqrt, tan, asin, degrees
from pathlib import Path

import numpy as np

CAD = Path(__file__).resolve().parents[1] / "cad"
sys.path.insert(0, str(CAD))
try:
    import build123d  # noqa: F401
except ImportError:      # only the module-level constants are needed: stub the CAD kernel names
    import types
    _stub = types.ModuleType("build123d")
    for _n in ("Box Cylinder Cone Pos Rot Polygon extrude export_stl export_step Plane Axis Sphere chamfer mirror").split():
        setattr(_stub, _n, None)
    sys.modules["build123d"] = _stub
import build_changer as B  # noqa: E402

M = 1e-3  # mm -> m


def params(**over):
    """All geometry numbers in one dict (mm / deg).  Keys mirror build_changer.py."""
    p = dict(
        Z_LIP_TOP=B.Z_LIP_TOP, Z_LIP_BOT=B.Z_LIP_BOT, Z_RING_TOP=B.Z_RING_TOP, R_RING=B.R_RING, R_BORE=B.R_BORE,
        R_CAV=B.R_CAV, BORE_CHAMFER=B.BORE_CHAMFER, WIN=B.WIN, R_WIN=B.R_WIN, RAMP=B.RAMP, H_CREST=B.H_CREST,
        PHI_V=B.PHI_V, PHI_STOP=B.PHI_STOP, STOP_T=B.STOP_T, H_STOP=B.H_STOP, DOWEL_D=B.DOWEL_D, DOWEL_L=B.DOWEL_L,
        DOWEL_RIN=B.DOWEL_RIN, H_DOWEL=B.H_DOWEL, PIN_D=B.PIN_D, PIN_RIN=B.PIN_RIN, PIN_L=B.PIN_L,
        TOOL_PIN_Z=B.TOOL_PIN_Z, R_PL=B.R_PL, PL_ANG=B.PL_ANG, PL_BALL_D=B.PL_BALL_D, PL_PROTRUDE=B.PL_PROTRUDE,
        SPRING_K=B.SPRING["rate_N_per_mm"], SPRING_FREE=B.SPRING["free"], SPRING_INSTALLED=8.05,
        PL_TRAVEL=3.0, SKIRT_H=B.SKIRT_H, R_SKIRT_IN=B.R_SKIRT_IN, R_SKIRT_OUT=B.R_SKIRT_OUT,
        R_SKIRT_MOUTH=B.R_SKIRT_MOUTH, TP_T=B.TP_T, R_TP=B.R_TP, R_SPIG=B.R_SPIG, SPIG_TOP=B.SPIG_TOP,
        SPIG_CHAMFER=B.SPIG_CHAMFER, R_SPIG_BORE=B.R_SPIG_BORE, LIP_IN_R=B.LIP_IN_R, LIP_IN_T=B.LIP_IN_T,
        ROTOR_R=B.ROTOR_R, SLOT_W=B.SLOT_W, SLOT_D=B.SLOT_D, SLOT_LEAD=B.SLOT_LEAD, ROTOR_TOP=B.ROTOR_TOP,
        ROTOR_FLOAT=1.0, BLADE_T=B.BLADE_T, BLADE_W=B.BLADE_W, Z_CAR_BOT=B.Z_CAR_BOT, Z_BLADE_TIP=B.Z_BLADE_TIP,
        CAR_BODY_R=B.CAR_BODY_R, BLADE_SPRING=0.0,       # v4: axial blade travel (mm) on a spring; 0 = rigid (v3.1)
        NECK_SQ=B.NECK_SQ, NECK_H=B.NECK_H, NECK_CH=1.5,
        Z_TOOL_FACE_ENTRY=B.Z_TOOL_FACE_ENTRY, Z_TOOL_FACE_LOCK=B.Z_TOOL_FACE_LOCK,
        WALL_T=B.WALL_T, WALL_HW=B.WALL_HW, WALL_TOP=B.WALL_TOP, DOCK_H=B.DOCK_H, SLOT_CLR=B.SLOT_CLR,
        PLATE_GAP=B.PLATE_GAP, DOCK_CH=3.0, FLARE_DEG=45.0,
        PIN_FUNNEL=(6.0, 1.0),                          # extra deg each side of a window x depth at the lip bottom
        TOOL_MASS=0.102, TOOL_COM=-38.0, LUBE=1.0, SLOT_LAND=None, TOP_BEVEL=0.0, TAIL_REST=None, BODY_HALF=39.0, MAGNET_N=0.0, RELIEF_RIN=None, ROTOR_DRAG=0.01, ROTOR_RANGE=(-4.0, 140.0),
    )
    p.update(over)
    # 2026-10-24 master v4 (cad/build_master_v4.py -> v4/master_v4.json): the hub grew 6 -> 14.5 mm for the servo cable
    # plug, so on the arm the roll-horn plane sits at z = +MASTER_DROP in the v3.1 CAD frame.  The gripper body frame
    # is fixed to the horn, so every master / tool z below the hub moves MASTER_DROP farther from the wrist (all the
    # absolute z keys; the rest are relative heights).  MASTER_DROP = 0 reproduces the pre-10/24 geometry.
    drop = float(p.get("MASTER_DROP", 0.0))
    if drop and not p.get("_dropped"):
        for k in ("Z_LIP_TOP", "Z_LIP_BOT", "Z_RING_TOP", "Z_CAR_BOT", "Z_BLADE_TIP", "Z_TOOL_FACE_ENTRY", "Z_TOOL_FACE_LOCK"):
            p[k] = p[k] - drop
        p["_dropped"] = True
    return p


def master_drop_from_cad(default: float = 8.5) -> float:
    """MASTER_DROP from cad/v4/master_v4.json (the master-parts agent's build), else the 10/24 value."""
    import json
    try:
        return float(json.loads((CAD / "v4" / "master_v4.json").read_text())["params"]["MASTER_DROP"])
    except Exception:  # noqa: BLE001
        return default


# ------------------------------------------------------------------ helpers
def _fmt(a):
    return " ".join(f"{v:.6g}" for v in np.asarray(a).ravel())


class Geoms:
    """Collects <mesh> assets and <geom> elements for one body."""

    def __init__(self, prefix, assets, cls):
        self.prefix, self.assets, self.cls, self.items, self.n = prefix, assets, cls, [], 0

    def hull(self, pts_mm, **kw):
        name = f"{self.prefix}{self.n}"
        self.n += 1
        self.assets.append(f'<mesh name="{name}" vertex="{_fmt(np.asarray(pts_mm) * M)}"/>')
        self.items.append(self._g(f'type="mesh" mesh="{name}"', name, kw))

    def prim(self, gtype, size_mm, pos_mm=(0, 0, 0), **kw):
        name = f"{self.prefix}{self.n}"
        self.n += 1
        extra = f'type="{gtype}" size="{_fmt(np.asarray(size_mm) * M)}" pos="{_fmt(np.asarray(pos_mm) * M)}"'
        if "fromto" in kw:
            ft = kw.pop("fromto")
            extra = f'type="{gtype}" size="{_fmt(np.asarray(size_mm) * M)}" fromto="{_fmt(np.asarray(ft) * M)}"'
        self.items.append(self._g(extra, name, kw))

    def _g(self, core, name, kw):
        attrs = " ".join(f'{k}="{v}"' for k, v in kw.items())
        return f'<geom name="{name}" class="{self.cls}" {core} {attrs}/>'

    def xml(self):
        return "\n".join(self.items)


def sector_profile(prof_rz, a0, a1, na=1):
    """Convex hull points: a 2-D (r, z) profile swept from angle a0 to a1 (deg).  Small spans only."""
    pts = []
    for i in range(na + 1):
        a = radians(a0 + (a1 - a0) * i / na)
        for r, z in prof_rz:
            pts.append((r * cos(a), r * sin(a), z))
    return pts


def floor_chunk(r0, r1, a0, a1, z0, hfun):
    """Hull of the floor relief between radii r0..r1, angles a0..a1, from z0 up to z0 + hfun(r, phi) (corners)."""
    pts = []
    for a in (a0, a1):
        for r in (r0, r1):
            x, y = r * cos(radians(a)), r * sin(radians(a))
            pts.append((x, y, z0 - 0.01))
            pts.append((x, y, z0 + max(hfun(r, a), 0.0)))
    return pts


# ------------------------------------------------------------------ master
def master(p, assets):
    """Geoms fixed to the gripper body (ring, lip, ramps, dowels, stops, skirt) and the plunger / blade bodies."""
    g = Geoms("m", assets, "petg")
    zt, zb = p["Z_LIP_TOP"], p["Z_LIP_BOT"]
    fun_a, fun_d = p["PIN_FUNNEL"]
    ch = p["BORE_CHAMFER"]
    # lip: 3-deg bins; windows cut r < R_WIN; pin funnel 1 mm deep near the windows; 45-deg bore chamfer
    step = 3.0
    for i in range(int(360 / step)):
        a0, a1 = i * step, (i + 1) * step
        am = (a0 + a1) / 2
        dk = min(abs(((am - 120 * k) + 180) % 360 - 180) for k in range(3))
        if dk < p["WIN"]:
            prof = [(p["R_WIN"], zb), (p["R_RING"], zb), (p["R_RING"], zt), (p["R_WIN"], zt)]
        else:
            zbf = zb + (fun_d if dk < p["WIN"] + fun_a else 0.0)
            rin = p["R_BORE"]
            if zbf > zb:   # funnel: bottom raised for r < R_WIN + 2.5 (keep the outer rim)
                g.hull(sector_profile([(p["R_WIN"] + 2.5, zb), (p["R_RING"], zb), (p["R_RING"], zt), (p["R_WIN"] + 2.5, zt)], a0, a1))
                prof = [(rin + max(ch - fun_d, 0), zbf), (p["R_WIN"] + 2.5, zbf), (p["R_WIN"] + 2.5, zt), (rin, zt), (rin, zb + ch)]
            else:
                prof = [(rin + ch, zb), (p["R_RING"], zb), (p["R_RING"], zt), (rin, zt), (rin, zb + ch)]
        g.hull(sector_profile(prof, a0, a1))
    # cavity outer wall (pins never reach it unless the tool tilts)
    for i in range(36):
        g.hull(sector_profile([(p["R_CAV"], zt), (p["R_RING"], zt), (p["R_RING"], p["Z_RING_TOP"]), (p["R_CAV"], p["Z_RING_TOP"])], i * 10, i * 10 + 10))
    # floor relief per window: ramp -> crest (cut by the dowel trough) -> stop wall; stop block behind the far dowel
    r0 = p.get("RELIEF_RIN") or p["R_BORE"]                 # v4: relief ring around the spigot in the cavity
    r1, rm = p["R_CAV"], (r0 + p["R_CAV"]) / 2
    ra, rb = p["RAMP"]
    tr_top = p["H_CREST"] + 0.8
    for k in range(3):
        a0k = 120 * k

        def h(r, a, a0k=a0k):
            ph = a - a0k
            if ph < ra:
                hh = 0.0
            elif ph < rb:
                hh = p["H_CREST"] * (ph - ra) / (rb - ra)
            else:
                hh = p["H_CREST"]
            t = r * sin(radians(ph - p["PHI_V"]))                         # tangential distance from the V line
            trough = 0.3 + abs(t) * tan(radians(30))
            if trough < tr_top:
                hh = min(hh, trough)
            return hh
        a = ra
        while a < p["PHI_STOP"][0] - 1e-6:
            a1 = min(a + 1.0, p["PHI_STOP"][0])
            near_v = abs((a + a1) / 2 - p["PHI_V"]) < 20
            for (q0, q1) in ([(r0, rm), (rm, r1)] if near_v else [(r0, r1)]):
                if max(h(q0, a0k + a), h(q1, a0k + a), h(q0, a0k + a1), h(q1, a0k + a1)) > 0.02:
                    g.hull(floor_chunk(q0, q1, a0k + a, a0k + a1, zt, h))
            a = a1
        for j in range(int((p["PHI_STOP"][1] - p["PHI_STOP"][0]) / 5)):
            aa = a0k + p["PHI_STOP"][0] + 5 * j
            g.hull(floor_chunk(r0, r1, aa, aa + 5, zt, lambda r, a: p["H_STOP"]))
        # stop block: radial-parallel box, tangential STOP_T from the V line, r R_BORE..R_CAV+0.3
        c = radians(a0k + p["PHI_V"])
        tm, tw = (p["STOP_T"][0] + p["STOP_T"][1]) / 2, (p["STOP_T"][1] - p["STOP_T"][0]) / 2
        rmid, rw = (r0 + r1 + 0.3) / 2, (r1 + 0.3 - r0) / 2
        pos = (rmid * cos(c) - tm * sin(c), rmid * sin(c) + tm * cos(c), zt + p["H_STOP"] / 2)
        g.prim("box", (rw, tw, p["H_STOP"] / 2), pos, euler=f"0 0 {c:.6f}")
        # steel dowels (the V), radial, tangential offset +-D/2
        zd = zt + p["H_DOWEL"]
        for s in (-1, 1):
            q0, q1 = p["DOWEL_RIN"], p["DOWEL_RIN"] + p["DOWEL_L"]
            t = s * p["DOWEL_D"] / 2
            ft = (q0 * cos(c) - t * sin(c), q0 * sin(c) + t * cos(c), zd, q1 * cos(c) - t * sin(c), q1 * sin(c) + t * cos(c), zd)
            g.items.append(f'<geom name="mdw{k}{"ab"[s > 0]}" class="steel" type="cylinder" size="{p["DOWEL_D"] / 2 * M:.6g}" fromto="{_fmt(np.array(ft) * M)}"/>')
    # skirt funnel: r R_SKIRT_MOUTH at the bottom -> R_SKIRT_IN at the lip bottom
    for i in range(60):
        g.hull(sector_profile([(p["R_SKIRT_MOUTH"], zb - p["SKIRT_H"]), (p["R_SKIRT_OUT"], zb - p["SKIRT_H"]),
                               (p["R_SKIRT_OUT"], zb), (p["R_SKIRT_IN"], zb)], i * 6, i * 6 + 6))
    xml = [g.xml()]
    # plungers: slide along +z (into the master), preloaded spring, steel ball
    F0 = p["SPRING_K"] * (p["SPRING_FREE"] - p["SPRING_INSTALLED"])
    k_si = p["SPRING_K"] * 1000.0
    zc = zb - p["PL_PROTRUDE"] + p["PL_BALL_D"] / 2
    for i, a in enumerate(p["PL_ANG"]):
        x, y = p["R_PL"] * cos(radians(a)), p["R_PL"] * sin(radians(a))
        xml.append(f'<body name="plunger{i}" pos="{x * M:.6g} {y * M:.6g} {zc * M:.6g}">'
                   f'<joint name="pl{i}" type="slide" axis="0 0 1" range="0 {p["PL_TRAVEL"] * M:.6g}" stiffness="{k_si:.6g}" '
                   f'springref="{-F0 / k_si:.6g}" damping="5" armature="0.02" frictionloss="0.2" solreflimit="0.001 1" solimplimit="0.95 0.99 0.0001"/>'
                   f'<geom name="ball{i}" class="steel" type="sphere" size="{p["PL_BALL_D"] / 2 * M:.6g}" mass="0.0015"/></body>')
    # blade carrier on the gripper servo (hinge about z); optional axial spring (v4)
    bt, bw = p["BLADE_T"] / 2, p["BLADE_W"] / 2
    zc0, ztip = p["Z_CAR_BOT"], p["Z_BLADE_TIP"]
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            pts += [(sx * bt, sy * bw, zc0), (sx * (bt - 1.0), sy * bw, ztip), (sx * bt, sy * bw, ztip + 1.0)]
    assets.append(f'<mesh name="blade" vertex="{_fmt(np.array(pts) * M)}"/>')
    slide = ""
    if p["BLADE_SPRING"] > 0:
        slide = (f'<joint name="blade_ax" solreflimit="0.001 1" solimplimit="0.95 0.99 0.0001" type="slide" axis="0 0 1" range="0 {p["BLADE_SPRING"] * M:.6g}" stiffness="300" '
                 f'springref="-0.004" damping="2" armature="0.002"/>')
    xml.append(f'<body name="blade" pos="0 0 0"><joint name="blade" type="hinge" axis="0 0 1" damping="0.05" armature="0.005" frictionloss="0.01"/>'
               f'{slide}<geom name="bladeg" class="petg" type="mesh" mesh="blade" mass="0.004"/>'
               f'<geom name="carrier" class="petg" type="cylinder" size="{p["CAR_BODY_R"] * M:.6g} 0.002" pos="0 0 {(zc0 + 2) * M:.6g}" mass="0.004"/></body>')
    return "\n".join(xml)


# ------------------------------------------------------------------ tool
def tool(p, assets, name="tool", pos="0 0 0", quat="1 0 0 0"):
    """Free body in the TOOL frame (face z = 0, tool along -z).  Rotor = child body with x/y float + hinge."""
    g = Geoms("t", assets, "pla")
    tp = p["TP_T"]
    g.prim("cylinder", (p["R_TP"], tp / 2), (0, 0, -tp / 2))
    # spigot: annulus r R_SPIG_BORE..R_SPIG with a top chamfer, plus the inward retaining lip at the top
    st, sc = p["SPIG_TOP"], p["SPIG_CHAMFER"]
    for i in range(36):
        g.hull(sector_profile([(p["R_SPIG_BORE"], 0), (p["R_SPIG"], 0), (p["R_SPIG"], st - sc), (p["R_SPIG"] - sc, st),
                               (p["R_SPIG_BORE"], st)], i * 10, i * 10 + 10))
        g.hull(sector_profile([(p["LIP_IN_R"], st - p["LIP_IN_T"]), (p["R_SPIG_BORE"] + 0.01, st - p["LIP_IN_T"]),
                               (p["R_SPIG_BORE"] + 0.01, st), (p["LIP_IN_R"], st)], i * 10, i * 10 + 10))
    # square neck with chamfered long edges, body block beyond it (stops axial slide against the wall's far face)
    h, c = p["NECK_SQ"] / 2, p["NECK_CH"]
    oct2 = [(h, h - c), (h - c, h), (-h + c, h), (-h, h - c), (-h, -h + c), (-h + c, -h), (h - c, -h), (h, -h + c)]
    z0, z1 = -tp - p["NECK_H"], -tp
    g.hull([(x, y, z) for x, y in oct2 for z in (z0, z1)])
    g.prim("box", (p["BODY_HALF"], 18.0, 8.0), (0, 0, z0 - 8.0))
    # steel pins: capsules radial at z TOOL_PIN_Z (only the part outside the spigot matters)
    pins = []
    for k in range(3):
        a = radians(120 * k + p["PHI_V"])
        r0, r1 = p["R_SPIG"] - 0.5 + p["PIN_D"] / 2, p["PIN_RIN"] + p["PIN_L"] - p["PIN_D"] / 2
        ft = (r0 * cos(a), r0 * sin(a), p["TOOL_PIN_Z"], r1 * cos(a), r1 * sin(a), p["TOOL_PIN_Z"])
        pins.append(f'<geom name="pin{k}" class="steel_tool" type="capsule" size="{p["PIN_D"] / 2 * M:.6g}" fromto="{_fmt(np.array(ft) * M)}" mass="0.0007"/>')
    # rotor: two slot cheeks (disc segments with a 45-deg lead-in) on a cylinder; slot along rotor y, width in x
    ga = Geoms("r", assets, "pla")
    R, sw, sd, lead, top = p["ROTOR_R"], p["SLOT_W"] / 2, p["SLOT_D"], p["SLOT_LEAD"], p["ROTOR_TOP"]
    for sx in (-1, 1):
        pts = []
        for i in range(13):
            a = radians(-90 + 180 * i / 12)
            x, y = R * cos(a), R * sin(a)
            if x >= sw:
                pts.append((sx * x, y, top - sd))
            if x >= sw + lead:
                pts.append((sx * x, y, top))
        ymax = sqrt(R * R - sw * sw)
        for y in (-ymax, ymax):
            pts += [(sx * sw, y, top - sd), (sx * sw, y, top - lead), (sx * (sw + lead), y, top)]
        ga.hull(pts)
    ga.prim("cylinder", (R, 2.0), (0, 0, top - sd - 2.0))
    fl = p["ROTOR_FLOAT"] * M
    lo, hi = p["ROTOR_RANGE"]
    rotor = (f'<body name="rotor" pos="0 0 0">'
             f'<joint name="rot_x" solreflimit="0.001 1" solimplimit="0.95 0.99 0.0001" type="slide" axis="1 0 0" range="{-fl:.6g} {fl:.6g}" damping="1" frictionloss="0.05" armature="0.002"/>'
             f'<joint name="rot_y" solreflimit="0.001 1" solimplimit="0.95 0.99 0.0001" type="slide" axis="0 1 0" range="{-fl:.6g} {fl:.6g}" damping="1" frictionloss="0.05" armature="0.002"/>'
             f'<joint name="rot_z" solreflimit="0.001 1" solimplimit="0.95 0.99 0.0001" type="hinge" axis="0 0 1" range="{radians(lo):.6g} {radians(hi):.6g}" damping="0.02" '
             f'frictionloss="{p["ROTOR_DRAG"]:.6g}" armature="0.0005"/>'
             f'{ga.xml()}<inertial pos="0 0 {(top - 4) * M:.6g}" mass="0.004" diaginertia="2e-7 2e-7 2e-7"/></body>')
    m = p["TOOL_MASS"]
    inertia = f'<inertial pos="0 0 {p["TOOL_COM"] * M:.6g}" mass="{m:.6g}" diaginertia="{m * 0.0006:.6g} {m * 0.0006:.6g} {m * 0.0005:.6g}"/>'
    return (f'<body name="{name}" pos="{pos}" quat="{quat}"><freejoint name="{name}_free"/>{inertia}'
            f'{g.xml()}{"".join(pins)}{rotor}<site name="tool_face" pos="0 0 0" size="0.002"/></body>')


# ------------------------------------------------------------------ dock
def dock(p, assets):
    """Static post geoms in the DOCK frame: floor block under the slot, two side walls, two flares (all with the
    3 mm x 45-deg face chamfers around the slot outline).
    v4 options: SLOT_LAND (side-wall contact only for x 0..LAND, relieved 1.5 mm behind it, so a pitched neck cannot
    jam across the wall), TOP_BEVEL (45-deg bevel on the wall's arm-side top edge: a plate that lands on it is cammed
    toward the arm), TAIL_REST (a rib under the far end of the tool body: defines the parked pitch)."""
    g = Geoms("d", assets, "pla_dock")
    T, H, top = p["WALL_T"], p["DOCK_H"], p["WALL_TOP"]
    hw = p["NECK_SQ"] / 2 + p["SLOT_CLR"]
    zf, zn = -p["NECK_SQ"] / 2, p["NECK_SQ"] / 2
    c = p["DOCK_CH"]
    land = p.get("SLOT_LAND") or T
    bev = p.get("TOP_BEVEL", 0.0)
    fl = tan(radians(p["FLARE_DEG"]))           # dy/dz of the flare (45 deg -> 1)
    # floor block (x, z profile with chamfers), |y| < hw
    if land >= T:
        xz = [(0, -H), (T, -H), (T, zf - c), (T - c, zf), (c, zf), (0, zf - c)]
    else:       # v4: floor land x 0..land only (nothing behind it under the neck/body)
        xz = [(0, -H), (land, -H), (land, zf), (c, zf), (0, zf - c)]
    g.hull([(x, y, z) for x, z in xz for y in (-hw - 1, hw + 1)])
    for s in (-1, 1):
        if land >= T:
            xy = [(0, hw + c), (c, hw), (T - c, hw), (T, hw + c), (T, p["WALL_HW"]), (0, p["WALL_HW"])]
            g.hull([(x, s * y, z) for x, y in xy for z in (-H, zn)])
        else:   # contact land x 0..land (arm-side chamfer only), relieved +1.5 mm behind it
            xy = [(0, hw + c), (c, hw), (land, hw), (land, p["WALL_HW"]), (0, p["WALL_HW"])]
            g.hull([(x, s * y, z) for x, y in xy for z in (-H, zn)])
            xy = [(land, hw + 1.5), (T, hw + 1.5), (T, p["WALL_HW"]), (land, p["WALL_HW"])]
            g.hull([(x, s * y, z) for x, y in xy for z in (-H, zn)])
        # flare: inner line y = hw + (z - zn) * fl, chamfered at both faces; optional top bevel on the arm side
        pts = []
        for x, off in ((0, c), (c, 0), (T - c, 0), (T, c)):
            for z in (zn, top):
                zz = top - bev if (z == top and x == 0 and bev > 0) else z
                pts.append((x, s * (hw + (zz - zn) * fl + off), zz))
            ztop = top - bev if (x == 0 and bev > 0) else top
            pts += [(x, s * p["WALL_HW"], zn), (x, s * p["WALL_HW"], ztop)]
        if bev > 0:
            pts += [(bev, s * p["WALL_HW"], top), (bev, s * (hw + (top - zn) * fl), top)]
        g.hull(pts)
    if p.get("TAIL_REST") is not None:
        x0 = p["NECK_H"] - p["PLATE_GAP"] + 4.0      # from just past the wall to beyond the tool CoM
        x1 = p["NECK_H"] - p["PLATE_GAP"] + 20.0
        zt = -p["BODY_HALF"] + p["TAIL_REST"]
        g.hull([(x, y, z) for x in (x0, x1) for y in (-10, 10) for z in (-H, zt)])
    return g.xml()
