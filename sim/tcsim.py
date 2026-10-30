# SPDX-License-Identifier: GPL-3.0-or-later
"""Tool-changer physics sim: the SO-101 MJCF (sim/assets/so101) with the master on the wrist-roll body, a free tool,
and the dock post; a full pick-up + drop-off cycle under a sampled error budget.

    python tcsim.py nominal [--design v3|v4] [--video out.mp4]
    (Monte Carlo: mc.py)

Servo model (sim/physics_v2/research/PROPERTIES.md rows 26-30, real_data/FINDINGS.md): lerobot P=16 makes each joint a
~22 N.m/rad spring with ~0.2 N.m Coulomb friction (the +-0.5..1.6 deg hysteresis seen in the teleop logs), stall
2.94 N.m.  Gravity sag and the friction band therefore come out of the physics, not out of a noise term.

Designs:
  v3  = v3.1 as documented: poses taught through the leader arm with the tool mounted (the taught COMMAND includes the
        sag with the tool), plain playback, event-free fixed approach C + 1.5 mm, roll to the lock angle.
  v4  = encoder-servoed standoff (integral outer loop on Present_Position until E, then frozen so the insertion is
        compliant), axial overtravel, roll overtravel to the stop block then relax, geometry overrides from V4.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
import geom as G  # noqa: E402

MJCF = ROOT / "sim" / "assets" / "so101" / "so101_new_calib.xml"
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DEG = math.pi / 180

# v4 geometry overrides (mm / deg); filled in as the design iterates (DESIGN_v4.md)
V4 = dict(LUBE=0.6,          # PTFE dry lube on the ramps, V and ball track
          PLATE_GAP=2.0,      # plate back to wall face when seated (was 1.0): room for x error while lowering
          TOP_BEVEL=5.0,      # 45-deg bevel on the wall's arm-side top edge
          SLOT_LAND=8.0,      # side-wall + floor land (was the full 12 mm wall): no pitch jam across the wall (R2: 6 -> 8)
          SLOT_CLR=0.4,       # per side (was 0.25)
          TAIL_REST=-0.3,     # rib under the tool body, 0.3 below level (R2: not hyperstatic)
          MAGNET_N=3.0,       # R2: 2x N52 6x3 flush in the floor land + M8 steel washer flush in the neck underside
          SKIRT_H=5.5, R_SKIRT_MOUTH=40.0, R_SKIRT_OUT=41.5,   # r3: taller lead-in skirt, capture +-4 -> +-6 mm
          R_BORE=14.8,        # lip bore 0.8 mm radial clearance (was 0.3): the steel V locates; the arm lags ~0.5-1 mm
                              # while retracting, a 0.3 mm bore drags the parked tool out of the dock
          R_WIN=24.5,         # windows 0.8 mm wider so offset pins still pass (cavity stays 24.2 as in the CAD)
          PL_ANG=(75.0, 195.0, 315.0), SPRING_INSTALLED=9.05,   # R1: plungers away from the dowels; preload 10 N
          RELIEF_RIN=15.6,    # ramps / crest / stop wall start at r 15.6 (was the bore): the spigot never rubs them
          DOWEL_RIN=15.8,     # V dowels start at r 15.8 (was 14.5): dowel ends clear the spigot
          BLADE_SPRING=0.0,
          # 10/24 master v4: hub 14.5 mm (was 6) -> the whole master stack 8.5 mm farther from the wrist (geom.params).
          # TC_MASTER_DROP=0 in the environment reproduces the pre-10/24 runs.
          MASTER_DROP=float(os.environ.get("TC_MASTER_DROP", G.master_drop_from_cad())))   # R2: no spring (no room); the arm's axial compliance does its job + blade search at C


@dataclass
class Err:
    """One draw of the error budget (all zero = nominal)."""
    K: dict = field(default_factory=lambda: {j: 22.0 for j in ARM})         # N.m/rad servo stiffness (P=16)
    fric: dict = field(default_factory=lambda: {j: 0.2 for j in ARM})       # N.m Coulomb
    q_noise_deg: dict = field(default_factory=lambda: {j: 0.0 for j in ARM})  # per-run joint repeatability
    band_frac: dict = field(default_factory=lambda: {j: 0.0 for j in ARM})    # v3 teach: where in the friction band
    dock_shift_mm: tuple = (0.0, 0.0, 0.0)                                   # dock moved after teaching (dock frame)
    dock_yaw_deg: float = 0.0
    rotor_err_deg: float = 0.0                                               # parked rotor angle error
    mu_scale: float = 1.0
    geo: dict = field(default_factory=dict)                                  # print-tolerance overrides
    tool_mass: float = 0.102
    seed: int = 0


# ------------------------------------------------------------------ model
def build_xml(p, err: Err, dock_pos, dock_quat):
    src = MJCF.read_text()
    # drop the stock moving jaw (the master replaces the gripper)
    src = re.sub(r'<!-- Link moving_jaw_so101_v1 -->.*?</body>\s*', "", src, count=1, flags=re.S)
    src = src.replace('meshdir="assets"', f'meshdir="{(MJCF.parent / "assets").as_posix()}"')
    # the master replaces the stock gripper motor + fixed jaw (both meshes live in the gripper body)
    for part in ("sts3215_03a_v1_5", "wrist_roll_follower_so101_v1"):
        src = re.sub(rf'<!-- Part {part} -->\s*<geom[^>]*/>\s*<geom[^>]*/>', "", src, count=1)
    # master housing (cage + gripper servo, measured CAD envelope) as a dock-colliding box
    cage = '<geom name="cage" type="box" size="0.0251 0.017 0.0192" pos="0.0125 0 -0.0252" contype="8" conaffinity="0" rgba="0.2 0.5 0.9 1" mass="0"/>'
    src = src.replace('<!-- Frame gripperframe -->', cage + '<!-- Frame gripperframe -->', 1)
    # arm collision meshes only touch the dock
    src = src.replace('<default class="collision">\n        <geom group="3"/>',
                      '<default class="collision">\n        <geom group="3" contype="8" conaffinity="0"/>')
    assets = []
    mu = err.mu_scale
    master_xml = G.master(p, assets)
    tool_xml = G.tool(p, assets)
    dock_xml = G.dock(p, assets)
    site = f'<site name="face" pos="0 0 {p["Z_TOOL_FACE_LOCK"] * G.M:.6g}" size="0.002"/>'
    src = src.replace('<!-- Frame gripperframe -->', master_xml + site + '\n<!-- Frame gripperframe -->', 1)
    defaults = f"""
  <default>
    <default class="petg"><geom contype="1" conaffinity="0" friction="{0.30 * mu * p["LUBE"]:.3g} 0.002 0.0001" solref="0.002 1" solimp="0.95 0.99 0.0002" condim="3" margin="0" rgba="0.2 0.5 0.9 1" density="800"/></default>
    <default class="steel"><geom contype="1" conaffinity="0" friction="{0.15 * mu:.3g} 0.002 0.0001" solref="0.002 1" solimp="0.95 0.99 0.0002" condim="3" rgba="0.8 0.8 0.85 1" density="0"/></default>
    <default class="steel_tool"><geom contype="2" conaffinity="5" friction="{0.15 * mu:.3g} 0.002 0.0001" solref="0.002 1" solimp="0.95 0.99 0.0002" condim="3" rgba="0.8 0.8 0.85 1" density="0"/></default>
    <default class="pla"><geom contype="2" conaffinity="5" friction="{0.30 * mu * p["LUBE"]:.3g} 0.002 0.0001" solref="0.002 1" solimp="0.95 0.99 0.0002" condim="3" rgba="0.95 0.5 0.1 1" density="0"/></default>
    <default class="pla_dock"><geom contype="4" conaffinity="11" friction="{0.35 * mu:.3g} 0.002 0.0001" solref="0.002 1" solimp="0.95 0.99 0.0002" condim="3" rgba="0.6 0.6 0.6 1"/></default>
  </default>"""
    src = src.replace("<worldbody>", defaults + "\n  <worldbody>", 1)
    # master geoms in the gripper body: give them no mass of their own (the gripper inertial stands in); add master mass
    world_add = f"""
    <geom name="table" type="plane" size="1 1 0.01" pos="0 0 0" contype="4" conaffinity="2" friction="0.4 0.002 0.0001" rgba="0.3 0.35 0.4 1"/>
    <body name="dock" pos="{G._fmt(dock_pos)}" quat="{G._fmt(dock_quat)}">{dock_xml}<site name="dock_origin" size="0.002"/></body>
    {tool_xml}
"""
    src = src.replace("</worldbody>", world_add + "</worldbody>", 1)
    src = src.replace("<asset>", "<asset>\n" + "\n".join(assets), 1)
    # actuators: arm position servos (kp = K), blade servo
    acts = "\n".join(
        f'<position name="a_{j}" joint="{j}" kp="{err.K[j]:.4g}" forcerange="-2.94 2.94" ctrlrange="-6.3 6.3"/>' for j in ARM)
    acts += '\n<position name="a_blade" joint="blade" kp="22" forcerange="-0.25 0.25" ctrlrange="-6.3 6.3"/>'
    src = re.sub(r"<actuator>.*?</actuator>", f"<actuator>{acts}</actuator>", src, flags=re.S)
    opt = ('<option timestep="0.001" integrator="implicitfast" cone="elliptic" impratio="3" iterations="60" ls_iterations="20">'
           '<flag multiccd="enable"/></option>')
    src = src.replace("<default>", opt + "\n  <default>", 1)
    return src


def joint_props(m, err: Err):
    for j in ARM:
        jid = m.joint(j).id
        m.dof_frictionloss[m.jnt_dofadr[jid]] = err.fric[j]
        m.dof_damping[m.jnt_dofadr[jid]] = 1.0
        m.dof_armature[m.jnt_dofadr[jid]] = 0.032
    # master mass: hub/cage/ring/servo ~119 g replaces the stock gripper (87 g + 12 g jaw)
    b = m.body("gripper").id
    drop = float(getattr(err, "master_drop", None) or 0.0)
    # 10/24 master v4: hub 9.4 cm3 (was 5.3) + cage 28.7 (was 25.4) of PLA ~ +9 g; the stack below the hub is 8.5 lower
    m.body_mass[b] = 0.119 + (0.009 if drop else 0.0)
    # NOT moved down with the stack: moving this CoM 8.5 mm along the roll axis (-32 -> -40.5 mm) makes the twin drop
    # the locked tool within 10 ms with no arm motion (zero contacts at t = 10 ms; mj_setConst and dt 0.5 ms do not
    # help; -36 mm is still fine).  Not physics (no torque about the roll axis): a twin defect, open (RealIO agent
    # 2026-10-24).  Physically the CoM is ~8 mm lower; the wrist_flex gravity load differs by ~0.01 N.m.
    m.body_ipos[b] = np.array([0.0, 0.0, -0.032])
    m.body_mass[m.body("tool").id] = err.tool_mass
    m.body_mass[m.body("tool").id] = err.tool_mass


# ------------------------------------------------------------------ kinematics helpers
def quat_from_R(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).ravel())
    return q


def site_pose(m, d, name="face"):
    s = m.site(name).id
    return d.site_xpos[s].copy(), d.site_xmat[s].reshape(3, 3).copy()


def ik(m, d, p_target, R_target, q0, w_rot=0.3, iters=200):
    """Damped least squares on the 5 arm joints: face site at p_target (m), gripper frame -> R_target."""
    q = np.array(q0, float)
    qadr = [m.jnt_qposadr[m.joint(j).id] for j in ARM]
    dadr = [m.jnt_dofadr[m.joint(j).id] for j in ARM]
    sid = m.site("face").id
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    for _ in range(iters):
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        p, R = d.site_xpos[sid], d.site_xmat[sid].reshape(3, 3)
        ep = p_target - p
        Re = R_target @ R.T
        er = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        mujoco.mj_jacSite(m, d, jp, jr, sid)
        J = np.vstack([jp[:, dadr], w_rot * jr[:, dadr]])
        e = np.concatenate([ep, w_rot * er])
        if np.linalg.norm(ep) < 1e-6 and np.linalg.norm(er) < 1e-5:
            break
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e)
        q = q + np.clip(dq, -0.1, 0.1)
    d.qpos[qadr] = q
    mujoco.mj_kinematics(m, d)
    p, R = d.site_xpos[sid].copy(), d.site_xmat[sid].reshape(3, 3).copy()
    return q, float(np.linalg.norm(p_target - p) * 1000)


def solve_seat(m, d, p_face, q0):
    """Tool axis horizontal (gripper z horizontal) at p_face, neck flats level; returns q, R (gripper frame)."""
    qadr = [m.jnt_qposadr[m.joint(j).id] for j in ARM]
    dadr = [m.jnt_dofadr[m.joint(j).id] for j in ARM]
    sid = m.site("face").id
    q = np.array(q0, float)
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    for _ in range(300):
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        p, R = d.site_xpos[sid], d.site_xmat[sid].reshape(3, 3)
        # residuals: position (3), z-axis vertical component (1), gripper y axis vertical component (1) = flats level
        e = np.concatenate([p_face - p, [-R[2, 2] * 0.1, -R[2, 1] * 0.1]])
        mujoco.mj_jacSite(m, d, jp, jr, sid)
        # d(R[2,k])/dq = (w x R[:,k])_z  -> row = (R[:,k] x e_z) . w  ... use numeric instead (robust)
        Jn = np.zeros((5, 5))
        for i in range(5):
            dqv = np.zeros(5); dqv[i] = 1e-6
            d.qpos[qadr] = q + dqv
            mujoco.mj_kinematics(m, d)
            p2, R2 = d.site_xpos[sid].copy(), d.site_xmat[sid].reshape(3, 3).copy()
            e2 = np.concatenate([p_face - p2, [-R2[2, 2] * 0.1, -R2[2, 1] * 0.1]])
            Jn[:, i] = (e2 - e) / 1e-6
        if np.linalg.norm(e) < 1e-8:
            break
        q = q - np.linalg.lstsq(Jn, e, rcond=None)[0].clip(-0.1, 0.1)
    d.qpos[qadr] = q
    mujoco.mj_kinematics(m, d)
    return q, d.site_xmat[sid].reshape(3, 3).copy()


# ------------------------------------------------------------------ scene setup
SEAT_FACE = np.array([0.274, 0.0, 0.100])     # post 1 (centre): coupling face when seated, base frame (m)
# 2026-10-24 (RealIO): with the 10/24 master (+8.5 mm) the routine's away pose F (-60, +45) at 274 mm needs
# shoulder_lift -105 deg (follower range +-99, SAFE_DEG 84) and the standoff E leaves 2.4 deg elbow margin; at 310 mm
# both clear SAFE_DEG by >= 5 deg.  TC_SEAT_R_MM moves the dock radially (the dock height is fixed by the print).
if os.environ.get("TC_SEAT_R_MM"):
    SEAT_FACE = np.array([float(os.environ["TC_SEAT_R_MM"]) / 1000.0, 0.0, 0.100])


def dock_frame(p, q_seat_R, p_face):
    """Dock origin and rotation from the seated gripper frame: x_D = tool direction = -z_gripper."""
    xD = -q_seat_R[:, 2]
    xD[2] = 0.0
    xD /= np.linalg.norm(xD)
    zD = np.array([0.0, 0.0, 1.0])
    yD = np.cross(zD, xD)
    RD = np.column_stack([xD, yD, zD])
    origin = p_face + (p["PLATE_GAP"] + p["TP_T"]) * G.M * xD      # wall face = plate back + gap
    return origin, RD


def make(design="v4", err: Err | None = None):
    err = err or Err()
    p = G.params(**(V4 if design.startswith("v4") else {}))
    if design == "v4nolube":
        p["LUBE"] = 1.0
    p.update(err.geo)
    p["TOOL_MASS"] = err.tool_mass
    # pass 1: kinematics-only model to place the dock (taught = nominal geometry, nominal dock)
    xml0 = build_xml(p, err, (1, 1, -1), (1, 0, 0, 0))
    m0 = mujoco.MjModel.from_xml_string(xml0)
    d0 = mujoco.MjData(m0)
    q_seat, R_seat = solve_seat(m0, d0, SEAT_FACE, np.array([0, 0.06, 1.19, -1.26, 0.05]))
    origin, RD = dock_frame(p, R_seat, SEAT_FACE)
    # true dock = taught dock moved by the error (dock frame)
    sh = np.array(err.dock_shift_mm) * G.M
    yaw = err.dock_yaw_deg * DEG
    Ryaw = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    origin_true = origin + RD @ sh
    RD_true = Ryaw @ RD
    xml = build_xml(p, err, origin_true, quat_from_R(RD_true))
    m = mujoco.MjModel.from_xml_string(xml)
    err.master_drop = float(p.get("MASTER_DROP", 0.0))
    joint_props(m, err)
    d = mujoco.MjData(m)
    return dict(m=m, d=d, p=p, err=err, design=design, q_seat=q_seat, R_seat=R_seat, dock_origin=origin, RD=RD,
                dock_origin_true=origin_true, RD_true=RD_true, xml=xml, m0=m0, d0=d0)


def place_tool_in_dock(S):
    """Tool seated in the (true) dock, its rotor parked (+ the parked-angle error)."""
    m, d, p = S["m"], S["d"], S["p"]
    RD, o = S["RD_true"], S["dock_origin_true"]
    # tool frame: z_T = -x_D (tool points into the wall), x_T / y_T level as in the seated gripper frame
    Rg = S["RD_true"] @ S["RD"].T @ S["R_seat"]          # seated gripper frame, moved with the dock
    face = o - (p["PLATE_GAP"] + p["TP_T"]) * G.M * RD[:, 0]
    # neck bottom on the slot floor: drop by the slot clearance in z (gravity will settle it)
    face = face + np.array([0, 0, -0.0002])
    jid = m.joint("tool_free").id
    a = m.jnt_qposadr[jid]
    d.qpos[a:a + 3] = face
    d.qpos[a + 3:a + 7] = quat_from_R(Rg)
    d.qpos[m.jnt_qposadr[m.joint("rot_z").id]] = S["err"].rotor_err_deg * DEG


# ------------------------------------------------------------------ trajectory + controller
class Runner:
    def __init__(self, S, record=None):
        self.S, self.m, self.d = S, S["m"], S["d"]
        m = self.m
        self.qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in ARM])
        self.dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in ARM])
        self.act = np.array([m.actuator(f"a_{j}").id for j in ARM])
        self.ab = m.actuator("a_blade").id
        self.blade_q = m.jnt_qposadr[m.joint("blade").id]
        self.I = np.zeros(5)              # outer-loop correction (v4)
        self.integrate = False
        self.blade_off = 0.0
        self.magnet = S["p"].get("MAGNET_N", 0.0)
        # clamped outer loop during contact: cancels the friction lag (~band) but can add at most K x clamp of
        # torque, so the insertion stays compliant (v4)
        # per-robot clamp from the one-time friction-band calibration (measured band, +-15 % measurement error)
        err = S["err"]
        rng = np.random.default_rng(err.seed + 7)
        band = np.array([err.fric[j] / err.K[j] for j in ARM]) * rng.uniform(0.85, 1.15, 5)
        self.I_clamp = 1.3 * band + 0.3 * DEG
        self.I_clamp[4] = 0.0
        self.offset = np.zeros(5)         # v3 taught-command offset (sag with tool + band) + repeatability
        self.log = []
        self.record = record
        self.t_rec = 0.0
        self.events = {}
        self.roll_tq = []

    def q(self):
        return self.d.qpos[self.qadr].copy()

    def _magnet(self):
        """Dock magnets (v4): 2x N52 6x3 under the tail rest pull a steel disc in the tool body.  F = F0 / (1 + g/1 mm)^2
        along -z_dock, g = gap; nothing beyond 6 mm or 4 mm sideways."""
        m, d, S, p = self.m, self.d, self.S, self.S["p"]
        tb = m.body("tool").id
        Rt, pt = d.xmat[tb].reshape(3, 3), d.xpos[tb]
        disc = pt + Rt @ (np.array([-p["NECK_SQ"] / 2, 0.0, -11.0]) * G.M)          # R3: neck magnet (-x face, tool z -11)
        o, RD = S["dock_origin_true"], S["RD_true"]
        loc = RD.T @ (disc - o) * 1000
        mag = np.array([4.0, 0.0, -p["NECK_SQ"] / 2])                                # R3: dock magnet in the land (1 mm deep)
        g = loc[2] - mag[2]
        d.xfrc_applied[tb] = 0.0
        if -0.5 < g < 6.0 and np.hypot(loc[0] - mag[0], loc[1] - mag[1]) < 4.0:
            # 2026-10-05 arm-redteam: MAGNET_G0 sets the fall-off length.  Default 1.0 = the original 1/(1+g)^2.
            # A charge-model calc of the real N52 6x3 PAIR (1 mm PLA skin) gives 6.3 N seated and 3.6 N at +1 mm,
            # i.e. F0 ~6.3, G0 ~3 mm (toolchanger/analysis/magnet_check.py).
            g0 = float(p.get("MAGNET_G0", 1.0))
            f = -self.magnet * (g0 / (g0 + max(g, 0.0))) ** 2 * RD[:, 2]
            d.xfrc_applied[tb, :3] = f
            d.xfrc_applied[tb, 3:] = np.cross(disc - d.xipos[tb], f)

    def step_to(self, q_goal, blade_goal, T, integrate=False, ki=6.0, roll_limit=None, hold=0.0):
        """Linear joint-space move to q_goal over T s (+ hold s), 50 Hz command updates."""
        m, d = self.m, self.d
        if isinstance(q_goal, list):                 # Cartesian-straight path: list of IK solutions
            path = [self.q_cmd] + q_goal
            q_goal = path[-1]
        else:
            path = None
        q_start = self.q_cmd if hasattr(self, "q_cmd") else self.q()
        b_start = self.b_cmd if hasattr(self, "b_cmd") else 0.0
        n = int((T + hold) / m.opt.timestep)
        sub = int(0.02 / m.opt.timestep)
        if roll_limit is not None:
            m.actuator_forcerange[self.act[4]] = [-roll_limit, roll_limit]
        for i in range(n):
            t = i * m.opt.timestep
            s = min(t / T, 1.0) if T > 0 else 1.0
            s = s * s * (3 - 2 * s)
            if path is None:
                qc = q_start + (q_goal - q_start) * s
            else:
                u = s * (len(path) - 1)
                k = min(int(u), len(path) - 2)
                qc = path[k] + (path[k + 1] - path[k]) * (u - k)
            bc = b_start + (blade_goal + self.blade_off - b_start) * s
            if i % sub == 0:
                qm = self.q()
                if integrate:
                    self.I += ki * (qc - qm) * 0.02
                    lim = self.I_clamp if integrate == "clamped" else 6 * DEG
                    self.I = np.clip(self.I, -lim, lim)
                d.ctrl[self.act] = qc + self.offset + self.I
                d.ctrl[self.ab] = bc
            if self.magnet > 0:
                self._magnet()
            mujoco.mj_step(m, d)
            if i % 10 == 0:
                self.roll_tq.append((d.time, float(d.actuator_force[self.act[4]]), float(d.qpos[self.qadr[4]])))
            if self.record is not None and d.time >= self.t_rec:
                self.record(self)
                self.t_rec += 1 / 30
            if not np.isfinite(d.qpos).all():
                raise FloatingPointError("sim diverged")
        self.q_cmd, self.b_cmd = q_goal.copy(), blade_goal + self.blade_off

    # --- measurements
    def rel_tool(self):
        """Tool face pose in the master (gripper) frame: dz (mm, + = toward master), lateral (mm), tilt (deg), roll (deg)."""
        m, d, p = self.m, self.d, self.S["p"]
        g = m.body("gripper").id
        Rg, pg = d.xmat[g].reshape(3, 3), d.xpos[g]
        tb = m.body("tool").id
        Rt, pt = d.xmat[tb].reshape(3, 3), d.xpos[tb]
        loc = Rg.T @ (pt - pg) * 1000
        Rrel = Rg.T @ Rt
        tilt = math.degrees(math.acos(max(-1, min(1, Rrel[2, 2]))))
        roll = math.degrees(math.atan2(Rrel[1, 0], Rrel[0, 0]))
        return dict(dz=loc[2] - p["Z_TOOL_FACE_LOCK"], lat=float(np.hypot(loc[0], loc[1])), tilt=tilt, roll=roll)

    def tool_in_dock(self):
        m, d, S = self.m, self.d, self.S
        tb = m.body("tool").id
        o, RD = S["dock_origin_true"], S["RD_true"]
        loc = RD.T @ (d.xpos[tb] - o) * 1000
        Rt = RD.T @ d.xmat[tb].reshape(3, 3)
        ax = -Rt[:, 2]                          # tool direction in the dock frame (should be +x)
        tilt = math.degrees(math.acos(max(-1, min(1, ax[0]))))
        return dict(x=loc[0], y=loc[1], z=loc[2], tilt=tilt)

    def blade_axial_mm(self):
        m, d = self.m, self.d
        try:
            return float(d.qpos[m.jnt_qposadr[m.joint("blade_ax").id]] * 1000)
        except KeyError:
            return 0.0

    def blade_engaged(self):
        m, d = self.m, self.d
        b = d.qpos[self.blade_q]
        # rotor angle in the master frame = tool roll rel. master + rotor hinge
        rot = d.qpos[m.jnt_qposadr[m.joint("rot_z").id]]
        rel = math.degrees(b - (math.radians(self.rel_tool()["roll"]) + rot))
        rel = (rel + 90) % 180 - 90
        return abs(rel)

    def joint_load(self):
        return np.abs(self.d.actuator_force[self.act]) / 2.94


# ------------------------------------------------------------------ the swap cycle
def gravity_offset(S, with_tool=True):
    """Static sag at the seat pose: dq = K^-1 tau_gravity (incl. the tool when mounted)."""
    m0, d0 = S["m0"], S["d0"]
    qadr = [m0.jnt_qposadr[m0.joint(j).id] for j in ARM]
    dadr = [m0.jnt_dofadr[m0.joint(j).id] for j in ARM]
    d0.qpos[:] = m0.qpos0
    d0.qpos[qadr] = S["q_seat"]
    mujoco.mj_forward(m0, d0)
    tau = d0.qfrc_bias[dadr].copy()
    # m0 has the stock gripper mass (0.087) and no master mass change: correct to the master + tool
    g = m0.body("gripper").id
    jp = np.zeros((3, m0.nv))
    extra = [(0.119 - m0.body_mass[g], np.array([0, 0, -0.032]))]
    if with_tool:
        extra.append((S["err"].tool_mass, np.array([0, 0, (S["p"]["Z_TOOL_FACE_LOCK"] + S["p"]["TOOL_COM"]) * G.M])))
    for mass, loc in extra:
        pt = d0.xpos[g] + d0.xmat[g].reshape(3, 3) @ loc
        mujoco.mj_jac(m0, d0, jp, None, pt, g)
        tau += jp[:, dadr].T @ np.array([0, 0, 9.81 * mass])
    K = np.array([S["err"].K[j] for j in ARM])
    return tau / K


def cycle(S, record=None, verbose=False):
    """Pick-up from the dock, lift, carry away and back, drop-off.  Returns a result dict."""
    m, d, p, err, design = S["m"], S["d"], S["p"], S["err"], S["design"]
    mujoco.mj_resetData(m, d)
    place_tool_in_dock(S)
    R = Runner(S, record)
    m0, d0 = S["m0"], S["d0"]
    RD, o = S["RD"], S["dock_origin"]           # TAUGHT dock frame (commands are computed from it)
    face_seat = o - (p["PLATE_GAP"] + p["TP_T"]) * G.M * RD[:, 0]
    Rg = S["R_seat"]
    lock_roll = S["q_seat"][4]

    def Rroll(deg):
        a = deg * DEG
        return Rg @ np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])

    def wp(dx, dz, roll_deg=0.0, q0=None):
        """IK by continuation from the seat (straight-line steps) so far waypoints stay on the same branch."""
        q = S["q_seat"].copy()
        n = int(max(abs(dx), abs(dz)) / 5) + 1
        for i in range(1, n + 1):
            pt = face_seat + RD @ np.array([dx * i / n, 0, dz * i / n]) * G.M
            q, e = ik(m0, d0, pt, Rroll(roll_deg * i / n), q)
        if e > 0.05:
            raise RuntimeError(f"IK failed for waypoint {dx, dz, roll_deg}: {e:.2f} mm")
        return q

    v4 = design.startswith("v4")
    entry = p["Z_TOOL_FACE_ENTRY"] - p["Z_TOOL_FACE_LOCK"]      # -0.3 mm: master 0.3 further back at entry
    ax_over = V4_RUN["axial_over"] if v4 else 1.5
    roll_over = V4_RUN["roll_over"] if v4 else 0.0
    # q targets (encoder space, from the taught dock); straight Cartesian lines, IK every ~1 mm / 1.5 deg
    cache = {}

    def line(a, b):
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]), abs(b[2] - a[2]) / 1.5)) + 1
        out = []
        for i in range(1, n + 1):
            key = tuple(round(a[k] + (b[k] - a[k]) * i / n, 4) for k in range(3))
            if key not in cache:
                cache[key] = wp(*key)
            out.append(cache[key])
        return out
    dn = V4_RUN["down_over"] if v4 else 0.0
    ci = "clamped" if v4 else False
    roll_lim = V4_RUN["roll_limit"] * 2.94 if v4 else 0.55 * 2.94
    P_F, P_E, P_C = (-40, 80, 45), (-30, 0, 45), (entry + ax_over, 0, 45)
    P_L, P_Lr, P_A = (entry + ax_over, 0, -roll_over), (entry + ax_over, 0, 0), (0, 35, 0)
    P_Lc = (0, 0, 0) if v4 else P_Lr          # v4: back off to the centre of the axial gap before lifting
    P_Fl = (-40, 80, 0)
    P_Cd, P_C0, P_Ed = (0, -dn, 0), (0, 0, 0), (-30, 0, 45)
    P_Dd = (0, 0 if v4 else -dn, 45 + roll_over)
    qF = wp(*P_F)
    qE = wp(*P_E)
    # sign of the lock twist: the master turns from +45 (entry) to 0 (lock) relative to the tool
    rng = np.random.default_rng(err.seed)
    rep = np.array([err.q_noise_deg[j] for j in ARM]) * DEG
    fr = np.array([err.fric[j] / err.K[j] for j in ARM])
    if v4:
        R.offset = rep.copy()
    else:
        R.offset = gravity_offset(S, with_tool=True) + rep + fr * np.array([err.band_frac[j] for j in ARM])
    # start: tool LOCKED on the master, neck 35 mm above the slot (pose A); settle
    qA0 = wp(*P_A)
    d.qpos[R.qadr] = qA0
    d.ctrl[R.act] = qA0 + R.offset
    mujoco.mj_kinematics(m, d)
    g = m.body("gripper").id
    Rg_now, pg_now = d.xmat[g].reshape(3, 3), d.xpos[g]
    ja = m.jnt_qposadr[m.joint("tool_free").id]
    d.qpos[ja:ja + 3] = pg_now + Rg_now @ np.array([0, 0, p["Z_TOOL_FACE_LOCK"] * G.M])
    d.qpos[ja + 3:ja + 7] = quat_from_R(Rg_now)
    for i in range(3):
        d.qpos[m.jnt_qposadr[m.joint(f"pl{i}").id]] = (p["Z_TOOL_FACE_LOCK"] - p["Z_TOOL_FACE_ENTRY"]) * G.M
    d.qpos[m.jnt_qposadr[m.joint("rot_z").id]] = 0.0
    R.q_cmd = qA0.copy()
    R.b_cmd = 0.0
    blade0 = -45 * DEG                         # blade aligned with the parked rotor at entry roll (+45)
    res = dict(design=design, seed=err.seed)
    m.actuator_forcerange[R.act[4]] = [-(0.25 if v4 else 0.10) * 2.94, (0.25 if v4 else 0.10) * 2.94]
    try:
        R.step_to(line(P_A, P_A), 0.0, 0.0, hold=0.5)
        res["start"] = R.rel_tool()
        R.step_to(line(P_A, P_A), 0.0, 0.0, integrate=v4, hold=0.6 if v4 else 0.2)
        # ---- DROP-OFF
        R.I = np.clip(R.I, -R.I_clamp, R.I_clamp) if v4 else R.I
        R.step_to(line(P_A, P_Cd), 0.0, 1.8, integrate=ci)
        R.step_to(line(P_Cd, P_Cd), 0.0, 0.0, hold=0.3, integrate=ci)
        res["drop_seated"] = R.tool_in_dock()
        if v4:   # release the press-down before unlocking (stored arm deflection would side-load the spigot)
            R.step_to(line(P_Cd, P_C0), 0.0, 0.4, integrate=ci)
            R.step_to(line(P_C0, P_C0), 0.0, 0.0, hold=0.3, integrate=ci)
        R.step_to(line(P_C0 if v4 else P_Cd, P_Dd), -45 * DEG, 1.5, roll_limit=roll_lim, integrate=ci)
        R.step_to(line(P_Dd, P_Dd), -45 * DEG, 0.0, hold=0.3, roll_limit=roll_lim, integrate=ci)
        res["unlocked"] = R.rel_tool()
        R.step_to(line(P_Dd, P_Ed), -45 * DEG, 2.0 if v4 else 1.5, roll_limit=0.55 * 2.94, integrate=ci)
        R.step_to(line(P_Ed, P_F), -45 * DEG, 1.0)
        R.step_to(qF, -45 * DEG, 0.0, hold=0.3)
        res["parked"] = pk = R.tool_in_dock()
        x_seat = -(p["PLATE_GAP"] + p["TP_T"])
        drop_ok = abs(pk["x"] - x_seat) < 5.0 and abs(pk["y"]) < 2.0 and abs(pk["z"]) < 2.5 and pk["tilt"] < 6.0
        res["drop_ok"] = bool(drop_ok)
        if not drop_ok:
            res["ok"] = False
            res["fail"] = "drop: tool not parked in the slot"
            res["roll_trace"] = R.roll_tq[::5]
            return res
        # parked-rotor disturbance (jaw springback / a bump while parked)
        d.qpos[m.jnt_qposadr[m.joint("rot_z").id]] += err.rotor_err_deg * DEG
        # ---- PICK-UP
        R.I[:] = 0.0
        R.step_to(line(P_F, P_E), blade0, 1.5, integrate=v4)
        if v4:   # r3: settle until the encoder servo has converged (|err| < 0.15 deg) or 2 s
            for _ in range(10):
                R.step_to(qE, blade0, 0.0, integrate=True, hold=0.2)
                if np.max(np.abs(R.q() - qE)[:4]) < 0.15 * DEG:
                    break
        else:
            R.step_to(qE, blade0, 0.0, hold=0.2)
        res["standoff_err_deg"] = ((R.q() - qE) / DEG).tolist()
        R.I = np.clip(R.I, -R.I_clamp, R.I_clamp)
        R.step_to(line(P_E, P_C), blade0, 2.0, roll_limit=0.55 * 2.94, integrate=ci)   # compliant insertion
        R.step_to(line(P_C, P_C), blade0, 0.0, hold=0.3, integrate=ci)
        res["insert"] = R.rel_tool()
        if v4 and p["BLADE_SPRING"] == 0:
            # R2 blade search AT THE INSERTION POSE: sweep toward the rotor's open stop until the blade stalls (engaged,
            # rotor on its stop).  If the blade sat on the rotor top, the arm (still pressing) pushes the last ~4 mm in
            # when it drops into the slot.  Same logic as swap_routine._blade_search.
            b_prev, stalled, b = math.degrees(R.d.qpos[R.blade_q]), 0, -45.0
            hit = None
            while b > -45.0 - 230:
                b -= 10
                R.step_to(line(P_C, P_C), b * DEG, 0.15, integrate=ci)
                bq = math.degrees(R.d.qpos[R.blade_q])
                stalled = stalled + 1 if (abs(bq - b_prev) < 1.0 and abs(b - bq) > 5.0) else 0
                b_prev = bq
                if stalled >= 2:
                    hit = bq
                    break
            if hit is not None:
                # rotor open stop (tool frame) seen from the master at entry roll: stop - 45
                R.blade_off = (hit - (p["ROTOR_RANGE"][0] - 45.0)) * DEG    # so goal -45 deg puts the rotor at 0
                R.step_to(line(P_C, P_C), blade0, 0.4, integrate=ci)
            res["search"] = [None if hit is None else round(hit, 1), round(R.rel_tool()["dz"], 2)]
            res["insert_after_search"] = R.rel_tool()
        t0 = d.time
        R.step_to(line(P_C, P_L), 0.0, 1.5, roll_limit=roll_lim, integrate=ci)  # twist to lock (+ overtravel)
        R.step_to(line(P_L, P_L), 0.0, 0.0, hold=0.3, roll_limit=roll_lim, integrate=ci)
        res["roll_peak_Nm"] = max(abs(t) for tt, t, _ in R.roll_tq if tt >= t0)
        R.step_to(line(P_L, P_Lr), 0.0, 0.3, roll_limit=0.25 * 2.94)        # relax to the lock angle
        R.step_to(line(P_Lr, P_Lr), 0.0, 0.0, hold=0.2, roll_limit=0.25 * 2.94)
        res["locked"] = R.rel_tool()
        if v4:
            R.step_to(line(P_Lr, P_Lc), 0.0, 0.4, roll_limit=0.55 * 2.94, integrate=True)
            if v4 and p["BLADE_SPRING"] > 0:
                # blade search: sweep TOWARD the rotor's open stop (the slot repeats every 180 deg, so -100 deg covers
                # every parked angle): the rotor is pushed onto its stop and the spring drops the blade in
                if not (R.blade_axial_mm() < 1.0 and R.blade_engaged() < 8):
                    b, hit = 0, None
                    while b > -190:
                        b -= 10
                        R.step_to(line(P_Lc, P_Lc), b * DEG, 0.15, roll_limit=0.25 * 2.94)
                        if R.blade_axial_mm() < 1.0:
                            hit = b
                            break
                    if hit is not None:          # keep pushing: the rotor goes onto its open stop, the blade stalls
                        R.step_to(line(P_Lc, P_Lc), (hit - 20) * DEG, 0.4, roll_limit=0.25 * 2.94)
                        b_stop = math.degrees(R.d.qpos[R.blade_q])
                        R.blade_off = (b_stop - p["ROTOR_RANGE"][0]) * DEG     # blade angle that puts the rotor at 0
                        R.step_to(line(P_Lc, P_Lc), 0.0, 0.5, roll_limit=0.25 * 2.94)
                    res["search"] = [hit, round(R.blade_axial_mm(), 2), round(R.blade_engaged(), 1)]

        res["blade_err_deg"] = R.blade_engaged()
        res["blade_ax_mm"] = R.blade_axial_mm()
        if v4:   # take the tool's weight first (slow 3 mm, full outer loop), then lift
            P_A3 = (0, 3, 0)
            R.step_to(line(P_Lc, P_A3), 0.0, 1.0, roll_limit=0.55 * 2.94, integrate=True)
            R.step_to(line(P_A3, P_A3), 0.0, 0.0, hold=0.4, roll_limit=0.55 * 2.94, integrate=True)
            R.step_to(line(P_A3, P_A), 0.0, 1.5, roll_limit=0.55 * 2.94, integrate=True)
        else:
            R.step_to(line(P_Lc, P_A), 0.0, 1.5, roll_limit=0.55 * 2.94)          # lift out of the slot
        R.step_to(line(P_A, P_A), 0.0, 0.0, hold=0.3)
        res["lifted"] = R.rel_tool()
        res["lifted_dock"] = R.tool_in_dock()
        pick_ok = (abs(res["lifted"]["dz"]) < 0.6 and res["lifted"]["lat"] < 0.8 and res["lifted"]["tilt"] < 2.0
                   and abs(res["lifted"]["roll"]) < 3.0 and res["lifted_dock"]["z"] > 25 and res["blade_err_deg"] < 10
                   and res["blade_ax_mm"] < 1.5)
        res["pick_ok"] = bool(pick_ok)
        res["ok"] = bool(pick_ok)
        if not pick_ok:
            res["fail"] = classify_pick(res)
        res["load_max"] = R.joint_load().tolist()
    except FloatingPointError:
        res["ok"] = False
        res["fail"] = "sim diverged"
    res["roll_trace"] = R.roll_tq[::5]
    return res


def classify_pick(r):
    li, ld = r["lifted"], r["lifted_dock"]
    if ld["z"] < 25:
        if abs(r["locked"]["roll"]) > 15:
            return "pick: pins never left the windows (axial short / jam at the lip)"
        return "pick: tool stayed in the dock (not locked)"
    if r["blade_err_deg"] >= 10:
        return "pick: drive blade not in the rotor slot"
    if abs(li["roll"]) >= 3:
        return "pick: pin not in its V (resting on a crest/flank)"
    return "pick: tool attached but mis-seated (tilt/offset)"


V4_RUN = dict(axial_over=2.0, roll_over=4.0, down_over=1.0, roll_limit=0.85)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["nominal", "xml"])
    ap.add_argument("--design", default="v4")
    a = ap.parse_args()
    S = make(a.design)
    if a.mode == "xml":
        (HERE / "out").mkdir(exist_ok=True)
        (HERE / "out" / "scene.xml").write_text(S["xml"])
        print("ngeom", S["m"].ngeom, "nbody", S["m"].nbody)
        return
    import time
    t = time.time()
    r = cycle(S)
    r.pop("roll_trace", None)
    print(json.dumps(r, indent=1, default=float))
    print("wall", round(time.time() - t, 1), "s  sim", round(S["d"].time, 1), "s")


if __name__ == "__main__":
    main()
