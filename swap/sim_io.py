# SPDX-License-Identifier: GPL-3.0-or-later
"""ArmIO for the MuJoCo twin (toolchanger/sim/tcsim.py): runs swap_routine.Swapper against the physics sim.

    python sim_io.py --seeds 1000 1010         # drop + pick per seed, error budget from sim/mc.py
    python sim_io.py --n 100 --seed0 5000      # Monte Carlo of the ROUTINE (with its checks + retry)
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "sim"))
import tcsim as T  # noqa: E402
import mc  # noqa: E402
import swap_routine as SR  # noqa: E402

DEG = math.pi / 180
sys.path.insert(0, str(HERE.parents[1] / "dashboard"))
from splat import fk as _fk  # noqa: E402
_ZOFF = np.array([_fk.ZERO_OFFSET_RAD.get(j, 0.0) for j in _fk.ARM_JOINTS])
_SAFE = np.array([103.0, 84.0, 90.0, 84.0, 165.0])         # splat/sweep.SAFE_DEG (sweep imports nothing heavy, but keep sim_io light)


class SimIO:
    def __init__(self, S):
        self.S, self.m, self.d = S, S["m"], S["d"]
        self.R = T.Runner(S)            # reuse: joint addresses, magnets, measurements
        self.m0, self.d0 = S["m0"], S["d0"]

    def read_q(self):
        return self.d.qpos[self.R.qadr] / DEG

    def read_blade(self):
        return float(self.d.qpos[self.R.blade_q] / DEG)

    def read_load(self):
        return self.d.actuator_force[self.R.act] / 2.94

    def write(self, q_goal, blade_goal):
        # 10/24: margin of the command to the real follower's SAFE_DEG in lerobot degrees (the MJCF has no joint limits)
        ql = (np.asarray(q_goal) * DEG - _ZOFF) / DEG
        self.safe_margin = min(getattr(self, "safe_margin", 1e9), float(np.min(_SAFE - np.abs(ql))))
        self.d.ctrl[self.R.act] = np.asarray(q_goal) * DEG
        self.d.ctrl[self.R.ab] = blade_goal * DEG

    def set_roll_torque(self, frac):
        self.m.actuator_forcerange[self.R.act[4]] = [-frac * 2.94, frac * 2.94]

    def read_temp(self):
        return np.full(6, 35.0)          # the twin has no thermal model: exercises the check path only

    def set_gripper_torque(self, frac):
        self.m.actuator_forcerange[self.R.ab] = [-frac * 2.94, frac * 2.94]

    def tick(self, dt):
        for _ in range(int(round(dt / self.m.opt.timestep))):
            if self.R.magnet > 0:
                self.R._magnet()
            mujoco.mj_step(self.m, self.d)
        if not np.isfinite(self.d.qpos).all():
            raise FloatingPointError("sim diverged")

    def face_offset_q(self, q_seat_deg, dx, dz, droll, q0_deg):
        """FK of the TAUGHT seat pose (encoder angles) defines the dock frame; local IK for the offset."""
        m0, d0 = self.m0, self.d0
        key = tuple(np.round(q_seat_deg, 4))
        if getattr(self, "_seat_key", None) != key:
            qadr = [m0.jnt_qposadr[m0.joint(j).id] for j in T.ARM]
            d0.qpos[:] = m0.qpos0
            d0.qpos[qadr] = np.asarray(q_seat_deg) * DEG
            mujoco.mj_kinematics(m0, d0)
            p, Rg = T.site_pose(m0, d0)
            xD = -Rg[:, 2].copy(); xD[2] = 0; xD /= np.linalg.norm(xD)
            zD = np.array([0, 0, 1.0]); yD = np.cross(zD, xD)
            self._seat = (p, Rg, np.column_stack([xD, yD, zD]))
            self._seat_key = key
        p, Rg, RD = self._seat
        a = droll * DEG
        Rr = Rg @ np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        q, e = T.ik(m0, d0, p + RD @ np.array([dx, 0, dz]) * 1e-3, Rr, np.asarray(q0_deg) * DEG)
        return q / DEG


def start_locked_at_A(S, io: SimIO):
    """Tool locked on the master, neck 35 mm above the slot (same start as tcsim.cycle)."""
    m, d, p = S["m"], S["d"], S["p"]
    mujoco.mj_resetData(m, d)
    T.place_tool_in_dock(S)
    q_seat = S["q_seat"] / DEG
    qA = io.face_offset_q(q_seat, 0, 35, 0, q_seat)
    d.qpos[io.R.qadr] = qA * DEG
    d.ctrl[io.R.act] = qA * DEG
    mujoco.mj_kinematics(m, d)
    g = m.body("gripper").id
    Rg, pg = d.xmat[g].reshape(3, 3), d.xpos[g]
    ja = m.jnt_qposadr[m.joint("tool_free").id]
    d.qpos[ja:ja + 3] = pg + Rg @ np.array([0, 0, p["Z_TOOL_FACE_LOCK"] * 1e-3])
    d.qpos[ja + 3:ja + 7] = T.quat_from_R(Rg)
    for i in range(3):
        d.qpos[m.jnt_qposadr[m.joint(f"pl{i}").id]] = (p["Z_TOOL_FACE_LOCK"] - p["Z_TOOL_FACE_ENTRY"]) * 1e-3
    io.tick(0.5)


def run_seed(seed, design="v4", calibrate=True, quiet=True, hz=50.0):
    err = mc.sample(seed, design)
    S = T.make(design, err)
    io = SimIO(S)
    start_locked_at_A(S, io)
    q_seat = S["q_seat"] / DEG                  # taught = nominal; the real dock is moved by err.dock_shift
    dock = SR.DockTeach("post1", q_seat=list(q_seat), blade_park=0.0)
    logs = []
    fric = SR.FrictionCal()
    if calibrate:   # friction calibration at a free pose above the dock (tool on: the arm carries it anyway)
        fric = SR.calibrate_friction(io, io.face_offset_q(q_seat, -20, 60, 0, io.read_q()), hz=hz)
    sw = SR.Swapper(io, cfg=SR.Config(hz=hz), fric=fric, log=(lambda s: logs.append(s)) if quiet else print)
    t0 = time.time()
    out = {"seed": seed, "design": design, "hz": hz, "band_measured": fric.band_deg,
           "band_true": [round(err.fric[j] / err.K[j] / DEG, 3) for j in T.ARM]}
    try:
        out["drop"] = sw.run(sw.drop, dock)
        out["parked"] = io.R.tool_in_dock()
        # parked-rotor disturbance, then a no-tool load reference at the 3-mm-lifted pose (verify step)
        d, m = S["d"], S["m"]
        d.qpos[m.jnt_qposadr[m.joint("rot_z").id]] += err.rotor_err_deg * DEG
        out["pick"] = sw.run(sw.pick, dock)
        rel, ind = io.R.rel_tool(), io.R.tool_in_dock()
        out["truth"] = {"rel": rel, "dock": ind, "blade_err": io.R.blade_engaged(), "blade_ax": io.R.blade_axial_mm()}
        truth_ok = abs(rel["dz"]) < 0.6 and rel["lat"] < 0.8 and rel["tilt"] < 2 and abs(rel["roll"]) < 3 and ind["z"] > 25
        out["truth_ok"] = bool(truth_ok)
        out["routine_says_ok"] = bool(out["pick"]["ok"])
        out["false_positive"] = bool(out["pick"]["ok"] and not truth_ok)
    except FloatingPointError as e:
        out["error"] = repr(e)
    out["safe_margin_deg"] = round(float(getattr(io, "safe_margin", float("nan"))), 2)
    out["seat_r_mm"] = round(float(T.SEAT_FACE[0]) * 1000, 1)
    out["master_drop_mm"] = float(S["p"].get("MASTER_DROP", 0.0))
    out["wall_s"] = round(time.time() - t0, 1)
    out["log"] = logs[-12:]
    return out


def _run_hz(args):
    seed, hz = args
    return run_seed(seed, hz=hz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*")
    ap.add_argument("--n", type=int, default=0)
    ap.add_argument("--seed0", type=int, default=5000)
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--out", default=str(HERE.parent / "sim" / "out" / "routine_v4.jsonl"))
    ap.add_argument("--hz", type=float, default=50.0, help="control rate (the real arm runs 25 Hz)")
    a = ap.parse_args()
    seeds = a.seeds or list(range(a.seed0, a.seed0 + a.n))
    if Path(a.out).exists():               # resumable: skip seeds already in the output
        done = {json.loads(l)["seed"] for l in Path(a.out).read_text().splitlines() if l.strip()}
        seeds = [x for x in seeds if x not in done]
    if a.workers > 1:
        from multiprocessing import Pool
        with Pool(a.workers) as pool, open(a.out, "a") as f:
            for r in pool.imap_unordered(_run_hz, [(x, a.hz) for x in seeds]):
                f.write(json.dumps(r, default=float) + "\n"); f.flush()
                print(r["seed"], r.get("truth_ok"), r.get("routine_says_ok"), r.get("wall_s"), flush=True)
    else:
        with open(a.out, "a") as f:
            for s in seeds:
                r = run_seed(s, quiet=False, hz=a.hz)
                f.write(json.dumps(r, default=float) + "\n"); f.flush()
                print(json.dumps({k: v for k, v in r.items() if k != "log"}, default=float, indent=1))


if __name__ == "__main__":
    main()
