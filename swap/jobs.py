# SPDX-License-Identifier: GPL-3.0-or-later
"""Changer sessions: everything that moves runs as a job inside sweep.run() with real_io.TcSweep (see real_io.py).

    run_session(robot, job, cfg, ask=...)  -> (exit code, TcSweep)

Jobs (each returns a callable job(tc)):
    job_twist     roll-only lock / unlock twists at a Torque_Limit ladder, the arm held at SERVICE (C2 / hand check)
    job_friction  calibrate_friction (DESIGN_v4 S2) at a free pose, 2 runs, larger band per joint kept
    job_teach     body joints LIMP, owner seats the tool in the dock by hand, 3 recordings (D3)
    job_swap      approach -> drop / pick / cycles through RealSwapper (the sim-validated routine) -> park
plan_checks()  the offline audit every motion command runs first (and `run_swap.py check` prints).
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

import real_io as R
import swap_routine as SR
from real_io import J, REAL_HZ, sweep

LOGS = R.HERE / "logs"
#: the joint-space approach / park legs end / start here (dock frame, above F); F_HIGH -> F is a Cartesian descent
F_HIGH = (R.F_POSE[0], R.F_POSE[1] + 55.0, 0.0)
TRANSIT = np.array([sweep.TRANSIT[j] for j in J])


class NullLink:
    """No dashboard server: reports go nowhere (sweep.FakeLink would keep every payload in RAM)."""
    token = ""

    def get_json(self, path):
        return {}

    def report(self, payload):
        return {"ok": True}


def run_session(robot: Any, job: Callable, cfg: R.RealConfig, *, ask: Optional[Callable[[str], str]] = None,
                clock: Optional[Callable[[], float]] = None, sleep: Optional[Callable[[float], None]] = None,
                stop: Optional[R.StopFlag] = None, max_runtime_s: float = 900.0, log_path: Optional[str] = None,
                home: Optional[Dict[str, float]] = None, on_tc: Optional[Callable[[R.TcSweep], None]] = None
                ) -> Tuple[int, Optional[R.TcSweep]]:
    """sweep.run with TcSweep: the job runs as the 'plan', sweep.run's finally parks (or holds, see TcSweep.park)."""
    tap = R._ObsTap(robot)
    own_stop = stop is None
    stop = stop or R.StopFlag().install()
    holder: Dict[str, R.TcSweep] = {}

    def factory(*a, **k):
        tc = R.TcSweep(*a, **k)
        tc.cfg, tc.stop, tc.ask = cfg, stop, ask
        for j in J:                                  # the start pose may sit outside SAFE_DEG (folded rest)
            tc.limits[j] = max(tc.limits[j], abs(float(tc.home[j])) + 1.0)
        holder["tc"] = tc
        if on_tc:
            on_tc(tc)
        return tc

    home = home or sweep.home_from_obs(tap.get_observation())
    old = sweep.MAX_RUNTIME_S
    sweep.MAX_RUNTIME_S = max_runtime_s
    try:
        code = sweep.run(tap, NullLink(), home, False, clock=clock or R.precise_clock(), sleep=sleep or R.precise_sleep,
                         segments=[job], log_path=log_path, extend_dwells=False, sweep_cls=factory)
    finally:
        sweep.MAX_RUNTIME_S = old
        if own_stop:
            stop.uninstall()
    return code, holder.get("tc")


# ---------------------------------------------------------------------------------------------------- poses
_SERVICE: Optional[np.ndarray] = None


def service_pose() -> np.ndarray:
    """Coupling pointing straight ahead (+x), axis level, face 240 mm out and 200 mm above the table, roll at the
    ENTRY angle (lock + 45): where owner presents a tool by hand for the twist check (Stage C)."""
    global _SERVICE
    if _SERVICE is None:
        kin = R.FaceKinematics()
        Rt = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])     # gripper x up, z back (-x)
        q, err = kin.ik(np.array([0.24, 0.0, 0.20]), Rt, [0.0, -10.0, 40.0, -30.0, 0.0])
        if err > 0.5:
            raise SR.SwapError(f"service pose IK residual {err:.2f} mm")
        q[4] += 45.0
        _SERVICE = q
    return _SERVICE.copy()


def approach_legs(start: Sequence[float], target: Sequence[float]) -> List[np.ndarray]:
    """Leashed legs start -> (TRANSIT) -> target, the reverse of sweep's park."""
    s = np.asarray(start, float)
    legs = []
    if np.max(np.abs(s - TRANSIT)) > sweep.PARK_VIA_TRANSIT_DEG and np.max(np.abs(np.asarray(target) - s)) > sweep.HOME_RADIUS_DEG:
        legs.append(TRANSIT.copy())
    legs.append(np.asarray(target, float))
    return legs


def legs_path(start, legs) -> List[np.ndarray]:
    path, cur = [np.asarray(start, float)], np.asarray(start, float)
    for g in legs:
        p = R.leash_path(cur, g)
        path += p
        cur = p[-1]
    return path


# ---------------------------------------------------------------------------------------------------- audit
def routine_waypoints(cfg: SR.Config = SR.Config()) -> Dict[str, Tuple[float, float, float]]:
    """Every dock-frame (dx, dz, droll) the routine visits (swap_routine.pick / drop / retries / wiggle)."""
    w = {"F": (*cfg.away_mm, 0), "F45": (*cfg.away_mm, cfg.twist_deg), "Fu": (cfg.away_mm[0], cfg.away_mm[1] + 10.0, 0), "A": (0, 35, 0), "A3": (0, 3, 0),
         "Cd": (0, -cfg.down_over_mm, 0), "C0": (0, 0, 0), "D": (0, 0, cfg.twist_deg + cfg.roll_over_deg),
         "Dback": (-cfg.standoff_mm, 0, cfg.twist_deg)}
    for dz in cfg.retry_dz_mm:
        c = -0.3 + cfg.axial_over_mm
        w[f"E{dz:+.1f}"] = (-cfg.standoff_mm, dz, cfg.twist_deg)
        w[f"C{dz:+.1f}"] = (c, dz, cfg.twist_deg)
        w[f"L{dz:+.1f}"] = (c, dz, -cfg.roll_over_deg)
        w[f"Lr{dz:+.1f}"] = (c, dz, 0)
    for d in (cfg.wiggle_deg, -cfg.wiggle_deg):
        w[f"wig{d:+.0f}"] = (0, 0, d)
    return w


def plan_checks(docks: Dict[str, SR.DockTeach], home: Sequence[float], actions: Sequence[Tuple[str, str]] = (),
                tool_on_start: bool = True, blade: Optional[R.BladeCal] = None) -> Tuple[List[str], Dict[str, Any]]:
    """Offline audit (no hardware): kinematics bridge, IK of every routine waypoint, joint limits, the approach and
    park legs (master + tool vs the table and every dock), the blade window.  Returns (problems, info)."""
    import tool_session
    probs: List[str] = []
    info: Dict[str, Any] = {"docks": {}}
    kin = R.FaceKinematics()
    seats = [d.q_seat for d in docks.values()]
    for name, d in docks.items():
        di: Dict[str, Any] = {}
        try:
            br = tool_session.check_kinematics_bridge(lambda *a: kin.face_offset_q(*a), d.q_seat)
        except Exception as exc:  # noqa: BLE001
            br = float("inf")
            probs.append(f"{name}: bridge test failed: {exc!r}")
        di["bridge_mm"] = round(br, 3)
        if not br < 0.5:
            probs.append(f"{name}: kinematics bridge error {br:.2f} mm >= 0.5 (fk zero offsets?)")
        if not -150.0 <= d.q_seat[4] <= 100.0:
            probs.append(f"{name}: lock roll {d.q_seat[4]:.1f} deg leaves no room for the +49 deg twist")
        worst = 0.0
        for wn, (dx, dz, dr) in routine_waypoints().items():
            q = kin.face_offset_q(d.q_seat, dx, dz, dr, d.q_seat)
            c = kin.dock_coords(d.q_seat, q)
            res = math.hypot(math.hypot(c[0] - dx, c[1]), c[2] - dz)
            worst = max(worst, res)
            for i, j in enumerate(J):
                if abs(q[i]) > sweep.SAFE_DEG[j]:
                    probs.append(f"{name} {wn}: {j} {q[i]:.1f} beyond SAFE_DEG {sweep.SAFE_DEG[j]}")
            if res > 0.5:
                probs.append(f"{name} {wn}: IK misses by {res:.2f} mm")
        di["ik_worst_mm"] = round(worst, 3)
        info["docks"][name] = di
    if actions:
        first = docks[actions[0][1]]
        qH = kin.face_offset_q(first.q_seat, *F_HIGH, first.q_seat)
        ap = legs_path(home, approach_legs(home, qH))
        probs += R.check_clearance(ap, tool_on_start, home, seats, label="approach")
        down = [kin.face_offset_q(first.q_seat, F_HIGH[0], F_HIGH[1] + (R.F_POSE[1] - F_HIGH[1]) * t, 0.0, first.q_seat)
                for t in np.linspace(0, 1, 12)]
        probs += R.check_clearance(down, tool_on_start, home, seats, label="descent to F")
        for i, j in enumerate(J):
            for q in [qH] + down:
                if abs(q[i]) > sweep.SAFE_DEG[j]:
                    probs.append(f"approach: {j} {q[i]:.1f} beyond SAFE_DEG")
                    break
        # the session ends at F of the last dock (drop and pick both end there), then sweep's park
        last = docks[actions[-1][1]]
        qF2 = kin.face_offset_q(last.q_seat, *R.F_POSE, last.q_seat)
        tool_end = tool_on_start
        for a, _ in actions:
            tool_end = a == "pick"
        probs += R.check_clearance(R.park_path(qF2, home), tool_end, home, seats, label="park")
        # drop transfer F -> A with the tool (and pick start) are routine legs (validated in the sim)
    if blade is not None:
        probs += blade.problems_for_swap()
    return probs, info


def friction_pose(docks: Dict[str, SR.DockTeach], home: Sequence[float], tool_on: bool = True
                  ) -> Tuple[np.ndarray, List[str]]:
    """A free pose for calibrate_friction (sim: dock offset (-20, 60)); higher if the +-8 deg excursions of any joint
    would bring the master / tool near the table or a dock.  Falls back to the service pose."""
    kin = R.FaceKinematics()
    seats = [d.q_seat for d in docks.values()]
    cands = []
    if docks:
        d0 = next(iter(docks.values()))
        cands = [kin.face_offset_q(d0.q_seat, -20.0, dz, 0.0, d0.q_seat) for dz in (60.0, 80.0, 100.0, 130.0)]
    sv = service_pose()
    sv[4] -= 45.0
    cands.append(sv)
    last = []
    for q in cands:
        if np.any(np.abs(q[:4]) > np.array([sweep.SAFE_DEG[j] for j in J[:4]]) - 8.0):
            continue
        exc = []
        for j in range(4):
            for s in (-8.0, 8.0):
                e = q.copy()
                e[j] += s
                exc.append(e)
        probs = R.check_clearance(exc, tool_on, home, seats, label="friction excursion")
        probs += R.check_clearance(legs_path(home, approach_legs(home, q)), tool_on, home, seats, label="friction approach")
        if not probs:
            return q, []
        last = probs
    return cands[-1], last


# ---------------------------------------------------------------------------------------------------- jobs
def _routine_conditions(tc: R.TcSweep) -> None:
    """The conditions the sim was validated in: lerobot's P 16 and fast servo profile, then let the sag settle."""
    sweep.set_body_p_gain(tc.robot, sweep.TELEOP_P_COEFFICIENT)
    tc.set_profile("fast")
    tc.settle(1.0)


def _approach(tc: R.TcSweep, target: Sequence[float], label: str) -> None:
    start = R.deg5(tc.robot.get_observation())
    for k, g in enumerate(approach_legs(start, target)):
        tc.leash_to(g, f"{label} {k + 1}")


def job_twist(levels: Sequence[float], tries: int, out: Dict[str, Any],
              service: Optional[Sequence[float]] = None) -> Callable[[R.TcSweep], None]:
    """Roll-only twists at the SERVICE pose (arm torque-held).  Lock = roll from entry to lock - overtravel; unlock =
    lock to entry + overtravel; each at Torque_Limit `level`, then relax to 25 %.  Every twist waits for ENTER."""

    def twist(tc: R.TcSweep, io: R.RealIO, body: np.ndarray, r0: float, r1: float, r_rest: float, level: float):
        tc.check_stop()                          # critic B2: never write 85 % after a Ctrl-C at the prompt
        T = 1.5 / tc.cfg.speed
        n = max(2, int(round(T * REAL_HZ)))
        io.set_roll_torque(level)
        tc.kind = "twist"
        peak, t_sat = 0.0, 0.0
        q = body.copy()
        for i in range(1, n + int(0.3 / tc.cfg.speed * REAL_HZ) + 1):
            s = min(i / n, 1.0)
            q[4] = r0 + (r1 - r0) * (0.5 - 0.5 * math.cos(math.pi * s))
            io.write(q, 0.0)
            ld = abs(float(tc.last_load[4]))
            peak = max(peak, ld)
            t_sat += (1.0 / REAL_HZ) if ld >= 0.9 * level else 0.0
        reached = float(R.deg5(tc.robot.get_observation())[4])
        io.set_roll_torque(0.25)
        tc.kind = "free"
        q[4] = r_rest
        for _ in range(int(0.3 / tc.cfg.speed * REAL_HZ) + 1):
            io.write(q, 0.0)
        end = float(R.deg5(tc.robot.get_observation())[4])
        temp = float(np.max(tc.last_temp)) if len(tc.last_temp) else float("nan")
        return {"level": level, "from": round(r0, 2), "to": round(r1, 2), "reached": round(reached, 2),
                "short_deg": round(abs(r1 - reached), 2), "rest_err_deg": round(end - r_rest, 2),
                "peak_load": round(peak, 3), "peak_Nm": round(peak * R.grip_cap.STALL_NM, 2),
                "sat_s": round(t_sat, 2), "temp_c": temp}

    def job(tc: R.TcSweep) -> None:
        tc.cfg.roll_stall_s = 1e9          # a short twist is the measurement here, not a fault (bounded in time)
        io = R.RealIO(tc)
        io.setup(need_twist=max(levels) > 0.55)
        sv = np.asarray(service if service is not None else service_pose(), float)
        start = R.deg5(tc.robot.get_observation())
        probs = R.check_clearance(legs_path(start, approach_legs(start, sv)), True, start, [], "service approach")
        if probs:
            raise SR.SwapError("service approach not clear: " + "; ".join(probs))
        _approach(tc, sv, "SERVICE")
        _routine_conditions(tc)
        io.resync()
        io.set_blade_ref(io.blade.deg(float(tc.robot.get_observation()["gripper.pos"])))
        body = R.deg5(tc.robot.get_observation())
        body[:4] = sv[:4]
        r_entry = float(sv[4])
        r_lock = r_entry - 45.0
        over = SR.Config().roll_over_deg
        out.update(service=sv.round(2).tolist(), r_entry=r_entry, r_lock=r_lock, protection=io.protection,
                   protection_warnings=io.protection_warnings, protection_notes=io.protection_notes, twists=[])
        tc.hands_on = True        # from here a person may hold the tool on the coupling: any failure HOLDS (B2)
        for level in levels:
            skip = False
            for k in range(tries):
                for kind in ("lock", "unlock"):
                    a = tc.prompt(f"[{level:.0%} {kind} {k + 1}/{tries}] "
                                  + ("present the tool at the ENTRY angle and hold its body firmly; " if kind == "lock"
                                     else "keep holding the tool body; ")
                                  + "ENTER = twist, s = skip to the next level, q = finish: ")
                    a = "q" if a is None else a.strip().lower()            # EOF / Ctrl-C at the prompt = finish
                    if a == "q":
                        out["stopped"] = "operator"
                        out["max_step_deg"] = io.max_step.round(2).tolist()
                        tc.prompt("Take the tool off the coupling (or leave it locked), hands clear, then ENTER to "
                                  "park: ")
                        tc.hands_on = False
                        return
                    if a == "s":
                        skip = True
                        break
                    if kind == "lock":
                        r = twist(tc, io, body, r_entry, r_lock - over, r_lock, level)
                    else:
                        r = twist(tc, io, body, r_lock, r_entry + over, r_entry, level)
                    r["kind"] = kind
                    out["twists"].append(r)
                    tc.note(f"twist {kind} @ {level:.0%}: peak {r['peak_load']:.0%} ({r['peak_Nm']} N.m), "
                            f"short {r['short_deg']} deg, rest err {r['rest_err_deg']} deg, {r['temp_c']:.0f} C")
                if skip:
                    break
        out["max_step_deg"] = io.max_step.round(2).tolist()
        tc.prompt("Ladder done. Hands clear of the arm, then ENTER to park: ")
        tc.hands_on = False

    return job


def job_friction(q_mid: Sequence[float], runs: int, out: Dict[str, Any]) -> Callable[[R.TcSweep], None]:
    def job(tc: R.TcSweep) -> None:
        io = R.RealIO(tc)
        io.setup(need_twist=False)
        _approach(tc, q_mid, "FRICTION")
        _routine_conditions(tc)
        io.resync()
        io.set_blade_ref(io.blade.deg(float(tc.robot.get_observation()["gripper.pos"])))
        io.routine_hz = REAL_HZ
        q0 = np.asarray(q_mid, float)
        bands = []
        for r in range(runs):
            tc.note(f"friction calibration run {r + 1}/{runs}: +-8 deg triangle per joint (pan, lift, elbow, wrist_flex)")
            f = SR.calibrate_friction(io, q0, hz=REAL_HZ, joints=range(4))
            bands.append(f.band_deg)
            tc.note(f"  bands {f.band_deg}")
        b = np.array(bands)
        out["runs"] = b.round(3).tolist()
        out["band_deg"] = [round(float(v), 3) for v in b.max(axis=0)]
        out["band_deg"][4] = SR.FrictionCal().band_deg[4]
        spread = (b.max(axis=0) - b.min(axis=0)) / np.maximum(b.max(axis=0), 1e-6)
        out["agree_30pct"] = bool(np.all(spread[:4] <= 0.30)) if runs > 1 else None
        out["q_mid"] = q0.round(3).tolist()

    return job


def job_teach(name: str, n: int, out: Dict[str, Any]) -> Callable[[R.TcSweep], None]:
    """No motion.  The body goes LIMP (owner supports the arm), the gripper / blade keeps torque at the cap."""

    def job(tc: R.TcSweep) -> None:
        io = R.RealIO(tc)
        io.setup(need_twist=False)
        ask = lambda p: (tc.prompt(p) or "")  # noqa: E731
        a = ask("Teach: SUPPORT THE ARM with one hand. Type LIMP to release the arm joints (the blade keeps its "
                "capped torque), anything else to cancel: ").strip().upper()
        if a != "LIMP":
            out["cancelled"] = True
            return
        tc.limp = True                        # first: a partial failure must not let the park drive a held arm
        tc.robot.bus.disable_torque(list(J))
        tc.note("arm joints LIMP (gripper/blade still torqued at the cap)")
        samples = []
        while len(samples) < n:
            a = ask(f"[{len(samples) + 1}/{n}] Seat the tool in dock '{name}' (neck on the land, body on the tail "
                    f"rest), hold the arm so it does not hang on the tool, then ENTER (q = abort): ").strip().lower()
            if a == "q":
                out["cancelled"] = True
                break
            obs = []
            for _ in range(13):
                obs.append(R.deg5(tc.robot.get_observation()))
                tc.sleep(0.04)
            samples.append(np.median(np.array(obs), axis=0))
            tc.note(f"  recorded {np.round(samples[-1], 2).tolist()}")
        if samples:
            s = np.array(samples)
            out["samples"] = s.round(3).tolist()
            out["q_seat"] = np.median(s, axis=0).round(3).tolist()
            out["spread_deg"] = (s.max(axis=0) - s.min(axis=0)).round(3).tolist()
        ask("Lift the tool out and lay the arm back on its rest by hand, then ENTER (torque stays off): ")

    return job


def job_swap(actions: Sequence[Tuple[str, str]], docks: Dict[str, SR.DockTeach], fric: SR.FrictionCal,
             tool_on_start: bool, out: Dict[str, Any], on_result: Optional[Callable[[dict], None]] = None
             ) -> Callable[[R.TcSweep], None]:
    """Approach the first dock's away pose F, run the actions (drop / pick) through RealSwapper, end at F."""

    def job(tc: R.TcSweep) -> None:
        io = R.RealIO(tc, fric)
        io.setup(need_twist=True)
        bp = io.blade.problems_for_swap()
        start = R.deg5(tc.robot.get_observation())
        probs, _ = plan_checks(docks, start, list(actions), tool_on_start)    # from where the arm REALLY is
        bp += probs
        if tool_on_start:                     # the drop turns the blade -45 (+ slack) from where it is now
            b0 = io.blade.deg(float(tc.robot.get_observation()["gripper.pos"]))
            lo, hi = io.blade.window
            if not (lo <= b0 - 50.0 and b0 + 5.0 <= hi):
                bp.append(f"blade at {b0:.0f} deg: the drop needs {b0 - 50:.0f}..{b0 + 5:.0f} inside {lo:.0f}..{hi:.0f}")
        if bp:
            raise SR.SwapError("refused before any motion: " + "; ".join(bp))
        first = docks[actions[0][1]]
        qH = io.face_offset_q(first.q_seat, *F_HIGH, first.q_seat)
        _approach(tc, qH, "APPROACH")
        _routine_conditions(tc)
        io.resync()
        if tool_on_start:
            io.set_blade_ref(io.blade.deg(float(tc.robot.get_observation()["gripper.pos"])))
        scfg = SR.Config(hz=tc.cfg.routine_hz)
        sw = R.RealSwapper(io, scfg, fric, log=tc.note)
        tc.retreat = make_retreat(tc, io, sw, docks)
        sw.goto(first, R.F_POSE, T=2.0 / tc.cfg.speed)            # Cartesian descent F_HIGH -> F (checked)
        tool_on = tool_on_start
        out["results"] = []
        for k, (action, name) in enumerate(actions):
            dock = docks[name]
            tc.note(f"--- swap {k + 1}/{len(actions)}: {action} at {name} ---")
            if action == "drop":
                if not tool_on:
                    raise SR.SwapError("drop requested but the master carries no tool")
                sw.goto(dock, (0.0, 35.0, 0.0))
                res = sw.run(sw.drop, dock)
            else:
                if tool_on:
                    raise SR.SwapError("pick requested but the master already carries a tool")
                sw.goto(dock, R.F_POSE)
                res = sw.run(sw.pick, dock)
            res = dict(res, action=action, t=round(tc.clock() - tc.t0, 1))
            out["results"].append(res)
            if on_result:
                on_result(res)
            tc.note(f"result: {json.dumps(res, default=float)}")
            if not res.get("ok"):
                why = f"{action} at {name} not ok: {res.get('error') or res}"
                if "error" in res and not (tc.engaged or tc.fault_hold):
                    # an exception outside contact: sweep's finally retreats to F and parks (normal safe park)
                    raise sweep._Abort(sweep.EXIT_ERROR, why)
                # engaged, E-stop, or the routine's own verdict (lock / weight check failed): the tool may be half
                # on the master -> hold in place and ask (PARK only once owner has looked)
                tc.fault_hold = True
                tc.fault_reason = tc.fault_reason or why
                # exit 4 comes from TcSweep.park() refusing (hold); an operator PARK that succeeds ends normally
                raise sweep._Abort(sweep.EXIT_ERROR, tc.fault_reason)
            tool_on = action == "pick"
        out["max_step_deg"] = io.max_step.round(2).tolist()
        out["limiter_clips"] = io.lim.clipped
        out["dilated_s"] = round(io.dilated_ticks / REAL_HZ, 1)

    return job


def make_retreat(tc: R.TcSweep, io: R.RealIO, sw: R.RealSwapper, docks: Dict[str, SR.DockTeach]):
    """Park helper: when the face is near a dock (but not engaged), back out along a Cartesian line to F first."""

    def retreat() -> None:
        q = R.deg5(tc.robot.get_observation())
        for name, d in docks.items():
            c = io.kin.dock_coords(d.q_seat, q)
            (x0, x1), (y0, y1), (z0, z1) = R.NEAR_BOX_MM
            if x0 < c[0] < x1 and y0 < c[1] < y1 and z0 < c[2] < z1:
                if R.in_zone((c[0], c[2])):
                    raise R.TcHold(f"retreat refused: the face is inside dock {name}'s zone {np.round(c, 1).tolist()}")
                if math.hypot(c[0] - R.F_POSE[0], c[2] - R.F_POSE[1]) < 2.0:
                    return
                tc.note(f"retreat from dock {name} {np.round(c[:3], 1).tolist()} mm to F before parking")
                io.resync()
                sw.q_cmd = np.array([float(tc.current[j]) for j in J])
                sw.I = np.clip(sw.I, -6.0, 6.0)
                sw.blade_off = 0.0
                sw.b_cmd = io.blade.deg(float(tc.current[R.GRIPPER])) - io.blade_ref
                io.guarded = False
                sweep.set_body_p_gain(tc.robot, sweep.PARK_P_COEFFICIENT)     # away from contact: sweep's park gain
                sw.p_now = sweep.PARK_P_COEFFICIENT
                try:
                    sw.goto(d, R.F_POSE, T=3.0 / tc.cfg.speed)
                finally:
                    io.guarded = True
                return

    return retreat


def save_log(kind: str, data: Dict[str, Any]) -> Path:
    LOGS.mkdir(exist_ok=True)
    p = LOGS / f"{kind}_{time.strftime('%Y%m%d_%H%M%S')}.json"
    p.write_text(json.dumps(data, indent=1, default=float))
    return p
