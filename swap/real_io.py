# SPDX-License-Identifier: GPL-3.0-or-later
"""RealIO: the swap routine's ArmIO on the real SO-101 follower, through dashboard/splat/sweep.py's control path.

Nothing here opens a port by itself.  run_swap.py connects with sweep.connect_real() and hands the robot to
jobs.run_session(), which runs everything inside sweep.run() with TcSweep (a sweep._Sweep subclass).  So every tick
goes through sweep's _step (SAFE_DEG clamp, 12 deg following-error watchdog, absolute 25 Hz grid, runtime cap) and the
session always ends in sweep.run's finally: park, or hold torque if the park is refused / fails.

What this module adds on top of sweep (TEST_PLAN_SO101.html Stage E, DESIGN_v4 S1-S5, HANDOFF 2026-10-05):
  * RealIO (swap_routine.ArmIO): encoder degrees as lerobot reports them; IK on splat/fk with the FITTED zero offsets
    (fk.lerobot_obs_to_q); the blade (gripper servo) in servo degrees through the wide follower_arm_tc gripper range.
  * Speed: the routine runs at Config.hz = 25 / speed while the real loop stays at 25 Hz, so speed 0.5 = every move
    takes twice as long (the 50 Hz sim step sequence) and speed 1.0 = the 25 Hz sim.  Plus a per-joint velocity /
    acceleration limiter on the command and a "commanded jump" guard that catches routine bugs.
  * Watchdogs per segment kind (free / contact / twist / park): tracking error |ref - q|, Present_Load (sign bit
    decoded, rule arm.control.load_sign), roll stall, temperature (2 reads), gripper cap read-back.
  * THE CONTACT RULE: while a tool is engaged with a dock (any contact-mode segment, or a segment that starts or ends
    inside the dock zone) a failure HOLDS torque in place (roll back to 25 %, gripper re-capped, no P-gain change)
    and sweep's park is refused, because a joint-space park would drag the docked tool and the dock.  Outside the
    zone the normal park runs (after a Cartesian retreat to the away pose F when near a dock).
  * E-stop: Ctrl-C (SIGINT handler, never a KeyboardInterrupt mid-packet) or the STOP file -> hold in place, always
    (also outside contact); then a post-fault menu in the same process (PARK only when not engaged / LIMP / exit
    holding), so the arm never has to be re-connected while it hangs in the air (lerobot's connect drops torque
    while it configures: memory arm-motion-safety, 2026-09-24).
"""
from __future__ import annotations

import os
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]                                   # roboticarm/
for p in (str(HERE), str(ROOT / "dashboard"), str(ROOT / "gripper" / "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)

from splat import fk, sweep  # noqa: E402
import grip_cap  # noqa: E402
import swap_routine as SR  # noqa: E402
import tool_session  # noqa: E402

J = fk.ARM_JOINTS
# 10/27: the taught dock seat has shoulder_pan 99.4 deg; sweep's 93 is calibrated half-range (108) - 15.  The changer swap
# is hand-supervised, so it runs with 5 deg of margin to the 108 stop instead (process-local; sweep.py is untouched elsewhere).
# Same for elbow_flex: the 30 mm standoff at the taught seat needs 79.9 deg (+ sag/tracking ~3); half-range is 98, limit 83 -> 90.
SWAP_SAFE_DEG = {"shoulder_pan": 103.0, "elbow_flex": 90.0}
sweep.SAFE_DEG.update(SWAP_SAFE_DEG)
BODY = sweep.BODY
GRIPPER = sweep.GRIPPER
REAL_HZ = sweep.HZ                      # 25: the real loop never runs faster than sweep's grid
STOP_FILE = HERE / "STOP"               # `run_swap.py stop` / stopbutton creates it -> hold in place

# ---------------------------------------------------------------------------------------------------- geometry
def _master_drop(default: float = 8.5) -> float:
    """cad/v4/master_v4.json MASTER_DROP: the 10/24 hub (14.5 mm, was 6) moves the whole master stack this far
    from the wrist (same source as sim/geom.params)."""
    import json
    try:
        return float(json.loads((HERE.parent / "cad" / "v4" / "master_v4.json").read_text())["params"]["MASTER_DROP"])
    except Exception:  # noqa: BLE001
        return default


MASTER_DROP_MM = _master_drop()
#: coupling face on the roll axis, gripper_link frame (= sim site "face": Z_TOOL_FACE_LOCK -62.2 mm in the v3.1
#: stack, -70.7 with the 10/24 master; the MuJoCo gripper body and fk's gripper_link agree to 0.01 mm / 0.001 deg)
FACE_Z_M = -(62.2 + MASTER_DROP_MM) / 1000.0
#: changer tool tip on the roll axis: ~30 mm past the stock jaw tip (TIP_G 98 mm) in v3.1, + the master drop
TC_TIP_Z_M = -(128.0 + MASTER_DROP_MM) / 1000.0
SKIRT_R_M = 0.040                       # skirt mouth radius (DESIGN_v4 r3: mouth r 40)
TABLE_MIN_M = sweep.TIP_MIN_M           # 30 mm, same rule as sweep's jaw tip
#: dock zone (dock frame mm, x along the tool axis into the wall, z up, origin = taught seat face): a segment that
#: starts or ends in here counts as "engaged" even in free mode (e.g. the 3 mm take-the-weight lift)
ZONE_DX_MM = -20.0                      # dx > -20: the pins (8 mm) and the 2 mm overtravel are well inside
ZONE_DZ_MM = 25.0                       # dz < 25: neck still between the slot walls (A = 35 mm is outside)
#: region from which a park first retreats along the axis to F (dock frame mm)
NEAR_BOX_MM = ((-110.0, 100.0), (-80.0, 80.0), (-200.0, 90.0))
#: dock + parked tool keep-out for approach / park legs (not for the routine's own legs), dock frame mm
DOCK_KEEPOUT_MM = ((-10.0, 90.0), (-45.0, 45.0), (-150.0, 25.0))
F_POSE = (*SR.Config.away_mm, 0.0)             # the routine's away pose (dx, dz, droll); weigh pose, approach target (from SR.Config.away_mm)

# ---------------------------------------------------------------------------------------------------- blade
ENTRY_DEG = 300.0                       # blade servo angle at the pick entry (DESIGN_v4 2c: entry >= 270 on 0-360)
BLADE_MARGIN_DEG = 3.0                  # commands stay this far inside the calibrated gripper range
SEARCH_SPAN_DEG = 260.0                 # swap_routine._blade_search: 26 x 10 deg
TC_GRIP_MIN_SPAN_DEG = 300.0            # a "wide" (follower_arm_tc) gripper calibration spans at least this


class EStop(SR.SwapError):
    """Operator stop (Ctrl-C or STOP file): hold in place."""


class ContactFault(SR.SwapError):
    """A watchdog tripped (tracking, load, stall, temperature, gripper cap)."""


class TcHold(RuntimeError):
    """Raised from TcSweep.park() when the arm must keep holding (sweep.run then exits 4, torque held)."""


# ---------------------------------------------------------------------------------------------------- helpers
def decode_load(v: Any) -> int:
    """Present_Load -> signed per-mille of max torque.  lerobot already decodes the sign-magnitude register (bit 10 =
    direction) and returns negative ints; a raw register value (e.g. a bus read with the encoding table bypassed)
    still carries bit 10.  Masking a decoded negative with 0x3FF turned -20 into 1004 and tripped camarm's load guard
    on 2026-10-04 (rule arm.control.load_sign)."""
    v = int(v)
    if v < 0:
        return v
    mag = v & 0x3FF
    return -mag if (v >> 10) & 1 else mag


def precise_clock() -> Callable[[], float]:
    """perf_counter + a 1 ms Windows timer for this process (time.monotonic and time.sleep tick in 15.6 ms steps on
    Windows: camarm 2026-10-05 measured 31/47 ms ticks instead of 40 at sweep's default clock)."""
    if sys.platform == "win32":
        try:
            import atexit
            import ctypes
            winmm = ctypes.WinDLL("winmm")
            if winmm.timeBeginPeriod(1) == 0:
                atexit.register(winmm.timeEndPeriod, 1)
        except Exception:  # noqa: BLE001
            pass
    return time.perf_counter


def precise_sleep(dt: float) -> None:
    """Sleep to within ~0.3 ms: coarse sleep, then spin on perf_counter for the last 2 ms."""
    end = time.perf_counter() + dt
    if dt > 0.003:
        time.sleep(dt - 0.002)
    while time.perf_counter() < end:
        pass


def deg5(obs: Dict[str, float]) -> np.ndarray:
    return np.array([float(obs[f"{j}.pos"]) for j in J])


# ---------------------------------------------------------------------------------------------------- kinematics
def _q(d5: Sequence[float]) -> Dict[str, float]:
    return fk.lerobot_obs_to_q({f"{j}.pos": float(v) for j, v in zip(J, d5)})


def T_gripper(d5: Sequence[float]) -> np.ndarray:
    return fk.fk_gripper(_q(d5))


def face_pose(d5: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    T = T_gripper(d5)
    return T[:3, 3] + T[:3, 2] * FACE_Z_M, T[:3, :3].copy()


def _Rz(a_deg: float) -> np.ndarray:
    a = math.radians(a_deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


class FaceKinematics:
    """face_offset_q for the real arm: same maths as sim_io.SimIO.face_offset_q (dock frame from the taught seat, DLS
    IK, position + 0.3 x rotation), but on splat/fk with the fitted zero offsets, in lerobot degrees."""

    W_ROT = 0.3

    def __init__(self) -> None:
        self._seat_key: Optional[tuple] = None
        self._seat = None
        self._memo: Dict[tuple, np.ndarray] = {}

    def seat_frame(self, q_seat: Sequence[float]):
        key = tuple(np.round(np.asarray(q_seat, float), 4))
        if key != self._seat_key:
            p, R = face_pose(q_seat)
            xD = -R[:, 2].copy()
            xD[2] = 0.0
            xD /= np.linalg.norm(xD)
            zD = np.array([0.0, 0.0, 1.0])
            yD = np.cross(zD, xD)
            self._seat = (p, R, np.column_stack([xD, yD, zD]))
            self._seat_key = key
            self._memo = {}
        return self._seat

    def ik(self, p_t: np.ndarray, R_t: np.ndarray, q0: Sequence[float], iters: int = 200) -> Tuple[np.ndarray, float]:
        x = np.radians(np.asarray(q0, float))
        h = 1e-6
        for _ in range(iters):
            p, R = face_pose(np.degrees(x))
            ep = p_t - p
            Re = R_t @ R.T
            er = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            if np.linalg.norm(ep) < 1e-6 and np.linalg.norm(er) < 1e-5:
                break
            e = np.concatenate([ep, self.W_ROT * er])
            Jm = np.zeros((6, 5))
            for i in range(5):
                xi = x.copy()
                xi[i] += h
                p2, R2 = face_pose(np.degrees(xi))
                Re2 = R2 @ R.T                      # small rotation R -> R2 (world frame)
                w = 0.5 * np.array([Re2[2, 1] - Re2[1, 2], Re2[0, 2] - Re2[2, 0], Re2[1, 0] - Re2[0, 1]])
                Jm[:3, i] = (p2 - p) / h
                Jm[3:, i] = self.W_ROT * w / h
            dx = Jm.T @ np.linalg.solve(Jm @ Jm.T + 1e-6 * np.eye(6), e)
            x = x + np.clip(dx, -0.1, 0.1)
        p, _ = face_pose(np.degrees(x))
        return np.degrees(x), float(np.linalg.norm(p_t - p) * 1000)

    def face_offset_q(self, q_seat, dx, dz, droll, q0) -> np.ndarray:
        p, R, RD = self.seat_frame(q_seat)
        key = (round(float(dx), 6), round(float(dz), 6), round(float(droll), 6))
        if key in self._memo:                     # the 5-DOF solution near the seat is unique: seed-independent
            return self._memo[key].copy()
        # continuation: march from the seed's own dock coordinates in <= 5 mm / 5 deg steps (a direct solve from a
        # far seed can land on another branch: 2026-10-24, F from the seat seed gave elbow 96 deg, 118 mm off)
        q = np.asarray(q0, float)
        c = self.dock_coords(q_seat, q)
        a = np.array([c[0], c[2], c[3]])
        b = np.array([dx, dz, droll], float)
        n = max(1, int(np.ceil(max(abs(b[0] - a[0]), abs(b[1] - a[1]), abs(b[2] - a[2])) / 5.0)))
        for i in range(1, n + 1):
            s = a + (b - a) * (i / n)
            q, err_mm = self.ik(p + RD @ np.array([s[0], 0.0, s[1]]) * 1e-3, R @ _Rz(s[2]), q)
        if not np.isfinite(q).all() or err_mm > 1.0:
            raise SR.SwapError(f"IK failed at dock offset {key} ({err_mm:.2f} mm)")
        self._memo[key] = q
        return q.copy()

    def dock_coords(self, q_seat, q) -> np.ndarray:
        """(dx, dy, dz) mm of the face in the dock frame + droll deg (about the tool axis) for joint angles q."""
        p0, R0, RD = self.seat_frame(q_seat)
        p, R = face_pose(q)
        d = RD.T @ (p - p0) * 1000.0
        M = R0.T @ R
        return np.array([d[0], d[1], d[2], math.degrees(math.atan2(M[1, 0], M[0, 0]))])


def in_zone(a: Optional[Sequence[float]]) -> bool:
    return a is not None and a[0] > ZONE_DX_MM and a[1] < ZONE_DZ_MM


def tc_points(d5: Sequence[float], tool_on: bool) -> np.ndarray:
    """World points (m) that bound the master (+ tool): the skirt ring at the face, and the tool body / tip."""
    T = T_gripper(d5)
    pts = []
    for k in range(8):
        a = 2 * math.pi * k / 8
        pts.append([SKIRT_R_M * math.cos(a), SKIRT_R_M * math.sin(a), FACE_Z_M])
    if tool_on:
        for z in np.linspace(FACE_Z_M, TC_TIP_Z_M, 5):
            for k in range(4):
                a = 2 * math.pi * k / 4
                pts.append([0.012 * math.cos(a), 0.012 * math.sin(a), z])
        pts.append([0.0, 0.0, TC_TIP_Z_M])
    P = np.array(pts)
    return (T[:3, :3] @ P.T).T + T[:3, 3]


def leash_path(q_from: Sequence[float], q_to: Sequence[float], leash: float = sweep.PARK_LEASH_DEG,
               arrive: float = sweep.PARK_ARRIVE_DEG, sub_deg: float = 1.0) -> List[np.ndarray]:
    """Kinematic preview of sweep._Sweep._move_leashed (each tick commands <= leash ahead of the measured pose per
    joint; assumes perfect tracking), sub-sampled every ~sub_deg for clearance checks."""
    cur = np.asarray(q_from, float).copy()
    to = np.asarray(q_to, float)
    out = [cur.copy()]
    for _ in range(10000):
        if np.max(np.abs(to - cur)) <= arrive:
            break
        nxt = cur + np.clip(to - cur, -leash, leash)
        n = max(1, int(np.ceil(np.max(np.abs(nxt - cur)) / sub_deg)))
        out += [cur + (nxt - cur) * (i / n) for i in range(1, n + 1)]
        cur = nxt
    n = max(1, int(np.ceil(np.max(np.abs(to - cur)) / sub_deg)))
    out += [cur + (to - cur) * (i / n) for i in range(1, n + 1)]
    return out


def park_path(q_from: Sequence[float], home: Sequence[float]) -> List[np.ndarray]:
    """Preview of sweep._Sweep.park(): via TRANSIT when far from home, then to home (both leashed)."""
    q = np.asarray(q_from, float)
    h = np.asarray(home, float)
    tr = np.array([sweep.TRANSIT[j] for j in J])
    path = [q]
    if np.max(np.abs(q - h)) > sweep.HOME_RADIUS_DEG and np.max(np.abs(q - tr)) > sweep.PARK_VIA_TRANSIT_DEG:
        path = leash_path(q, tr)
        q = path[-1]
    if np.max(np.abs(q - h)) > 0:
        path += leash_path(q, h)
    return path


def check_clearance(path: Sequence[Sequence[float]], tool_on: bool, home: Sequence[float],
                    docks: Sequence[Sequence[float]] = (), label: str = "") -> List[str]:
    """Approach / park leg audit: master (+ tool) points >= TABLE_MIN_M above the table outside sweep's home radius,
    outside every dock keep-out box, joints inside SAFE_DEG (or the start pose).  Returns problems ([] = ok)."""
    kin = FaceKinematics()
    h = np.asarray(home, float)
    lim = np.array([max(sweep.SAFE_DEG[j], abs(h[i]) + 1.0) for i, j in enumerate(J)])
    probs: List[str] = []
    for k, q in enumerate(path):
        q = np.asarray(q, float)
        if np.any(np.abs(q) > lim):
            probs.append(f"{label} step {k}: joint beyond SAFE_DEG {np.round(q, 1).tolist()}")
            break
        P = tc_points(q, tool_on)
        if np.max(np.abs(q - h)) > sweep.HOME_RADIUS_DEG and float(P[:, 2].min()) < TABLE_MIN_M:
            probs.append(f"{label} step {k}: changer {P[:, 2].min() * 1000:.0f} mm above the table "
                         f"(< {TABLE_MIN_M * 1000:.0f})")
            break
        for di, qs in enumerate(docks):
            p0, _, RD = kin.seat_frame(qs)
            L = (RD.T @ (P - p0).T).T * 1000.0
            (x0, x1), (y0, y1), (z0, z1) = DOCK_KEEPOUT_MM
            inside = (L[:, 0] > x0) & (L[:, 0] < x1) & (L[:, 1] > y0) & (L[:, 1] < y1) & (L[:, 2] > z0) & (L[:, 2] < z1)
            if inside.any():
                probs.append(f"{label} step {k}: enters dock {di} keep-out at {np.round(L[inside][0], 0).tolist()} mm")
                break
        if probs:
            break
    return probs


# ---------------------------------------------------------------------------------------------------- blade cal
@dataclass
class BladeCal:
    """Gripper servo (the changer blade) through lerobot's RANGE_0_100 gripper normalisation."""
    range_min: int
    range_max: int
    drive_mode: int = 0

    @classmethod
    def from_robot(cls, robot: Any) -> "BladeCal":
        c = robot.bus.calibration[GRIPPER]
        return cls(int(c.range_min), int(c.range_max), int(getattr(c, "drive_mode", 0)))

    @property
    def span_deg(self) -> float:
        return (self.range_max - self.range_min) * 360.0 / 4096.0

    @property
    def window(self) -> Tuple[float, float]:
        return (self.range_min * 360.0 / 4096 + BLADE_MARGIN_DEG, self.range_max * 360.0 / 4096 - BLADE_MARGIN_DEG)

    def pct(self, servo_deg: float) -> float:
        tick = servo_deg * 4096.0 / 360.0
        p = (tick - self.range_min) / (self.range_max - self.range_min) * 100.0
        return 100.0 - p if self.drive_mode else p

    def deg(self, pct: float) -> float:
        p = 100.0 - pct if self.drive_mode else pct
        return (self.range_min + p / 100.0 * (self.range_max - self.range_min)) * 360.0 / 4096.0

    def problems_for_swap(self) -> List[str]:
        lo, hi = self.window
        out = []
        if self.span_deg < TC_GRIP_MIN_SPAN_DEG:
            out.append(f"gripper range spans {self.span_deg:.0f} deg < {TC_GRIP_MIN_SPAN_DEG:.0f}: this is the stock "
                       f"calibration; the blade search would hit the range end and fake a stall. Use --id "
                       f"follower_arm_tc (gripcal.py)")
        if ENTRY_DEG - SEARCH_SPAN_DEG < lo or ENTRY_DEG + 49.0 > hi:
            out.append(f"blade window {lo:.0f}..{hi:.0f} deg does not hold entry {ENTRY_DEG:.0f} - search "
                       f"{SEARCH_SPAN_DEG:.0f} .. entry + 49")
        return out


# ---------------------------------------------------------------------------------------------------- limits
@dataclass
class RealConfig:
    speed: float = 0.5                          # 0.5 = every routine move takes 2x (first runs); 1.0 = the 25 Hz sim
    # per-joint limits on the COMMAND (5 arm joints), deg/s, deg/s^2; blade separately
    # body 25 deg/s (sweep's MAX_STEP_DEG x HZ is 37.5; at P 16 the loaded lift lags > 12 deg at 35-50 deg/s in the
    # twin, which trips sweep's following-error watchdog); roll is light
    vmax: Tuple[float, ...] = (25.0, 25.0, 25.0, 25.0, 90.0)
    amax: Tuple[float, ...] = (100.0, 100.0, 100.0, 100.0, 1500.0)
    #: time dilation: when the limiter is behind the routine's target by more than this (deg), the routine waits
    #: (extra real ticks, same target) until the command catches up - its path and holds are kept, only slower
    catchup_deg: float = 0.5
    catchup_blade_deg: float = 2.0
    catchup_max_s: float = 10.0
    blade_vmax: float = 300.0
    blade_amax: float = 8000.0
    # bug guard: the routine's requested target may never jump more than this per real tick (deg)
    jump_max: Tuple[float, ...] = (6.0, 6.0, 6.0, 6.0, 10.0)       # 150 / 250 deg/s: only a real jump trips it
    blade_jump_max: float = 30.0
    # watchdogs (thresholds checked against the MuJoCo twin, sim/out/realio_*: see RUNBOOK)
    track_contact_deg: float = 3.0              # |ref - q| body joints while engaged: this + the joint's measured
                                                # friction band (a 2.7 deg elbow band lags ~5 deg in a good pick, twin 5007)
    track_contact_ticks: int = 3
    track_free_deg: float = 8.0
    track_free_ticks: int = 5
    load_contact_max: float = 0.70              # |Present_Load| body joints while engaged
    load_contact_s: float = 0.4
    load_free_max: float = 0.90
    load_free_s: float = 1.0
    roll_stall_err_deg: float = 10.0            # roll |cmd - q| beyond the 4 deg overtravel + margin ...
    roll_stall_frac: float = 0.90               # ... at >= 90 % of its Torque_Limit ...
    roll_stall_s: float = 0.4                   # ... for this long (x 1/speed) -> stall
    temp_max_c: float = 55.0                    # Config.temp_pause_c; 2 consecutive reads
    grip_verify_s: float = 2.0
    load_every_contact: int = 1                 # Present_Load read every n ticks
    load_every_free: int = 4
    roll_torque_max: float = 0.85
    hold_s: float = 0.6
    ack_protection: bool = False                # roll protection registers may trip in the 85 % twist: operator ack

    @property
    def routine_hz(self) -> float:
        return REAL_HZ / self.speed


class Limiter:
    """Per-joint velocity + acceleration limit on a command stream that never overshoots its target: the step toward
    the target is at most min(vmax dt, |v_prev| dt + amax dt^2) and never past it (deceleration is not limited)."""

    def __init__(self, vmax: Sequence[float], amax: Sequence[float], dt: float) -> None:
        self.vmax, self.amax, self.dt = np.asarray(vmax, float), np.asarray(amax, float), dt
        self.last: Optional[np.ndarray] = None
        self.v = np.zeros(len(self.vmax))
        self.clipped = 0

    def reset(self, x: Sequence[float]) -> None:
        self.last = np.asarray(x, float).copy()
        self.v[:] = 0.0

    def step(self, target: Sequence[float]) -> np.ndarray:
        t = np.asarray(target, float)
        if self.last is None:
            self.reset(t)
            return t.copy()
        want = t - self.last
        same = np.sign(want) == np.sign(self.v)
        allow = np.minimum(self.vmax * self.dt, np.where(same, np.abs(self.v) * self.dt, 0.0) + self.amax * self.dt ** 2)
        s = np.clip(want, -allow, allow)
        if np.any(np.abs(s) < np.abs(want) - 1e-9):
            self.clipped += 1
        self.last = self.last + s
        self.v = s / self.dt
        return self.last.copy()


# ---------------------------------------------------------------------------------------------------- the sweep
class _ObsTap:
    """Robot proxy for sweep.run: forwards everything, remembers the last observation sweep's _step read."""

    def __init__(self, robot: Any) -> None:
        self._robot = robot
        self.last_obs: Optional[Dict[str, float]] = None

    def get_observation(self):
        self.last_obs = self._robot.get_observation()
        return self.last_obs

    def __getattr__(self, name):
        return getattr(self._robot, name)


class StopFlag:
    """SIGINT -> flag (Ctrl-C never raises inside a bus transaction); STOP file also counts."""

    def __init__(self, stop_file: Path = STOP_FILE) -> None:
        self.stop_file = stop_file
        self.sigint = False
        self._old = None

    def install(self) -> "StopFlag":
        if threading.current_thread() is threading.main_thread():
            self._old = signal.signal(signal.SIGINT, self._on_sigint)
        try:
            self.stop_file.unlink()                     # a stale STOP from an earlier run must not abort this one
        except FileNotFoundError:
            pass
        except OSError as exc:                          # a file lock etc.: refuse to start rather than miss stops
            raise SR.SwapError(f"cannot clear {self.stop_file} ({exc!r}); delete it by hand")
        return self

    def uninstall(self) -> None:
        if self._old is not None:
            signal.signal(signal.SIGINT, self._old)
            self._old = None

    def _on_sigint(self, *_):
        self.sigint = True
        print("\n[tc] Ctrl-C: E-STOP -> holding torque in place", flush=True)

    def why(self) -> Optional[str]:
        if self.sigint:
            return "Ctrl-C"
        if self.stop_file.exists():
            return f"STOP file {self.stop_file.name}"
        return None


class TcSweep(sweep._Sweep):
    """sweep._Sweep for changer sessions.  Same _step / reporter / park; adds the stop flag, the watchdogs, the
    contact rule and the post-fault menu.  Created by sweep.run(sweep_cls=...) via jobs.run_session()."""

    # set by jobs.run_session through the factory
    cfg: RealConfig
    stop: StopFlag
    ask: Optional[Callable[[str], str]] = None      # post-fault menu input (None = non-interactive -> keep holding)
    job: Optional[Callable[["TcSweep"], None]] = None
    log_fn: Callable[[str], None] = staticmethod(sweep._log)

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.engaged = False
        self.fault_hold = False
        self.fault_reason: Optional[str] = None
        self.limp = False                  # teach: body torque off by design -> nothing to park
        self.retreat: Optional[Callable[[], None]] = None
        self.io: Optional["RealIO"] = None
        self.kind = "free"                 # watchdog segment kind: free | contact | twist | park | hold
        self.ref: Optional[np.ndarray] = None
        self.n_ticks = 0
        self._holding = False
        self._track_bad = 0
        self._load_bad_s = 0.0
        self._roll_stall_s = 0.0
        self._hot = 0
        self._last_grip_verify = None
        self._load_fail = 0
        self.grip_raw: Optional[int] = None
        self._hold_pose: Optional[Dict[str, float]] = None
        self.hold_q: Optional[np.ndarray] = None
        self.hands_on = False             # twist: a person holds the tool on the coupling -> any failure holds
        self._last_roll_verify = None
        self.roll_tl0: Optional[int] = None
        self.roll_tl: Optional[int] = None
        self.peaks: Dict[str, Dict[str, float]] = {}
        self.events: List[str] = []
        self.last_load = np.zeros(5)
        self.last_temp = np.zeros(6)

    # -- sweep.run hook: no P-gain change before the finally-park while holding in contact
    @property
    def park_gain(self):
        return None if (self.engaged or self.fault_hold or self.limp or self.hands_on) else sweep.PARK_P_COEFFICIENT

    def prompt(self, text: str) -> Optional[str]:
        """Operator prompt.  None on EOF / Ctrl-C (callers treat None as the safe choice).  Time spent waiting does
        not count toward sweep's runtime cap (critic B2)."""
        if self.ask is None:
            return ""
        t = self.clock()
        try:
            a = self.ask(text)
        except (EOFError, KeyboardInterrupt):
            a = None
        self.t0 += self.clock() - t
        return a

    def note(self, msg: str) -> None:
        self.events.append(msg)
        self.log_fn(msg)

    # -- stop
    def check_stop(self) -> None:
        why = self.stop.why() if getattr(self, "stop", None) else None
        if why:
            self.fault_hold = True
            self.fault_reason = self.fault_reason or f"E-STOP ({why})"
            raise EStop(f"E-STOP: {why}")

    # -- one tick
    def _step(self, pose, phase, label, progress, guarded):
        if not self._holding:
            self.check_stop()
            if not getattr(self, "_capguarded", False):          # pm-eng 10/30: real motion only on a live take
                self._capguarded = True
                rn = type(getattr(self.robot, "_robot", self.robot)).__name__
                if not rn.startswith("Fake") and "Sim" not in rn and os.environ.get("TC_FAKE_BUS") != "1":
                    import capguard
                    capguard.require("real_io:" + os.path.basename(sys.argv[0]))
        try:
            super()._step(pose, phase, label, progress, guarded)
        except sweep._Abort as ab:
            if ab.code == sweep.EXIT_FOLLOW_ERR and phase != "park":     # blocked arm: hold, operator decides
                self.fault_hold = True
                self.fault_reason = self.fault_reason or f"sweep watchdog: {ab.reason}"
            raise
        self.n_ticks += 1
        if not self._holding:
            try:
                self._watch(pose, phase)
            except ContactFault as exc:
                self.note(f"WATCHDOG ({'engaged' if self.engaged else self.kind}): {exc}")
                if phase != "park":            # critic S3: something physical is wrong -> no automatic park
                    self.fault_hold = True
                    self.fault_reason = self.fault_reason or f"watchdog: {exc}"
                raise

    def _peak(self, key: str, vals: np.ndarray) -> None:
        r = self.peaks.setdefault(f"{self.kind}:{key}", {})
        for i, v in enumerate(np.abs(vals)):
            n = J[i] if i < len(J) else str(i)
            r[n] = max(r.get(n, 0.0), float(v))

    def _watch(self, pose, phase) -> None:
        cfg, dt = self.cfg, 1.0 / REAL_HZ
        kind = "park" if phase == "park" else self.kind
        obs = getattr(self.robot, "last_obs", None)
        if obs is None:
            return
        q = deg5(obs)
        cmd = np.array([float(pose[j]) for j in J])
        ref = self.ref if self.ref is not None else cmd
        track = np.abs(ref - q)[:4]
        engaged = self.engaged and kind not in ("park", "leash")
        if engaged:
            band = np.asarray((self.io.fric if self.io is not None else SR.FrictionCal()).band_deg[:4], float)
            lim, nt = cfg.track_contact_deg + band, cfg.track_contact_ticks
        else:
            lim, nt = np.full(4, cfg.track_free_deg), cfg.track_free_ticks
        if kind in ("park", "leash"):            # leashed legs command 8 deg ahead by design: sweep's 12 deg rule
            track = np.zeros(4)
        self._peak("track", track)
        self._track_bad = self._track_bad + 1 if np.any(track > lim) else 0
        if self._track_bad >= nt:
            raise ContactFault(f"tracking error {track.max():.1f} deg on {J[int(track.argmax())]} for {nt} ticks "
                               f"({'engaged' if engaged else kind})")
        every = cfg.load_every_contact if engaged or kind == "twist" else cfg.load_every_free
        if self.n_ticks % max(1, every) == 0:
            try:
                raw = self.robot.bus.sync_read("Present_Load", normalize=False)
                load = np.array([decode_load(raw[j]) for j in J]) / 1000.0
                self._load_fail = 0
            except Exception as exc:  # noqa: BLE001  a missed read is not a stop by itself (camarm rule)
                self._load_fail += 1
                if self._load_fail >= 5 and engaged:
                    raise ContactFault(f"Present_Load unreadable 5x while engaged ({exc!r})")
                load = None
            if load is not None:
                self.last_load = load
                self._peak("load", load)
                lmax = cfg.load_contact_max if engaged else cfg.load_free_max
                lsec = cfg.load_contact_s if engaged else cfg.load_free_s
                body_hi = np.abs(load[:4]).max() > lmax and kind not in ("park", "leash")   # leash = full push by design
                self._load_bad_s = self._load_bad_s + dt * every if body_hi else 0.0
                if self._load_bad_s >= lsec:
                    raise ContactFault(f"body load {np.abs(load[:4]).max():.0%} > {lmax:.0%} for {lsec:.1f} s")
                rl = (self.roll_tl or 1000) / 1000.0
                rerr = abs(cmd[4] - q[4])
                if abs(load[4]) >= cfg.roll_stall_frac * rl and rerr > cfg.roll_stall_err_deg:
                    self._roll_stall_s += dt * every
                else:
                    self._roll_stall_s = 0.0
                if self._roll_stall_s >= cfg.roll_stall_s / cfg.speed:
                    raise ContactFault(f"wrist_roll stalled: {abs(load[4]):.0%} load, {rerr:.1f} deg behind")
        if self.n_ticks % (REAL_HZ // 2) == 0:
            try:
                t = self.robot.bus.sync_read("Present_Temperature", normalize=False)
                temp = np.array([float(t[m]) for m in J + [GRIPPER] if m in t])
                self.last_temp = temp
                self._peak("temp", temp)
                self._hot = self._hot + 1 if temp.max() > cfg.temp_max_c else 0
                if self._hot >= 2:
                    raise ContactFault(f"servo temperature {temp.max():.0f} C > {cfg.temp_max_c:.0f} C on 2 reads")
            except ContactFault:
                raise
            except Exception:  # noqa: BLE001
                pass
        if self.roll_tl is not None:
            now = self.clock()
            if self._last_roll_verify is None or now - self._last_roll_verify >= cfg.grip_verify_s:
                self._last_roll_verify = now
                got = int(self.robot.bus.read("Torque_Limit", "wrist_roll", normalize=False))
                if got != self.roll_tl:
                    raise ContactFault(f"wrist_roll Torque_Limit reads {got}, expected {self.roll_tl} (servo rebooted?)")
        if self.grip_raw is not None:
            now = self.clock()
            if self._last_grip_verify is None or now - self._last_grip_verify >= cfg.grip_verify_s:
                self._last_grip_verify = now
                try:
                    grip_cap.verify_grip_cap(self.robot.bus, self.grip_raw)
                except RuntimeError as exc:
                    raise ContactFault(f"gripper cap lost: {exc}") from None

    # -- registers
    def set_roll_limit(self, frac: float) -> None:
        raw = int(round(frac * 1000))
        if raw == self.roll_tl:
            return
        bus = self.robot.bus
        bus.write("Torque_Limit", "wrist_roll", raw, normalize=False)
        got = int(bus.read("Torque_Limit", "wrist_roll", normalize=False))
        if got != raw:
            raise ContactFault(f"wrist_roll Torque_Limit read back {got}, wrote {raw}")
        self.roll_tl = raw

    def set_profile(self, kind: str) -> None:
        """'slow' = connect_real's profile (Acceleration 20, Goal_Velocity 300) for the leashed approach / park legs;
        'fast' = lerobot's (what the sim's servo model was fitted on) for the routine and the twist."""
        if kind == "fast":
            sweep.restore_fast_profile(self.robot)
            return
        try:
            for m in self.robot.bus.motors:
                self.robot.bus.write("Acceleration", m, 20)
            self.robot.bus.sync_write("Goal_Velocity", {m: 300 for m in self.robot.bus.motors}, normalize=False)
        except Exception as exc:  # noqa: BLE001  cosmetic: the leash is the primary speed limit
            self.note(f"slow profile write failed (continuing): {exc!r}")

    # -- holding
    def hold_in_place(self, reason: str) -> None:
        """Keep torque ON where the arm is: body goal = measured + the last holding offset clipped to the friction
        clamp (keeps gravity support, drops any push into an obstruction); roll at its measured angle at 25 %; the
        gripper cap re-applied.  Never parks, never changes P.  Safe to call more than once."""
        first = not self._holding
        self._holding = True
        self.fault_hold = True
        self.fault_reason = self.fault_reason or reason
        if first:
            self.note(f"NEEDS_ATTENTION: {reason} -> HOLDING torque in place"
                      + (" (tool ENGAGED with the dock: no park)" if self.engaged else ""))
        try:
            obs = self.robot.get_observation()
        except Exception as exc:  # noqa: BLE001
            self.note(f"hold: bus unreadable ({exc!r}); the servos keep their last goal")
            return
        q = deg5(obs)
        if self.hold_q is None:
            self.hold_q = q.copy()                 # where the arm was when the hold started (tests, log)
        if self._hold_pose is None:
            # computed ONCE (critic B1: re-basing on the sagged measurement each call ratcheted the arm down).
            # Not engaged: keep the full last command (all its gravity support).  Engaged: keep the support but
            # drop any push into an obstruction beyond the contact clamp + 1 deg.
            last = np.array([float(self.current[j]) for j in J])
            if self.engaged:
                c = np.append(SR.FrictionCal(self.io.fric.band_deg if self.io else SR.FrictionCal().band_deg).clamp()[:4]
                              + 1.0, 0.0)
                goal = q + np.clip(last - q, -c, c)
            else:
                goal = last.copy()
            goal[4] = q[4]
            pose = {j: float(goal[i]) for i, j in enumerate(J)}
            pose[GRIPPER] = float(self.current.get(GRIPPER, obs.get("gripper.pos", 0.0)))
            self._hold_pose = pose
        pose = dict(self._hold_pose)
        for what, fn in (("roll Torque_Limit 25 %", lambda: self.set_roll_limit(0.25)),
                         ("gripper cap", lambda: grip_cap.apply_grip_cap(self.robot.bus, grip_cap.CHANGER_MAX_NM))):
            try:
                fn()
            except Exception as exc:  # noqa: BLE001
                self.note(f"hold: {what} failed ({exc!r})")
        try:
            for _ in range(max(1, int(self.cfg.hold_s * REAL_HZ))):
                super()._step(pose, "hold", "HOLD", 1.0, guarded=False)
        except Exception as exc:  # noqa: BLE001
            self.note(f"hold: command failed ({exc!r}); the servos keep their last goal")
        if self.io is not None:
            self.io.resync()

    def go_limp(self) -> None:
        self.note("LIMP: releasing torque on all joints (operator supports the arm)")
        try:
            self.robot.bus.disable_torque()
        except Exception as exc:  # noqa: BLE001
            self.note(f"disable_torque failed ({exc!r})")

    def _menu(self) -> str:
        if self.ask is None:
            return "hold"
        if self.engaged:
            prompt = ("\n*** TOOL ENGAGED WITH THE DOCK. The arm is HOLDING torque (roll 25 %, gripper capped). ***\n"
                      "Type LIMP to release torque (support the arm AND the tool first), or press ENTER to exit "
                      "with torque held: ")
        else:
            prompt = ("\n*** The arm is HOLDING torque (not engaged with a dock). ***\n"
                      "Type PARK to retreat + park, LIMP to release torque (support the arm first), or press ENTER "
                      "to exit with torque held: ")
        a = (self.prompt(prompt) or "").strip().upper()
        if a == "LIMP":
            return "limp"
        if a == "PARK" and not self.engaged:
            return "park"
        if a == "PARK":
            self.note("PARK refused: a tool is engaged with the dock")
        return "hold"

    # -- park (sweep.run's finally)
    def park(self) -> None:
        if self.limp:
            self.note("park: arm is limp by design (teach) - nothing to park")
            return
        if self.engaged or self.fault_hold or self.hands_on:
            self.hold_in_place(self.fault_reason or ("tool engaged with the dock" if self.engaged else
                                                     "hands on the tool (twist)" if self.hands_on else "fault"))
            choice = self._menu()
            if choice == "limp":
                self.go_limp()
                self.limp = True
                raise TcHold("operator released torque (LIMP)")
            if choice != "park":
                raise TcHold(f"holding torque: {self.fault_reason}")
            self.note("operator chose PARK")
            self.fault_hold = False
            self.hands_on = False
            self._holding = False
            self.stop.sigint = False
            try:
                self.stop.stop_file.unlink()
            except FileNotFoundError:
                pass
            sweep.set_body_p_gain(self.robot, sweep.PARK_P_COEFFICIENT)
        self.kind = "park"
        self.ref = None
        try:
            self.set_roll_limit(0.25)
        except Exception as exc:  # noqa: BLE001
            self.note(f"park: roll Torque_Limit write failed ({exc!r})")
        if self.grip_raw is not None:            # a rebooted gripper servo comes back at 1.47 N.m: re-cap first
            try:
                grip_cap.apply_grip_cap(self.robot.bus, grip_cap.CHANGER_MAX_NM)
            except Exception as exc:  # noqa: BLE001
                self.note(f"park: gripper cap re-apply failed ({exc!r})")
        if self.retreat is not None:
            self.retreat()
            self.kind = "park"
            self.ref = None
            sweep.set_body_p_gain(self.robot, sweep.PARK_P_COEFFICIENT)
        self.set_profile("slow")
        obs = self.robot.get_observation()
        self.home[GRIPPER] = float(obs["gripper.pos"])     # never turn the blade (a locked tool's rotor) on the way
        super().park()
        if self.roll_tl0 is not None:
            try:
                self.set_roll_limit(self.roll_tl0 / 1000.0)
            except Exception as exc:  # noqa: BLE001
                self.note(f"roll Torque_Limit restore failed ({exc!r})")

    def execute(self, segments) -> None:
        job = segments[0]
        job(self)

    # -- legs
    def leash_to(self, q5: Sequence[float], label: str) -> None:
        """Approach leg through sweep's leashed mover (slow profile, P 32), holding the blade where it is."""
        self.kind, self.ref = "leash", None
        sweep.set_body_p_gain(self.robot, sweep.PARK_P_COEFFICIENT)
        self.set_profile("slow")
        g = float(self.robot.get_observation()["gripper.pos"])
        target = {j: float(q5[i]) for i, j in enumerate(J)}
        target[GRIPPER] = g
        self.follow_bad = 0
        self.current = self._observed()
        self._move_leashed(target, label)
        self.kind = "free"
        if self.io is not None:
            self.io.resync()

    def settle(self, seconds: float, label: str = "SETTLE") -> None:
        self.kind, self.ref = "free", None
        pose = dict(self.current)
        for _ in range(max(1, int(round(seconds * REAL_HZ)))):
            self._step(pose, "dwell", label, 0.0, guarded=True)
        if self.io is not None:
            self.io.resync()


# ---------------------------------------------------------------------------------------------------- RealIO
class RealIO:
    """swap_routine.ArmIO on a TcSweep (see module doc).  q in lerobot degrees, blade in the routine's relative
    degrees (servo degrees - blade_ref)."""

    def __init__(self, tc: TcSweep, fric: SR.FrictionCal = SR.FrictionCal(), blade: Optional[BladeCal] = None) -> None:
        self.tc, self.cfg = tc, tc.cfg
        tc.io = self
        self.fric = fric
        self.blade = blade or BladeCal.from_robot(tc.robot)
        self.kin = FaceKinematics()
        self.blade_ref = 0.0
        self.routine_hz = REAL_HZ
        self.lim = Limiter(self.cfg.vmax, self.cfg.amax, 1.0 / REAL_HZ)
        self.blim = Limiter([self.cfg.blade_vmax], [self.cfg.blade_amax], 1.0 / REAL_HZ)
        self._prev_target: Optional[np.ndarray] = None
        self._prev_blade: Optional[float] = None
        self._wrote = False
        self.swapper: Optional["RealSwapper"] = None
        self.guarded = True
        self.label = "tc"
        self.protection: Optional[dict] = None
        self.protection_warnings: List[str] = []
        self.protection_notes: List[str] = []
        self._temp_prev: Optional[np.ndarray] = None
        self.max_step = np.zeros(6)
        self.dilated_ticks = 0

    # -- session setup (after connect, before any motion)
    def setup(self, need_twist: bool) -> None:
        bus = self.tc.robot.bus
        self.tc.grip_raw = tool_session.apply_session_limits(bus)      # gripper <= 0.25 N.m for the whole session
        self.tc._last_grip_verify = self.tc.clock()
        self.tc.roll_tl0 = int(bus.read("Torque_Limit", "wrist_roll", normalize=False))
        self.tc.roll_tl = self.tc.roll_tl0
        self.tc.note(f"session limits: gripper Torque_Limit {self.tc.grip_raw} "
                     f"({self.tc.grip_raw / 1000 * grip_cap.STALL_NM:.2f} N.m), roll Torque_Limit was {self.tc.roll_tl0}")
        if need_twist:
            self.check_protection()

    def check_protection(self, frac: float = 0.85, seconds: Optional[float] = None) -> List[str]:
        """Read-only protection registers on the roll servo, and whether they can cut the 85 % twist short."""
        self.protection = tool_session.protection_report(self.tc.robot.bus)
        seconds = seconds if seconds is not None else (1.5 + 0.3) / self.cfg.speed
        self.protection_warnings, self.protection_notes = protection_verdict(self.protection, frac, seconds)
        self.tc.note(f"roll protection registers: {self.protection}")
        for w in self.protection_notes:
            self.tc.note("protection note: " + w)
        for w in self.protection_warnings:
            self.tc.note("PROTECTION BLOCK: " + w)
        if self.protection_warnings and not self.cfg.ack_protection:
            raise SR.SwapError("roll protection registers block the 85 % twist (see the log); nothing moved. "
                               "Rerun with --ack-protection only if you accept a lower effective cap")
        return self.protection_warnings

    # -- ArmIO
    def read_q(self) -> np.ndarray:
        self.tc.check_stop()
        obs = self.tc.robot.get_observation()
        self._obs = obs
        q = deg5(obs)
        if not np.isfinite(q).all():
            raise ContactFault("non-finite joint reading")
        return q

    def read_blade(self) -> float:
        obs = getattr(self, "_obs", None) or self.tc.robot.get_observation()
        pct = float(obs["gripper.pos"])
        if pct <= 0.05 or pct >= 99.95:
            raise SR.SwapError(f"gripper reads {pct:.2f} % = the end of its calibrated range: reads are clamped there "
                               f"(a stall would be faked)")
        return self.blade.deg(pct) - self.blade_ref

    def read_load(self) -> np.ndarray:
        raw = self.tc.robot.bus.sync_read("Present_Load", normalize=False)
        return np.array([decode_load(raw[j]) for j in J]) / 1000.0

    def read_temp(self) -> np.ndarray:
        t = self.tc.robot.bus.sync_read("Present_Temperature", normalize=False)
        cur = np.array([float(t[m]) for m in J + [GRIPPER] if m in t])
        prev = self._temp_prev if self._temp_prev is not None and len(self._temp_prev) == len(cur) else cur
        self._temp_prev = cur
        return np.minimum(prev, cur)                     # 2 reads: one bogus 75 C read (camarm 10/04) does not stop

    def set_roll_torque(self, frac: float) -> None:
        if not 0.05 <= frac <= self.cfg.roll_torque_max + 1e-9:
            raise SR.SwapError(f"roll torque {frac} outside 0.05..{self.cfg.roll_torque_max}")
        if frac > 0.55 + 1e-9:
            if self.protection is None:
                raise SR.SwapError("roll above 55 % before protection_report was read (RealIO.setup(need_twist=True))")
        self.tc.set_roll_limit(frac)
        self.tc.kind = "twist" if frac > 0.55 + 1e-9 else ("contact" if self.tc.engaged else "free")

    def set_gripper_torque(self, frac: float) -> None:
        nm = frac * grip_cap.STALL_NM
        if grip_cap.raw_for(nm) > grip_cap.raw_for(grip_cap.CHANGER_MAX_NM):     # compare registers, not floats
            raise SR.SwapError(f"gripper {nm:.2f} N.m > changer cap {grip_cap.CHANGER_MAX_NM}")
        self.tc.grip_raw = grip_cap.apply_grip_cap(self.tc.robot.bus, nm)

    def write(self, q_goal, blade_goal) -> None:
        tc, cfg = self.tc, self.cfg
        tc.check_stop()
        q_goal = np.asarray(q_goal, float)
        b_servo = float(blade_goal) + self.blade_ref
        if not (np.isfinite(q_goal).all() and math.isfinite(b_servo)):
            raise SR.SwapError("non-finite command")
        if self._prev_target is not None:
            jump = np.abs(q_goal - self._prev_target)
            if np.any(jump > np.asarray(cfg.jump_max)):
                k = int(np.argmax(jump - np.asarray(cfg.jump_max)))
                raise SR.SwapError(f"commanded jump {jump[k]:.1f} deg on {J[k]} in one tick (> {cfg.jump_max[k]}): "
                                   f"routine bug, refusing")
            if abs(b_servo - self._prev_blade) > cfg.blade_jump_max:
                raise SR.SwapError(f"commanded blade jump {abs(b_servo - self._prev_blade):.1f} deg in one tick")
        self._prev_target, self._prev_blade = q_goal.copy(), b_servo
        if self.lim.last is None:
            self.lim.reset(np.array([float(tc.current[j]) for j in J]))
            self.blim.reset([self.blade.deg(float(tc.current[GRIPPER]))])
        prev = self.lim.last.copy()
        q = self.lim.step(q_goal)
        b = float(self.blim.step([b_servo])[0])
        self.max_step = np.maximum(self.max_step, np.append(np.abs(q - prev), 0.0))
        self._send(q, b)
        extra = 0
        while (np.any(np.abs(q_goal - self.lim.last) > cfg.catchup_deg)
               or abs(b_servo - float(self.blim.last[0])) > cfg.catchup_blade_deg):
            extra += 1
            if extra > cfg.catchup_max_s * REAL_HZ:
                raise SR.SwapError(f"command limiter still {np.max(np.abs(q_goal - self.lim.last)):.1f} deg behind "
                                   f"after {cfg.catchup_max_s:.0f} s")
            q = self.lim.step(q_goal)
            b = float(self.blim.step([b_servo])[0])
            self._send(q, b)
        self.dilated_ticks += extra
        self._wrote = True

    def _send(self, q: np.ndarray, b: float) -> None:
        tc = self.tc
        for i, j in enumerate(J):
            if abs(q[i]) > tc.limits[j]:
                raise SR.SwapError(f"{j} command {q[i]:.1f} deg beyond the limit {tc.limits[j]:.0f}")
        lo, hi = self.blade.window
        if not lo <= b <= hi:
            raise SR.SwapError(f"blade command {b:.1f} deg outside the calibrated window {lo:.0f}..{hi:.0f}")
        sw = self.swapper
        tc.ref = q - sw.I if (sw is not None and sw.in_move) else q      # the LIMITED command minus the integral
        pose = {j: float(q[i]) for i, j in enumerate(J)}
        pose[GRIPPER] = float(self.blade.pct(b))
        tc._step(pose, "move", self.label, 0.0, guarded=self.guarded)

    def tick(self, dt: float) -> None:
        self.tc.check_stop()
        n = max(1, int(round(dt * self.routine_hz)))
        start = 1 if self._wrote else 0
        self._wrote = False
        for _ in range(start, n):
            if self._prev_target is None:              # nothing written since a resync: hold sweep's last command
                q = np.array([float(self.tc.current[j]) for j in J])
                b = self.blade.deg(float(self.tc.current[GRIPPER])) - self.blade_ref
            else:
                q, b = self._prev_target, self._prev_blade - self.blade_ref
            self.write(q, b)
            self._wrote = False

    def face_offset_q(self, q_seat, dx, dz, droll, q0) -> np.ndarray:
        return self.kin.face_offset_q(q_seat, dx, dz, droll, q0)

    def resync(self) -> None:
        """Restart the limiter / jump guard from the pose sweep last commanded (after a leash leg, settle or hold)."""
        self.lim.reset(np.array([float(self.tc.current[j]) for j in J]))
        self.blim.reset([self.blade.deg(float(self.tc.current[GRIPPER]))])
        self._prev_target, self._prev_blade = None, None

    # -- blade reference
    def set_blade_ref(self, servo_deg_for_zero: float) -> float:
        """Re-reference the routine's blade coordinate (only while no tool is locked); returns the shift."""
        d = float(servo_deg_for_zero) - self.blade_ref
        self.blade_ref = float(servo_deg_for_zero)
        if self.swapper is not None:
            self.swapper.b_cmd -= d
        return d


def protection_verdict(reg: Dict[str, int], frac: float, seconds: float) -> Tuple[List[str], List[str]]:
    """STS3215 (read-only): when the load stays above Overload_Torque (%) for Protection_Time (x 10 ms) the output
    drops to Protective_Torque (%).  Returns (blocking, notes).  Blocking: Max_Torque_Limit below the twist (the
    RAM Torque_Limit cannot exceed it).  Note: a twist that SATURATES longer than Protection_Time drops to
    Protective_Torque mid-lock; the routine's lock check then fails -> hold in place (a stall cut short = less heat,
    so this is a note, and tc_twist reports the saturated time `sat_s`)."""
    block, notes = [], []
    if reg.get("Max_Torque_Limit", 1000) < frac * 1000:
        block.append(f"Max_Torque_Limit {reg['Max_Torque_Limit']} < the {frac:.0%} twist (EEPROM; RAM Torque_Limit "
                     f"cannot exceed it)")
    ot, pt = reg.get("Overload_Torque"), reg.get("Protection_Time")
    if ot is not None and pt is not None and ot < frac * 100 and pt * 0.010 < seconds:
        notes.append(f"Overload_Torque {ot} % < {frac:.0%} and Protection_Time {pt * 10} ms < the {seconds:.1f} s "
                     f"twist+hold: a twist that saturates the whole time drops to Protective_Torque "
                     f"{reg.get('Protective_Torque')} % (lock check fails -> hold)")
    return block, notes


# ---------------------------------------------------------------------------------------------------- swapper
class RealSwapper(SR.Swapper):
    """swap_routine.Swapper with (a) the engaged flag per segment for the contact rule, (b) hold_here -> the RealIO
    hold, (c) pick re-references the blade so the search starts at ENTRY_DEG, (d) goto() transfer legs."""

    #: transfer legs ending here (dock frame) run at sweep's park gain: at lerobot's P 16 the loaded shoulder lags
    #: > 12 deg on the 60 mm A -> Fu leg with a tool (twin, 10/24) and trips sweep's following-error watchdog
    FAR_DZ_MM, FAR_DX_MM = 40.0, -40.0      # A (dz 35) is NOT far: the 32 -> 16 switch happens up at F (critic S7)
    SETTLE_AFTER_GAIN_S = 1.0

    def __init__(self, io: RealIO, cfg: SR.Config, fric: SR.FrictionCal, log: Callable = print) -> None:
        self.in_move = False
        self._seg: Optional[tuple] = None
        self._dock = None
        self.p_now = sweep.TELEOP_P_COEFFICIENT
        io.swapper = self
        io.routine_hz = cfg.hz
        super().__init__(io, cfg, fric, log)

    def _path(self, dock, a, b):
        self._seg = (tuple(a), tuple(b))
        self._dock = dock
        return super()._path(dock, a, b)

    def _far(self, w) -> bool:
        return w is not None and (w[1] >= self.FAR_DZ_MM or w[0] <= self.FAR_DX_MM)

    def _gain_for(self, a, b, mode: str) -> int:
        if mode != "contact" and self._far(b) and not in_zone(a):
            return sweep.PARK_P_COEFFICIENT
        return sweep.TELEOP_P_COEFFICIENT

    def move(self, path, blade, T, mode="free", hold=0.0, roll_torque=None):
        tc = self.io.tc
        a, b = self._seg if self._seg else (None, None)
        self._seg = None
        eng = mode == "contact" or in_zone(a) or in_zone(b)
        if eng:
            tc.engaged = True
        want = self._gain_for(a, b, mode) if a is not None else self.p_now
        if want != self.p_now:
            if not sweep.set_body_p_gain(tc.robot, want):
                raise ContactFault(f"P gain write {want} failed before {mode} {a}->{b}")
            try:
                got = tc.robot.bus.sync_read("P_Coefficient", normalize=False)
                if any(int(got[m]) != want for m in BODY if m in got):
                    raise ContactFault(f"P gain reads {got}, wanted {want}")
            except ContactFault:
                raise
            except Exception as exc:  # noqa: BLE001
                raise ContactFault(f"P gain read-back failed ({exc!r})")
            self.p_now = want
            if want == sweep.TELEOP_P_COEFFICIENT and a is not None and self._dock is not None:
                # back to the sim-validated gain before anything near the dock: let the free integral re-converge
                tc.kind = "free"
                self.io.label = f"settle P{want} at {a}"
                self.in_move = True
                try:
                    super().move(super()._path(self._dock, a, a), blade, 0.0, mode="free", hold=self.SETTLE_AFTER_GAIN_S)
                finally:
                    self.in_move = False
        tc.kind = "contact" if eng else "free"
        self.io.label = f"{mode} {a}->{b}" if a is not None else mode
        self.in_move = True
        try:
            super().move(path, blade, T, mode=mode, hold=hold, roll_torque=roll_torque)
        finally:
            self.in_move = False
        if b is not None:
            tc.engaged = in_zone(b)

    def hold_here(self):
        """Swapper.run's exception path.  Engaged with a dock: hold in place (the contact rule).  Outside: nothing
        here; the exception becomes an abort and sweep.run's finally retreats + parks (the brief's 'outside contact
        the normal safe park is allowed')."""
        tc = self.io.tc
        if tc.engaged or tc.fault_hold:
            tc.hold_in_place("routine exception while engaged" if tc.engaged else (tc.fault_reason or "fault"))

    def pick(self, dock):
        # no tool on the master: put the routine's entry blade goal at ENTRY_DEG on the servo (DESIGN_v4 2c window)
        self.io.set_blade_ref(ENTRY_DEG - (dock.blade_park - self.cfg.twist_deg))
        return super().pick(dock)

    def goto(self, dock, b: Sequence[float], T: Optional[float] = None, blade: Optional[float] = None) -> None:
        """Free Cartesian transfer from wherever the face is (dock coords of the commanded pose) to b."""
        c = self.io.kin.dock_coords(dock.q_seat, self.q_cmd)
        a = (float(c[0]), float(c[2]), float(c[3]))
        dist = max(abs(b[0] - a[0]), abs(b[1] - a[1]), abs(b[2] - a[2]) / 1.5)
        if dist < 0.5:
            return
        T = T if T is not None else max(1.0, dist / 25.0)
        self.move(self._path(dock, a, b), self.b_cmd - self.blade_off if blade is None else blade, T, mode="free",
                  roll_torque=self.cfg.roll_torque_free)
