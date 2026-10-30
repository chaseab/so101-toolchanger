# SPDX-License-Identifier: GPL-3.0-or-later
"""SO-101 tool-changer swap routine (v4): hardware-agnostic state machine + calibration.

Everything the arm does during a swap lives here; it talks to the arm only through an ``ArmIO`` (positions, loads,
goals, torque limits, time).  ``SimIO`` (sim_io.py) runs it against the MuJoCo twin (toolchanger/sim) - that is how
this exact code was tested.  ``RealIO`` is deliberately NOT implemented here: on the real follower every motion must
go through dashboard/splat/sweep.py's park / torque-hold path (memory: arm-motion-safety), and nothing runs on the arm
until the lock coupon has passed (DESIGN_v4.md).  Dry-run on the real bus = read-only checks only.

Units: joint angles in degrees (encoder space, as lerobot reports them), loads as a fraction of stall, mm for the
Cartesian offsets in the DOCK frame (x = along the tool axis into the wall, z = up).

RealIO contract (2026-10-05, see tool_session.py): call tool_session.apply_session_limits(bus) after every connect
(lerobot sets the gripper to 50 % = 1.47 N.m, > the ~0.6 N.m unlock hold); face_offset_q must use
fk.lerobot_obs_to_q / q_to_lerobot_action (fitted zero offsets, wrist_flex -27.9 deg) and pass
tool_session.check_kinematics_bridge; load docks only via load(path, calibration_fingerprint()).

What makes it robust (all found in the sim, see toolchanger/sim/out and TOOLCHANGER_v4.html):
  1. Poses are TAUGHT in encoder space (tool locked + seated by hand) -> dock placement error does not matter.
  2. Encoder-servoed playback: lerobot's P-only loop stops anywhere inside a +-0.5..1.6 deg friction band and sags
     with the load; an outer integral loop on Present_Position removes both (the magnetic encoder is on the output).
  3. During contact the integral is CLAMPED to 1.3 x the joint's measured friction band (calibrate_friction): it
     cancels the friction lag (~4 mm at the coupling) but adds at most ~2 N, so the chamfers can still steer the arm.
  4. Axial overtravel (+2 mm) on insertion; roll overtravel (+4 deg) to the stop block at 85 % torque, then relax.
  5. Release press-down before unlocking; back off to the centre of the axial gap before lifting; take the tool's
     weight with a slow 3 mm lift before the real lift (the wrist droops ~2 deg when the weight transfers).
  6. Blade search at the insertion pose toward the rotor's open stop (the slot repeats every 180 deg) -> any parked
     rotor angle works; a blade sitting on the rotor top drops in and the still-pressing arm finishes the insertion.
  7. Lock verification: roll position window + seat wiggle + blade stall + tool-weight signature; one retry; on a
     second failure hold torque at a safe pose and stop (never release torque mid-air).
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field, asdict
from typing import Callable, Dict, List, Optional, Protocol, Sequence

import numpy as np

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


class ArmIO(Protocol):
    def read_q(self) -> np.ndarray: ...                       # 5 arm joints, deg (encoder)
    def read_blade(self) -> float: ...                        # gripper servo (blade) deg
    def read_load(self) -> np.ndarray: ...                    # 5 arm joints, fraction of stall (signed)
    def write(self, q_goal: np.ndarray, blade_goal: float) -> None: ...
    def set_roll_torque(self, frac: float) -> None: ...       # wrist_roll Torque_Limit (fraction of stall)
    def set_gripper_torque(self, frac: float) -> None: ...    # gripper (blade) Torque_Limit (fraction of stall)
    def read_temp(self) -> np.ndarray: ...                    # Present_Temperature, deg C
    def tick(self, dt: float) -> None: ...                    # wait dt seconds (sim: step physics)
    def face_offset_q(self, q_seat: np.ndarray, dx: float, dz: float, droll: float, q0: np.ndarray) -> np.ndarray: ...
    # ^ joint angles that put the coupling face at the taught seat + (dx, 0, dz) mm (dock frame), roll + droll deg


@dataclass
class Config:
    hz: float = 50.0
    standoff_mm: float = 30.0
    axial_over_mm: float = 2.0
    roll_over_deg: float = 4.0
    down_over_mm: float = 1.0
    twist_deg: float = 45.0
    roll_torque_twist: float = 0.85
    roll_torque_normal: float = 0.55
    ki: float = 6.0                     # outer-loop integral gain (1/s)
    free_clamp_deg: float = 6.0
    lock_window_deg: float = 2.5        # |roll - lock| after relaxing
    wiggle_deg: float = 4.0
    wiggle_slack_deg: float = 2.2       # seated: only the tool turning in the slot (+-1.3 deg at 0.4 mm/side); unseated: the full 4
    weight_load_min: float = 0.015     # |wrist_flex load| at pose F differs by >= this with vs without the tool
    retries: int = 2                    # attempt 2/3 shift the standoff -0.8 / +0.8 mm vertically (search)
    retry_dz_mm: tuple = (0.0, -0.8, 0.8)   # R3: within the pin-tip / cavity margin (23.8 vs 24.2 at 0.8)
    gripper_torque: float = 0.085       # = 0.25 N.m, the v3.1 cap (V hold >= 0.6 N.m), always set before a swap
    roll_torque_free: float = 0.25      # v3.1: low roll torque except inserting (0.55) / twisting (0.85)
    away_mm: tuple = (-40.0, 80.0)      # 10/27: away pose F (dx back, dz up), dock frame. Old (-60, 45) needs lift -115 at the taught dock (~230 mm reach); this needs lift -70.5, elbow 71
    temp_pause_c: float = 55.0          # v3.1 R2 #23


@dataclass
class DockTeach:
    """One dock, taught once (teach_dock): encoder angles with the tool locked AND seated."""
    name: str
    q_seat: List[float]                 # deg, 5 joints (roll = the lock angle)
    blade_park: float = 0.0             # blade angle when the tool was left (deg)
    load_no_tool: Optional[List[float]] = None   # |wrist_flex load| at pose F without a tool (each drop fills it)
    load_tool: Optional[float] = None            # same with the tool (each successful pick fills it)


@dataclass
class FrictionCal:
    band_deg: List[float] = field(default_factory=lambda: [0.5, 1.6, 1.4, 0.1, 0.3])   # half-band per joint

    def clamp(self) -> np.ndarray:
        c = np.maximum(1.3 * np.array(self.band_deg) + 0.3, 0.6)   # floor: the sim calibration under-reads pan
        c[4] = 0.0                       # the roll is commanded directly (overtravel), never integrated
        return c


class SwapError(RuntimeError):
    pass


class Swapper:
    def __init__(self, io: ArmIO, cfg: Config = Config(), fric: FrictionCal = FrictionCal(), log: Callable = print):
        self.io, self.cfg, self.fric, self.log = io, cfg, fric, log
        self.I = np.zeros(5)
        self.q_cmd = io.read_q()
        self.b_cmd = io.read_blade()
        self.blade_off = 0.0
        self.trace: List[dict] = []
        io.set_gripper_torque(cfg.gripper_torque)
        io.set_roll_torque(cfg.roll_torque_free)
        self._t_temp = 0.0

    def hold_here(self):
        """Failure path: keep torque ON and command the current position (never disconnect / release mid-air)."""
        q = self.io.read_q()
        self.io.write(q, self.io.read_blade())
        self.q_cmd = q

    def run(self, fn, dock):
        """Every swap goes through here: any exception -> hold torque where the arm is, report NEEDS_ATTENTION."""
        try:
            return fn(dock)
        except Exception as e:  # noqa: BLE001
            self.hold_here()
            self.log(f"NEEDS_ATTENTION: {e!r} - holding torque at the current pose")
            return {"dock": dock.name, "ok": False, "error": repr(e), "needs_attention": True}

    # ------------------------------------------------------------------ motion primitives
    def _path(self, dock: DockTeach, a: Sequence[float], b: Sequence[float]) -> List[np.ndarray]:
        """Straight Cartesian line from a=(dx,dz,droll) to b, IK every ~1 mm / 1.5 deg."""
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]), abs(b[2] - a[2]) / 1.5)) + 1
        q = self.q_cmd.copy()
        out = []
        for i in range(1, n + 1):
            s = i / n
            q = self.io.face_offset_q(np.array(dock.q_seat), a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s,
                                      a[2] + (b[2] - a[2]) * s, q)
            out.append(q)
        return out

    def move(self, path: List[np.ndarray], blade: float, T: float, mode: str = "free", hold: float = 0.0,
             roll_torque: Optional[float] = None):
        """Follow a joint path over T s (+hold).  mode: 'free' (integral, wide clamp), 'contact' (clamped integral),
        'plain' (no outer loop)."""
        io, cfg = self.io, self.cfg
        if roll_torque is not None:
            io.set_roll_torque(roll_torque)
        dt = 1.0 / cfg.hz
        q0, b0 = self.q_cmd.copy(), self.b_cmd
        pts = [q0] + list(path)
        n = max(int((T + hold) * cfg.hz), 1)
        clamp = self.fric.clamp() if mode == "contact" else np.full(5, cfg.free_clamp_deg)
        if mode == "contact":
            self.I = np.clip(self.I, -clamp, clamp)
        for i in range(n):
            s = min(i * dt / T, 1.0) if T > 0 else 1.0
            s = s * s * (3 - 2 * s)
            u = s * (len(pts) - 1)
            k = min(int(u), len(pts) - 2)
            ref = pts[k] + (pts[k + 1] - pts[k]) * (u - k)
            bref = b0 + (blade + self.blade_off - b0) * s
            qm = io.read_q()
            if mode != "plain":
                self.I += cfg.ki * (ref - qm) * dt
                self.I = np.clip(self.I, -clamp, clamp)
            io.write(ref + self.I, bref)
            io.tick(dt)
            self._t_temp += dt
            if self._t_temp >= 0.5:                   # R2 #23: pause above 55 C (here: stop + hold)
                self._t_temp = 0.0
                temp = getattr(io, "read_temp", None)
                if temp is not None and float(np.max(temp())) > cfg.temp_pause_c:
                    raise SwapError(f"servo temperature {float(np.max(temp())):.0f} C > {cfg.temp_pause_c} C")
        self.q_cmd, self.b_cmd = pts[-1].copy(), blade + self.blade_off

    # ------------------------------------------------------------------ checks
    def _wiggle_ok(self, dock: DockTeach, at) -> bool:
        """Roll +-wiggle at 15 % torque: a seated tool (pins in the steel V's, preload) barely moves."""
        q0 = self.io.read_q()[4]
        moved = 0.0
        for d in (self.cfg.wiggle_deg, -self.cfg.wiggle_deg, 0.0):
            self.move(self._path(dock, at, (at[0], at[1], at[2] + d)), 0.0, 0.3, mode="contact", roll_torque=0.15)
            moved = max(moved, abs(self.io.read_q()[4] - q0))
        self.move(self._path(dock, at, at), 0.0, 0.2, mode="contact", roll_torque=self.cfg.roll_torque_normal)
        self.log(f"  wiggle: roll moved {moved:.2f} deg (limit {self.cfg.wiggle_slack_deg})")
        return moved < self.cfg.wiggle_slack_deg

    def _blade_search(self, dock: DockTeach, at, entry_goal: float) -> bool:
        """At the INSERTION pose (critic R2: no rotor spring - the arm keeps pressing, its axial compliance is the
        spring): sweep the blade toward the rotor's open stop until it stalls.  If the blade sat on the rotor top it
        drops into the slot on the way (the master moves the last ~4 mm in); then it drives the rotor onto its open
        stop.  Stall = the goal keeps going, the position stops.  The slot repeats every 180 deg and the rotor range
        is ~144 deg, so a 260-deg sweep always ends on the stop.  Single-turn servo: mount the blade so the entry
        angle sits >= 270 deg on the 0-360 scale (the sweep must not cross 0)."""
        start = self.b_cmd
        b_prev = self.io.read_blade()
        stalled = 0
        for k in range(1, 27):
            goal = start - 10.0 * k
            self.move(self._path(dock, at, at), goal - self.blade_off, 0.15, mode="contact")
            b = self.io.read_blade()
            stalled = stalled + 1 if (abs(b - b_prev) < 1.0 and abs(goal - b) > 5.0) else 0
            b_prev = b
            if stalled >= 2:
                # rotor on its open stop (-4 deg in the tool) -> blade angle that puts the rotor at 0 = b + 4
                self.blade_off = (b + 4.0) - entry_goal
                self.move(self._path(dock, at, at), entry_goal, 0.4, mode="contact")
                self.log(f"  blade search: stalled at {b:.1f} deg -> blade offset {self.blade_off:.1f}")
                return True
        return False

    def _load_at_F(self, dock: DockTeach, frm, F) -> float:
        """Weigh at the away pose F, always arriving FROM ABOVE (same friction state every time, critic R2): read
        |wrist_flex load|.  With vs without a tool differs by the tool's weight."""
        Fu = (F[0], F[1] + 10.0, F[2])
        b = self.b_cmd - self.blade_off
        self.move(self._path(dock, frm, Fu), b, 1.5, mode="free", roll_torque=self.cfg.roll_torque_free)
        self.move(self._path(dock, Fu, F), b, 0.6, mode="free", hold=0.6)
        # wrist_flex is the scale: ~0.04 N.m friction vs the tool's ~0.1 N.m about its axis; the shoulder's 0.5-0.8 N.m
        # stiction swamps a 100 g tool (routine Monte Carlo r2: 10/33 false "no tool" with the shoulder)
        return float(abs(self.io.read_load()[3]))

    # ------------------------------------------------------------------ the swap
    def pick(self, dock: DockTeach) -> dict:
        cfg = self.cfg
        self.io.set_gripper_torque(cfg.gripper_torque)          # every swap (a tool session may have raised it)
        self.io.set_roll_torque(cfg.roll_torque_free)
        entry = -0.3
        A3, A = (0, 3, 0), (0, 35, 0)
        F = (*cfg.away_mm, cfg.twist_deg)
        res = {"dock": dock.name, "attempts": []}
        if dock.load_no_tool is None:           # R3 F6: pick starts tool-free at F -> take the no-tool baseline now
            dock.load_no_tool = [self._load_at_F(dock, F, (*cfg.away_mm, 0))]
        for attempt in range(cfg.retries + 1):
            # r3: a spigot that lands >~3 mm off (dock moved up + stiction) can hook beside a window: retries search
            # +-1.5 mm vertically; the lock itself is located by the steel V's, so only the approach is offset
            dz = cfg.retry_dz_mm[min(attempt, len(cfg.retry_dz_mm) - 1)]
            E = (-cfg.standoff_mm, dz, cfg.twist_deg)
            C = (entry + cfg.axial_over_mm, dz, cfg.twist_deg)
            L, Lr, Lc = (C[0], dz, -cfg.roll_over_deg), (C[0], dz, 0), (0, 0, 0)
            att = {"dz_offset": dz}
            self.blade_off = 0.0                      # re-found by the blade search each attempt
            self.log(f"PICK {dock.name} attempt {attempt + 1}")
            # 2026-10-24 (RealIO jump guard): the arm is at F with roll 0 here (the weigh pose); a path starting at
            # F with roll +45 made the first 1/31 of the move a 45 deg roll snap (~1 tick).  Start where the arm is.
            self.move(self._path(dock, (F[0], F[1], 0.0) if attempt == 0 else E, E), dock.blade_park - cfg.twist_deg, 1.5,
                      mode="free")
            # R3 (= sim r3): settle until the encoder servo converged (|err| < 0.15 deg on the 4 arm joints) or 2 s
            qE = self._path(dock, E, E)[-1]
            for _ in range(10):
                self.move(self._path(dock, E, E), dock.blade_park - cfg.twist_deg, 0.0, mode="free", hold=0.2)
                if np.max(np.abs(self.io.read_q() - qE)[:4]) < 0.15:
                    break
            self.move(self._path(dock, E, C), dock.blade_park - cfg.twist_deg, 2.0, mode="contact",
                      roll_torque=cfg.roll_torque_normal)
            self.move(self._path(dock, C, C), dock.blade_park - cfg.twist_deg, 0.0, mode="contact", hold=0.3)
            att["blade_ok"] = self._blade_search(dock, C, dock.blade_park - cfg.twist_deg)
            if not att["blade_ok"]:
                att["fail"] = "blade never stalled (no rotor found)"
                res["attempts"].append(att)
                self.log("  blade search failed -> back off, retry")
                self.move(self._path(dock, C, E), dock.blade_park - cfg.twist_deg, 2.0, mode="contact")
                continue
            self.move(self._path(dock, C, L), dock.blade_park, 1.5, mode="contact", roll_torque=cfg.roll_torque_twist)
            self.move(self._path(dock, L, L), dock.blade_park, 0.0, mode="contact", hold=0.3)
            self.move(self._path(dock, L, Lr), dock.blade_park, 0.3, mode="contact", roll_torque=0.25)
            roll_err = self.io.read_q()[4] - dock.q_seat[4]
            att["roll_err_deg"] = round(float(roll_err), 2)
            self.move(self._path(dock, Lr, Lc), dock.blade_park, 0.4, mode="free", roll_torque=cfg.roll_torque_normal)
            att["wiggle_ok"] = self._wiggle_ok(dock, Lc)
            locked = abs(roll_err) < cfg.lock_window_deg and att["wiggle_ok"]
            if locked and att["blade_ok"]:
                self.move(self._path(dock, Lc, A3), 0.0, 1.0, mode="free")
                self.move(self._path(dock, A3, A3), 0.0, 0.0, mode="free", hold=0.4)
                self.move(self._path(dock, A3, A), 0.0, 1.5, mode="free")
                ld = self._load_at_F(dock, A, (*cfg.away_mm, 0))
                att["load_F"] = round(ld, 3)
                if dock.load_no_tool is not None and abs(ld - dock.load_no_tool[0]) < cfg.weight_load_min:
                    att["fail"] = "weight check at F: no tool on the master"
                    res["attempts"].append(att)
                    res["ok"] = False
                    self.log("PICK: no tool on the master after the lift -> NEEDS_ATTENTION (look at the dock)")
                    return res
                dock.load_tool = ld
                att["ok"] = True
                res["attempts"].append(att)
                res["ok"] = True
                return res
            else:
                att["fail"] = "lock check failed" if not locked else "blade not engaged"
            res["attempts"].append(att)
            self.log(f"  attempt failed: {att['fail']} -> unlock, back off, retry")
            self.move(self._path(dock, Lc, (0, 0, cfg.twist_deg + cfg.roll_over_deg)), dock.blade_park - cfg.twist_deg, 1.5,
                      mode="contact", roll_torque=cfg.roll_torque_twist)
            self.move(self._path(dock, (0, 0, cfg.twist_deg + cfg.roll_over_deg), E), dock.blade_park - cfg.twist_deg, 2.0,
                      mode="contact", roll_torque=cfg.roll_torque_normal)
        res["ok"] = False
        self.log("PICK FAILED twice: holding torque at the standoff, NEEDS_ATTENTION")
        return res

    def drop(self, dock: DockTeach) -> dict:
        cfg = self.cfg
        self.io.set_gripper_torque(cfg.gripper_torque)
        self.io.set_roll_torque(cfg.roll_torque_free)
        A, Cd, C0 = (0, 35, 0), (0, -cfg.down_over_mm, 0), (0, 0, 0)
        D = (0, 0, cfg.twist_deg + cfg.roll_over_deg)
        E, F = (-cfg.standoff_mm, 0, cfg.twist_deg), (*cfg.away_mm, cfg.twist_deg)
        self.log(f"DROP {dock.name}")
        if dock.load_tool is None:              # R3 F6: no with-tool baseline yet -> weigh at F before the drop
            dock.load_tool = self._load_at_F(dock, A, (*cfg.away_mm, 0))
            self.move(self._path(dock, (*cfg.away_mm, 0), A), 0.0, 1.5, mode="free")
        self.move(self._path(dock, A, A), 0.0, 0.0, mode="free", hold=0.6, roll_torque=cfg.roll_torque_free)
        self.move(self._path(dock, A, Cd), 0.0, 1.8, mode="contact")
        self.move(self._path(dock, Cd, Cd), 0.0, 0.0, mode="contact", hold=0.3)
        self.move(self._path(dock, Cd, C0), 0.0, 0.4, mode="contact")
        self.move(self._path(dock, C0, C0), 0.0, 0.0, mode="contact", hold=0.3)
        self.move(self._path(dock, C0, D), -cfg.twist_deg, 1.5, mode="contact", roll_torque=cfg.roll_torque_twist)
        self.move(self._path(dock, D, D), -cfg.twist_deg, 0.0, mode="contact", hold=0.3)
        unlocked = abs(self.io.read_q()[4] - (dock.q_seat[4] + cfg.twist_deg + cfg.roll_over_deg)) < 3.0
        self.move(self._path(dock, D, E), -cfg.twist_deg, 2.0, mode="contact", roll_torque=cfg.roll_torque_normal)
        dock.blade_park = self.b_cmd + cfg.twist_deg                       # blade angle that matches the parked rotor
        ld = self._load_at_F(dock, E, (*cfg.away_mm, 0))
        left = True if dock.load_tool is None else abs(ld - dock.load_tool) >= cfg.weight_load_min
        dock.load_no_tool = [ld]
        if not (unlocked and left):
            self.log("DROP: roll not reached or the tool is still on the master -> NEEDS_ATTENTION")
        return {"dock": dock.name, "ok": bool(unlocked and left), "roll_reached": bool(unlocked), "tool_left": bool(left),
                "load_F": round(ld, 3)}


# ------------------------------------------------------------------ calibration (one time per robot / per dock)
def calibrate_friction(io: ArmIO, q_mid: np.ndarray, amp_deg: float = 8.0, period_s: float = 6.0,
                       hz: float = 50.0, joints: Sequence[int] = range(5)) -> FrictionCal:
    """Slow triangle wave per joint (+-8 deg) around a free-space pose with plain P playback.  On the middle half of
    each constant-speed ramp the encoder lags the command by gravity sag + friction band; (lag_up - lag_down) / 2 is
    the band.  (+-3 deg read high-friction joints 4x low: they never left stiction.)  ~1 min, no contact.
    hz: control rate (the real arm runs 25 Hz, sweep.HZ).  joints: which joints to measure; the others keep the
    FrictionCal defaults (the roll band is never used: clamp()[4] = 0)."""
    bands = list(FrictionCal().band_deg)
    n = int(period_s * hz)
    dt = 1.0 / hz
    for j in joints:
        lag_up, lag_dn = [], []
        for i in range(2 * n):
            # phase starts at 0.25 (tri = 0): the wave starts AND ends at q_mid, never an 8 deg step (2026-10-24,
            # the old phase opened with a jump to q_mid - amp, fine in the sim, a 1-tick 8 deg step on the arm)
            ph = ((i % n) / n + 0.25) % 1.0
            tri = 4 * ph - 1 if ph < 0.5 else 3 - 4 * ph
            q = q_mid.copy()
            q[j] += amp_deg * tri
            io.write(q, io.read_blade())
            io.tick(dt)
            if i >= n:                                   # second period only (settled)
                lag = q[j] - io.read_q()[j]
                if 0.125 <= ph < 0.375:
                    lag_up.append(lag)
                elif 0.625 <= ph < 0.875:
                    lag_dn.append(lag)
        bands[j] = round(float(abs(np.median(lag_up) - np.median(lag_dn)) / 2), 3)
        io.write(q_mid, io.read_blade())
        io.tick(0.5)
    return FrictionCal(band_deg=bands)


def teach_dock(io: ArmIO, name: str, confirm: Callable[[str], None]) -> DockTeach:
    """With the tool LOCKED on the master: owner hand-guides the arm until the tool sits in the dock (neck on the
    land, body on the tail rest).  The arm joints must be limp while he does it -> on the real arm this runs only
    through sweep.py's torque-hold path with owner holding the arm.  Records the encoder angles."""
    confirm(f"Seat the tool in dock '{name}' by hand, then confirm")
    q = io.read_q()
    return DockTeach(name=name, q_seat=[round(float(v), 3) for v in q], blade_park=float(io.read_blade()))


def save(path, docks: Dict[str, DockTeach], fric: FrictionCal, calibration: Optional[str] = None):
    """calibration = tool_session.calibration_fingerprint() of the follower calibration the docks were taught with."""
    with open(path, "w") as f:
        json.dump({"docks": {k: asdict(v) for k, v in docks.items()}, "friction": asdict(fric),
                   "calibration": calibration}, f, indent=1)


def load(path, calibration: Optional[str]):
    """Docks + friction, ONLY if they were taught with the same follower calibration (2026-10-05, arm-redteam): a
    lerobot re-calibration moves the joint zeros by degrees (~5 mm at the dock per deg of lift) vs the +-4 mm capture,
    so stale teach data must fail loudly here, not as a jam at the dock.  Re-teach after any re-calibration, horn
    re-mount or crash."""
    with open(path) as f:
        d = json.load(f)
    if not calibration or d.get("calibration") != calibration:
        raise SwapError(f"dock teach file {path} was taught with calibration {d.get('calibration')!r}, the arm now has "
                        f"{calibration!r}: re-teach the docks (teach_dock) and re-run calibrate_friction")
    docks = {k: DockTeach(**v) for k, v in d["docks"].items()}
    for k, v in docks.items():                     # the unlock twist goes +49 deg past the lock roll; the follower
        if not -150.0 <= v.q_seat[4] <= 100.0:     # roll range is one turn (0..4095) -> keep the swap off the wrap
            raise SwapError(f"dock {k}: lock roll {v.q_seat[4]:.1f} deg leaves no room for the +49 deg twist")
    return docks, FrictionCal(**d["friction"])
