# SPDX-License-Identifier: GPL-3.0-or-later
"""Fake lerobot follower + Feetech bus for offline RealIO tests and `run_swap.py --dry-run` (no serial port, ever).

FakeRobot looks like lerobot's SO101Follower (use_degrees: body joints in degrees, gripper 0..100 of its calibrated
range, max_relative_target clamp relative to Present_Position) and FakeBus like its FeetechMotorsBus (read / write /
sync_read / sync_write with normalize=False, enable/disable_torque, disconnect(disable_torque)).  Two physics
backends:

* KinematicArm - first-order servos with a friction band and a load model; fast, for unit tests, twist / teach /
  friction dry runs.
* SimArm - the MuJoCo twin (toolchanger/sim/tcsim.py via swap/sim_io.SimIO): the real swap physics (dock, tool,
  plungers, magnets).  Reports TRUE lerobot degrees (MJCF radians minus fk.ZERO_OFFSET_RAD: the MuJoCo gripper body
  and fk's gripper_link agree to 0.01 mm), so RealIO's fk-based IK is tested against independent physics.

Fault injection: FakeBus.fail_when(pred) raises "[TxRxResult] There is no status packet!" on matching calls;
backend.hooks run every tick (jam a joint, reboot the gripper, ...); FakeBus.raw_load returns undecoded
sign-magnitude Present_Load.  Time: robot.clock() is simulated time; robot.sleep(dt) advances the physics, but only
from the thread that created the robot (sweep's reporter thread also calls sleep).
"""
from __future__ import annotations

import math
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

import real_io as R
from splat import fk

J = fk.ARM_JOINTS
ALL = J + ["gripper"]
STALL_NM = 2.94


class Cal(SimpleNamespace):
    pass


def calibration_from(path_or_dict) -> Dict[str, Cal]:
    import json
    d = path_or_dict if isinstance(path_or_dict, dict) else json.loads(Path(path_or_dict).read_text())
    return {m: Cal(id=v["id"], drive_mode=v.get("drive_mode", 0), homing_offset=v.get("homing_offset", 0),
                   range_min=v["range_min"], range_max=v["range_max"]) for m, v in d.items()}


#: follower_arm.json arm joints as on 2026-10-24 + a wide gripper range (what gripcal.py makes for follower_arm_tc)
DEFAULT_CAL = {
    "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": -1942, "range_min": 802, "range_max": 3266},
    "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": -1019, "range_min": 891, "range_max": 3145},
    "elbow_flex": {"id": 3, "drive_mode": 0, "homing_offset": 1912, "range_min": 893, "range_max": 3128},
    "wrist_flex": {"id": 4, "drive_mode": 0, "homing_offset": -1911, "range_min": 716, "range_max": 2729},
    "wrist_roll": {"id": 5, "drive_mode": 0, "homing_offset": 1747, "range_min": 0, "range_max": 4095},
    "gripper": {"id": 6, "drive_mode": 0, "homing_offset": -1911, "range_min": 57, "range_max": 4039},
}
STOCK_GRIPPER = {"id": 6, "drive_mode": 0, "homing_offset": -1911, "range_min": 1270, "range_max": 2667}

REG_DEFAULTS = {
    "Torque_Limit": 1000, "Max_Torque_Limit": 1000, "Protection_Current": 500, "Protective_Torque": 20,
    "Protection_Time": 200, "Overload_Torque": 80, "Over_Current_Protection_Time": 200, "Minimum_Startup_Force": 16,
    "CW_Dead_Zone": 0, "CCW_Dead_Zone": 0, "P_Coefficient": 16, "I_Coefficient": 0, "D_Coefficient": 32,
    "Goal_Velocity": 300, "Acceleration": 20, "Torque_Enable": 1, "Lock": 1,
}


class BusError(ConnectionError):
    pass


class FakeBus:
    def __init__(self, arm: "ArmBackend", calibration: Dict[str, Cal]) -> None:
        self.arm = arm
        self.calibration = calibration
        self.motors = {m: SimpleNamespace(id=calibration[m].id) for m in ALL}
        self.regs: Dict[str, Dict[str, int]] = {r: {m: v for m in ALL} for r, v in REG_DEFAULTS.items()}
        self.regs["Max_Torque_Limit"]["gripper"] = 500          # lerobot configure()
        self.regs["Torque_Limit"]["gripper"] = 500
        self.regs["Overload_Torque"]["gripper"] = 25
        self.regs["Protection_Current"]["gripper"] = 250
        self.raw_load = False
        self._fail: Optional[Callable[[str, str], bool]] = None
        self.ops: List[tuple] = []                 # (t, op, reg, values) of every write
        self.connected = True
        arm.bus = self
        for m in ALL:
            arm.set_torque_limit(m, self.regs["Torque_Limit"][m] / 1000.0)

    # -- faults
    def fail_when(self, pred: Optional[Callable[[str, str], bool]]) -> None:
        """pred(op, reg) -> True = this call fails (op in read/write)."""
        self._fail = pred

    def _check(self, op: str, reg: str) -> None:
        if not self.connected:
            raise BusError("bus not connected")
        if self._fail is not None and self._fail(op, reg):
            raise BusError("[TxRxResult] There is no status packet!")

    # -- normalisation (lerobot: DEGREES for the body with use_degrees, RANGE_0_100 for the gripper)
    def _pct(self, servo_deg: float) -> float:
        c = self.calibration["gripper"]
        tick = min(c.range_max, max(c.range_min, servo_deg * 4096 / 360))
        p = (tick - c.range_min) / (c.range_max - c.range_min) * 100.0
        return 100 - p if c.drive_mode else p

    def _servo_deg(self, pct: float) -> float:
        c = self.calibration["gripper"]
        p = min(100.0, max(0.0, 100 - pct if c.drive_mode else pct))
        return int(p / 100 * (c.range_max - c.range_min) + c.range_min) * 360 / 4096

    # -- lerobot bus API
    def sync_read(self, reg: str, motors=None, normalize: bool = True) -> Dict[str, Any]:
        self._check("read", reg)
        ms = ALL if motors is None else ([motors] if isinstance(motors, str) else list(motors))
        if reg == "Present_Position":
            pos = self.arm.positions()
            return {m: (self._pct(pos[m]) if m == "gripper" else pos[m]) for m in ms}
        if reg == "Present_Load":
            ld = self.arm.loads()
            out = {}
            for m in ms:
                v = int(round(max(-1.0, min(1.0, ld[m])) * 1000))
                out[m] = ((abs(v) | 0x400) if v < 0 else v) if self.raw_load else v
            return out
        if reg == "Present_Temperature":
            t = self.arm.temps()
            return {m: int(round(t[m])) for m in ms}
        return {m: self.regs[reg][m] for m in ms}

    def read(self, reg: str, motor: str, normalize: bool = True):
        return self.sync_read(reg, [motor], normalize)[motor]

    def sync_write(self, reg: str, values: Dict[str, Any], normalize: bool = True) -> None:
        self._check("write", reg)
        self.ops.append((self.arm.t, "write", reg, dict(values)))
        if reg == "Goal_Position":
            for m, v in values.items():
                self.arm.set_goal(m, self._servo_deg(float(v)) if m == "gripper" else float(v))
            return
        for m, v in values.items():
            self.regs.setdefault(reg, {})[m] = int(v)
            if reg == "Torque_Limit":
                self.arm.set_torque_limit(m, int(v) / 1000.0)
            elif reg == "P_Coefficient":
                self.arm.set_p(m, int(v))
            elif reg == "Torque_Enable":
                self.arm.set_torque(m, bool(v))

    def write(self, reg: str, motor: str, value, normalize: bool = True) -> None:
        self.sync_write(reg, {motor: value}, normalize)

    def disable_torque(self, motors=None, num_retry: int = 0) -> None:
        ms = ALL if motors is None else ([motors] if isinstance(motors, str) else list(motors))
        self.sync_write("Torque_Enable", {m: 0 for m in ms})

    def enable_torque(self, motors=None, num_retry: int = 0) -> None:
        ms = ALL if motors is None else ([motors] if isinstance(motors, str) else list(motors))
        self.sync_write("Torque_Enable", {m: 1 for m in ms})

    def disconnect(self, disable_torque: bool = True) -> None:
        self.ops.append((self.arm.t, "disconnect", "", {"disable_torque": bool(disable_torque)}))
        if disable_torque and self.connected:
            self.disable_torque()
        self.connected = False

    # -- helpers for tests
    def torque_on(self, m: str) -> bool:
        return bool(self.regs["Torque_Enable"][m])

    def reboot(self, m: str) -> None:
        """Brown-out: RAM registers back to EEPROM (Torque_Limit = Max_Torque_Limit), torque stays on."""
        self.regs["Torque_Limit"][m] = self.regs["Max_Torque_Limit"][m]
        self.arm.set_torque_limit(m, self.regs["Torque_Limit"][m] / 1000.0)


class FakeRobot:
    """lerobot SO101Follower stand-in (use_degrees=True)."""

    def __init__(self, arm: "ArmBackend", calibration=None, max_relative_target: float = 8.0) -> None:
        self.arm = arm
        self.bus = FakeBus(arm, calibration_from(calibration or DEFAULT_CAL))
        self.max_relative_target = max_relative_target
        self.is_calibrated = True
        self.calibration = self.bus.calibration
        self.motors = list(ALL)
        self._thread = threading.get_ident()
        self.sent: List[tuple] = []                 # (t, action) sent

    def get_observation(self) -> Dict[str, float]:
        return {f"{m}.pos": float(v) for m, v in self.bus.sync_read("Present_Position").items()}

    def send_action(self, action: Dict[str, float]) -> Dict[str, float]:
        goal = {k[:-4]: float(v) for k, v in action.items() if k.endswith(".pos")}
        if self.max_relative_target is not None:
            pres = self.bus.sync_read("Present_Position")
            goal = {m: pres[m] + max(-self.max_relative_target, min(self.max_relative_target, g - pres[m]))
                    for m, g in goal.items()}
        self.bus.sync_write("Goal_Position", goal)
        out = {f"{m}.pos": v for m, v in goal.items()}
        self.sent.append((self.arm.t, out))
        return out

    def disconnect(self) -> None:
        self.bus.disconnect(True)

    def clock(self) -> float:
        return self.arm.t

    def sleep(self, dt: float) -> None:
        if threading.get_ident() != self._thread or dt <= 0:
            return                                   # sweep's reporter thread: never step physics from it
        self.arm.advance(dt)


# ---------------------------------------------------------------------------------------------------- backends
class ArmBackend:
    t = 0.0
    bus: FakeBus
    hooks: List[Callable[["ArmBackend"], None]]

    def positions(self) -> Dict[str, float]: ...
    def loads(self) -> Dict[str, float]: ...
    def temps(self) -> Dict[str, float]: ...
    def set_goal(self, m: str, v: float) -> None: ...
    def set_torque_limit(self, m: str, frac: float) -> None: ...
    def set_p(self, m: str, p: int) -> None: ...
    def set_torque(self, m: str, on: bool) -> None: ...
    def advance(self, dt: float) -> None: ...


class KinematicArm(ArmBackend):
    """Each joint: servo spring K (P-scaled) with torque limit, gravity load on lift/elbow/wrist_flex, Coulomb
    friction band; first-order motion (tau 40 ms, vmax 200 deg/s).  Enough to exercise every RealIO path quickly."""

    BAND = dict(shoulder_pan=0.5, shoulder_lift=1.6, elbow_flex=1.4, wrist_flex=0.1, wrist_roll=0.3, gripper=0.2)
    GRAV = dict(shoulder_pan=0.0, shoulder_lift=0.9, elbow_flex=0.5, wrist_flex=0.08, wrist_roll=0.0, gripper=0.0)

    def __init__(self, start: Dict[str, float]) -> None:
        self.t = 0.0
        self.pos = {m: float(start[m]) for m in ALL}
        self.goal = dict(self.pos)
        self.limit = {m: 1.0 for m in ALL}
        self.p = {m: 16 for m in ALL}
        self.torque = {m: True for m in ALL}
        self.jam: Dict[str, float] = {}             # joint -> external torque (N.m) opposing motion (a block)
        self.hooks: List[Callable] = []
        self.temp = {m: 35.0 for m in ALL}
        self._load = {m: 0.0 for m in ALL}

    def positions(self):
        return dict(self.pos)

    def loads(self):
        return dict(self._load)

    def temps(self):
        return dict(self.temp)

    def set_goal(self, m, v):
        self.goal[m] = float(v)

    def set_torque_limit(self, m, frac):
        self.limit[m] = frac

    def set_p(self, m, p):
        self.p[m] = p

    def set_torque(self, m, on):
        self.torque[m] = on

    def advance(self, dt):
        n = max(1, int(round(dt / 0.005)))
        h = dt / n
        for _ in range(n):
            for hk in list(self.hooks):
                hk(self)
            for m in ALL:
                if not self.torque[m]:
                    self._load[m] = 0.0
                    continue
                K = 22.0 * self.p[m] / 16 * math.pi / 180          # N.m per deg
                tq = K * (self.goal[m] - self.pos[m])
                tq_lim = self.limit[m] * STALL_NM
                tq = max(-tq_lim, min(tq_lim, tq))
                self._load[m] = tq / STALL_NM
                net = tq - self.GRAV[m] * math.cos(math.radians(self.pos[m]))
                blk = self.jam.get(m, 0.0)
                fr = self.BAND[m] * K + blk
                if abs(net) <= fr:
                    continue
                v_eq = (abs(net) - fr) / K * math.copysign(1.0, net)    # deg of "free" deflection to close
                step = v_eq * (1 - math.exp(-h / 0.04))
                step = max(-200 * h, min(200 * h, step))
                self.pos[m] += step
            self.t += h


class SimArm(ArmBackend):
    """The MuJoCo twin (swap/sim_io.py), seen through lerobot units."""

    BLADE_SERVO_ZERO = 345.0     # servo deg of the sim blade 0 (tool locked): entry -45 -> 300 = real_io.ENTRY_DEG

    def __init__(self, seed: int = 5000, start="A", design: str = "v4", tool: bool = True) -> None:
        import sim_io as SI
        import tcsim as T
        import mc
        self.SI, self.T = SI, T
        err = mc.sample(seed, design)
        self.S = T.make(design, err)
        self.io = SI.SimIO(self.S)
        self.err = err
        if start == "A":
            SI.start_locked_at_A(self.S, self.io)
        elif start is not None:                     # lerobot deg5: tool locked on the master at a free pose
            self._start_locked_at(self.lerobot_to_mj(start), tool=tool)
        self.m, self.d = self.S["m"], self.S["d"]
        self.t = 0.0
        self.hooks: List[Callable] = []
        self.temp = {m: 35.0 for m in ALL}
        self.kp0 = np.array([float(self.m.actuator_gainprm[a, 0]) for a in self.io.R.act])
        self.torque = {m: True for m in ALL}
        self.limit = {m: 1.0 for m in ALL}
        self.q_seat_mj = self.S["q_seat"] / SI.DEG                # MJCF deg

    def _start_locked_at(self, q_mj_deg, tool: bool = True) -> None:
        """sim_io.start_locked_at_A at any arm pose (the tool parked in the dock instead when tool=False)."""
        import mujoco
        S, T, io = self.S, self.T, self.io
        m, d, p = S["m"], S["d"], S["p"]
        mujoco.mj_resetData(m, d)
        T.place_tool_in_dock(S)
        d.qpos[io.R.qadr] = np.radians(q_mj_deg)
        d.ctrl[io.R.act] = np.radians(q_mj_deg)
        mujoco.mj_kinematics(m, d)
        if tool:
            g = m.body("gripper").id
            Rg, pg = d.xmat[g].reshape(3, 3), d.xpos[g]
            ja = m.jnt_qposadr[m.joint("tool_free").id]
            d.qpos[ja:ja + 3] = pg + Rg @ np.array([0, 0, p["Z_TOOL_FACE_LOCK"] * 1e-3])
            d.qpos[ja + 3:ja + 7] = T.quat_from_R(Rg)
            for i in range(3):
                d.qpos[m.jnt_qposadr[m.joint(f"pl{i}").id]] = (p["Z_TOOL_FACE_LOCK"] - p["Z_TOOL_FACE_ENTRY"]) * 1e-3
        io.tick(0.5)

    # unit conversions: lerobot deg = MJCF rad - fitted zero offset
    @staticmethod
    def mj_to_lerobot(mj_deg5: Sequence[float]) -> np.ndarray:
        return np.array([math.degrees((math.radians(float(v)) - fk.ZERO_OFFSET_RAD.get(j, 0.0)) / fk.DIRECTION[j])
                         for j, v in zip(J, mj_deg5)])

    @staticmethod
    def lerobot_to_mj(d5: Sequence[float]) -> np.ndarray:
        return np.array([math.degrees(fk.DIRECTION[j] * math.radians(float(v)) + fk.ZERO_OFFSET_RAD.get(j, 0.0))
                         for j, v in zip(J, d5)])

    def q_seat(self) -> List[float]:
        return [float(v) for v in self.mj_to_lerobot(self.q_seat_mj)]

    def positions(self):
        q = self.mj_to_lerobot(self.io.read_q())
        out = {j: float(q[i]) for i, j in enumerate(J)}
        out["gripper"] = self.io.read_blade() + self.BLADE_SERVO_ZERO
        return out

    def loads(self):
        f = self.d.actuator_force
        out = {j: float(f[self.io.R.act[i]]) / STALL_NM for i, j in enumerate(J)}
        out["gripper"] = float(f[self.io.R.ab]) / STALL_NM
        return out

    def temps(self):
        return dict(self.temp)

    def set_goal(self, m, v):
        if m == "gripper":
            self.d.ctrl[self.io.R.ab] = math.radians(v - self.BLADE_SERVO_ZERO)
        else:
            i = J.index(m)
            self.d.ctrl[self.io.R.act[i]] = fk.DIRECTION[m] * math.radians(v) + fk.ZERO_OFFSET_RAD.get(m, 0.0)

    def _apply_range(self, m):
        frac = self.limit[m] if self.torque[m] else 0.0
        a = self.io.R.ab if m == "gripper" else self.io.R.act[J.index(m)]
        self.m.actuator_forcerange[a] = [-frac * STALL_NM, frac * STALL_NM]

    def set_torque_limit(self, m, frac):
        self.limit[m] = frac
        self._apply_range(m)

    def set_torque(self, m, on):
        self.torque[m] = on
        self._apply_range(m)

    def set_p(self, m, p):
        if m == "gripper":
            return
        i = J.index(m)
        a = self.io.R.act[i]
        kp = self.kp0[i] * p / 16.0
        self.m.actuator_gainprm[a, 0] = kp
        self.m.actuator_biasprm[a, 1] = -kp

    def advance(self, dt):
        for hk in list(self.hooks):
            hk(self)
        self.io.tick(dt)
        self.t += dt

    # truth for tests
    def tool_state(self) -> dict:
        R_ = self.io.R
        return {"rel": R_.rel_tool(), "dock": R_.tool_in_dock()}

    def jam_joint(self, name: str, torque_nm: float) -> None:
        """Hard friction on a joint (a block / stall): MuJoCo frictionloss."""
        dof = self.m.jnt_dofadr[self.m.joint(name).id]
        self.m.dof_frictionloss[dof] = torque_nm


def sim_robot(seed: int = 5000) -> FakeRobot:
    return FakeRobot(SimArm(seed=seed))


def kinematic_robot(start: Optional[Dict[str, float]] = None, gripper_cal: Optional[dict] = None) -> FakeRobot:
    from splat import sweep
    s = dict(sweep.REST_EXPECTED)
    s["gripper"] = 200.0                                      # servo deg (inside the wide range)
    s.update(start or {})
    cal = dict(DEFAULT_CAL)
    if gripper_cal is not None:
        cal["gripper"] = gripper_cal
    return FakeRobot(KinematicArm(s), cal)
