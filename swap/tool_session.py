# SPDX-License-Identifier: GPL-3.0-or-later
"""Session guards for a follower carrying the v4 tool changer (2026-10-05, arm-redteam).  No motion in here.

1. GRIPPER TORQUE CAP FOR THE WHOLE SESSION, not only during swaps.
   lerobot's SO follower configure() writes the gripper Max_Torque_Limit = 500 (50 % = 1.47 N.m) on EVERY connect.
   On the changer the gripper servo's reaction goes through the coupling: closing pushes the pins toward the stop
   block (hard direction), but OPENING against the jaw's open stop (or spreading the fingers inside something)
   pushes them in the UNLOCK direction, which only the V flanks hold: ~0.6 N.m with the r2 plungers (~10 N each
   plus ~1.5 N ramp lift, ~34 N total; v3.1 had 0.93 N.m at 3 x 14.8 + lift = 48.8 N; scaled by preload).  1.47 N.m > 0.6 N.m -> the tool
   twists out of its seat the first time a teleop/policy session opens the gripper hard.  The fin ray needs only
   1.2-2.4 N at its 5 mm squeeze (gripper/PLAN.md, cfe x1.28); 0.25 N.m gives 12.5 N per jaw (m1 z20 pinion), so
   the cap costs nothing.  Call apply_session_limits(robot.bus) right after robot.connect() in ANY script that
   runs the arm with a changer tool on it (teleop, record, eval, the swap RealIO).
   Also: calibrate each tool's gripper range so the OPEN end is >= 3 deg short of the open stop (the P loop then
   never stalls against it).

2. DOCK TEACH IS ONLY VALID FOR THE CALIBRATION IT WAS TAUGHT WITH.
   Docks are taught in encoder degrees.  A lerobot re-calibration moves every joint zero (often by degrees; 1 deg of
   shoulder_lift = ~5 mm at the dock) while the v4 capture is +-4 mm.  calibration_fingerprint() hashes the
   follower calibration file; swap_routine.save/load store and check it.

3. REAL-ARM KINEMATICS NEED THE FITTED ZERO OFFSETS.
   swap_routine works in encoder (lerobot) degrees.  The sim's face_offset_q uses the MuJoCo model, whose zero is
   the URDF zero.  A RealIO.face_offset_q MUST convert with dashboard/splat/fk.lerobot_obs_to_q /
   q_to_lerobot_action (fk.ZERO_OFFSET_RAD: wrist_flex -27.9 deg).  Using plain radians tilts every Cartesian
   approach / retreat direction by up to ~28 deg (the camarm pitch bug, fixed 10/05) and the master would drag the
   tool sideways on the 30 mm back-off.  check_kinematics_bridge() is the test RealIO must pass.
"""
from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Optional


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "gripper" / "tools"))
import grip_cap  # noqa: E402  (the single writer of the gripper cap)

CALIB = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/follower_arm.json"
#: read-only registers to check on the roll servo before the 85 % twist is ever run (critic r1: if Overload_Torque /
#: Protection_Current / Protection_Time trip during the 1.5 s twist, the servo drops to Overload_Torque mid-lock)
PROTECTION_REGS = ("Max_Torque_Limit", "Protection_Current", "Protective_Torque", "Protection_Time", "Overload_Torque",
                   "Over_Current_Protection_Time", "Minimum_Startup_Force", "CW_Dead_Zone", "CCW_Dead_Zone")


def apply_session_limits(bus: Any, nm: float = grip_cap.CHANGER_MAX_NM) -> int:
    """Gripper cap for a changer session (<= 0.25 N.m; pass grip_cap.FINRAY_STOCK_NM-style lower values for fin rays).
    Delegates to grip_cap.apply_grip_cap (RAM Torque_Limit + read-back).  Call ONCE after connect, then
    grip_cap.verify_grip_cap() every few seconds; swap_routine's per-swap 0.085 (= 0.25 N.m) is within the cap."""
    if nm > grip_cap.CHANGER_MAX_NM:
        raise ValueError(f"changer gripper cap {nm} N.m > {grip_cap.CHANGER_MAX_NM} (2x margin to the unlock hold)")
    return grip_cap.apply_grip_cap(bus, nm)


def protection_report(bus: Any, motor: str = "wrist_roll") -> dict:
    """Read-only dump of the servo protection registers (no writes)."""
    return {r: int(bus.read(r, motor, normalize=False)) for r in PROTECTION_REGS}


def calibration_fingerprint(path: Path = CALIB) -> Optional[str]:
    """sha256 over the per-joint homing offsets + ranges of the follower calibration (None if the file is missing)."""
    try:
        d = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None
    # arm joints only: a gripper re-calibration does not move the docks (critic r1).  NOT detected by any file hash:
    # a slipped horn or a re-clocked shoulder_pan -> after any crash, re-teach (or touch-check) the docks anyway.
    key = {j: [v.get("homing_offset"), v.get("range_min"), v.get("range_max")] for j, v in sorted(d.items()) if j != "gripper"}
    return hashlib.sha256(json.dumps(key, sort_keys=True).encode()).hexdigest()[:16]


def check_kinematics_bridge(face_offset_q, q_seat_deg, tol_mm: float = 0.5) -> float:
    """RealIO acceptance test (offline, no motion): ask RealIO.face_offset_q for a pure +10 mm move along the dock x
    axis and check, with fk + the fitted offsets, that the coupling moves 10 mm along the tool axis.  Returns the
    error in mm.  face_offset_q(q_seat, dx, dz, droll, q0) -> joint degrees, as in the ArmIO protocol."""
    import numpy as np
    sys.path.insert(0, str(ROOT / "dashboard"))
    from splat import fk
    J = fk.ARM_JOINTS

    def T(qd):
        return fk.fk_all(fk.lerobot_obs_to_q({f"{j}.pos": float(v) for j, v in zip(J, qd)}))["gripper_link"]
    q0 = np.asarray(q_seat_deg, float)
    q1 = np.asarray(face_offset_q(q0, 10.0, 0.0, 0.0, q0), float)
    T0, T1 = T(q0), T(q1)
    axis = -T0[:3, 2]                                   # tool axis = gripper -z (the coupling points along it)
    d = (T1[:3, 3] - T0[:3, 3]) * 1000
    return float(math.hypot(abs(float(np.dot(d, axis))) - 10.0, float(np.linalg.norm(d - np.dot(d, axis) * axis))))
