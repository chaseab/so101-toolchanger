# SPDX-License-Identifier: GPL-3.0-or-later
"""THE gripper torque cap (single writer for the stock fin ray gripper AND the tool changer).  No motion in here.
2026-10-05, arm-redteam (critic round 1: one function, RAM register only, verify during the session).

Why a cap at all: lerobot's SO follower configure() sets the gripper Max_Torque_Limit to 500 (50 % = 1.47 N.m) on
every connect, and teleop / policies close the gripper until it stalls (stock jaws "stall 2-5 % short of the command,
10-15 N", hotswap/CONCEPT.md).
* Fin ray on the STOCK gripper: the v3 fin ray needs 1.2-2.4 N for its 3-5 mm design squeeze (8.3 % strain at 5 mm,
  > 10 % past ~6 mm, PLAN.md).  1.47 N.m at a ~55 mm contact radius = ~25 N -> every full close over-crushes it.
  FINRAY_STOCK_NM 0.15 -> ~2.7 N.  Bench test F2 (redteam/ARM_REDTEAM.html) checks it still closes reliably at
  5 % (gear friction / Minimum_Startup_Force) and holds a Jenga block; change the value HERE only.
* Tool changer: opening against the jaw stop twists the coupling in the UNLOCK direction (V-flank hold ~0.6 N.m at the
  r2 plungers).  CHANGER_MAX_NM 0.25 (= swap_routine Config.gripper_torque 0.085) keeps a 2x margin.  NOTE: the
  changer gripper tool's m1 z20 pinion (r 10 mm) turns 0.15 N.m into ~7.5 N PER JAW, 3x what a fin ray wants; its
  body (still concept) must gear down (pitch r ~25 mm) or use rigid pads - see the red-team page.

Register: only the RAM Torque_Limit (addr 48).  Max_Torque_Limit is EEPROM and lerobot's enable_torque sets
Lock=1, so writing it mid-session is unreliable; RAM is enough because a servo that reboots (brown-out) comes back
with RAM = Max_Torque_Limit -> call verify_grip_cap() every few seconds in the control loop and stop if it fails.

Use, once, right after robot.connect():   apply_grip_cap(robot.bus, FINRAY_STOCK_NM)   (or CHANGER_MAX_NM)
"""
from __future__ import annotations

from typing import Any

STALL_NM = 30 * 0.0980665          # STS3215 12 V
FINRAY_STOCK_NM = 0.15
CHANGER_MAX_NM = 0.25
UNLOCK_HOLD_NM_EST = 0.6           # changer V-flank hold, unlock direction (estimate; coupon C1 measures it)


def raw_for(nm: float) -> int:
    return int(round(1000 * nm / STALL_NM))


def apply_grip_cap(bus: Any, nm: float = FINRAY_STOCK_NM, motor: str = "gripper") -> int:
    """Write the RAM Torque_Limit and read it back; returns the raw value (0..1000 = 0..100 % of stall)."""
    raw = raw_for(nm)
    if not 20 <= raw <= raw_for(UNLOCK_HOLD_NM_EST / 2):
        raise ValueError(f"gripper cap {nm} N.m (raw {raw}) outside 0.06..{UNLOCK_HOLD_NM_EST / 2} N.m")
    bus.write("Torque_Limit", motor, raw, normalize=False)
    verify_grip_cap(bus, raw, motor)
    return raw


def verify_grip_cap(bus: Any, raw: int, motor: str = "gripper") -> None:
    got = int(bus.read("Torque_Limit", motor, normalize=False))
    if got != raw:
        raise RuntimeError(f"gripper Torque_Limit is {got}, expected {raw} (servo rebooted?) - stop and re-apply")
