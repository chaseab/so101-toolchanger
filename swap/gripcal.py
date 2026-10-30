# SPDX-License-Identifier: GPL-3.0-or-later
"""Second follower calibration for changer sessions: follower_arm_tc = follower_arm with a WIDE gripper range.

Why (TEST_PLAN_SO101 Stage B): the changer's blade (gripper servo ID 6) needs a 260 deg search plus a +49 deg twist
follow, and lerobot clamps the gripper to its calibrated range (stock: 1270..2667 ticks = 123 deg).  A search that
hits the range end looks like a stall and gives a wrong blade offset.  The arm joints are copied unchanged, so
tool_session.calibration_fingerprint() (arm joints only) is identical and dock teach files stay valid.

    python gripcal.py show                       # both files, side by side (no hardware)
    python gripcal.py create --entry-tick N      # write follower_arm_tc.json (no hardware; N from `measure`)
    python gripcal.py check                      # offline validation of follower_arm_tc.json
    python gripcal.py measure --port COM8        # READ-ONLY: gripper raw Present_Position at the blade entry angle

Switching: changer scripts use --id follower_arm_tc (run_swap.py default); teleop / record with the stock gripper use
follower_arm.  sweep.connect_real() writes the file's homing offset + position limits to the servo EEPROM whenever
they differ from the motors (with torque briefly off - the arm must be at rest), so every id switch rewrites them.
That is expected.  Do NOT teleop the changer with follower_arm_tc: the leader's 0..100 would drive the blade over the
whole 350 deg (a per-tool jaw range is a separate, later calibration).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import tool_session  # noqa: E402

CAL_DIR = tool_session.CALIB.parent
STOCK = CAL_DIR / "follower_arm.json"
TC = CAL_DIR / "follower_arm_tc.json"
ENTRY_TICK = 3413                      # real_io.ENTRY_DEG 300 deg on the 0..4095 scale
TC_MIN, TC_MAX = 57, 4039              # 5 .. 355 deg: 4 deg from each end of the offset-space wrap
HALF = 2047


def make_tc(stock: dict, entry_tick_now: int) -> dict:
    """entry_tick_now = gripper Present_Position (raw, with the CURRENT homing offset) with the blade at its pick-entry
    angle.  New offset puts that angle at ENTRY_TICK; the range is widened to TC_MIN..TC_MAX."""
    g = dict(stock["gripper"])
    raw = entry_tick_now + int(g["homing_offset"])            # encoder reading without any offset (Present = raw - h)
    h = raw - ENTRY_TICK
    h = ((h + 2048) % 4096) - 2048                            # the register is sign-magnitude, |h| <= 2047
    if abs(h) > HALF:
        raise ValueError(f"homing offset {h} out of range")
    g.update(homing_offset=int(h), range_min=TC_MIN, range_max=TC_MAX)
    out = {k: dict(v) for k, v in stock.items()}
    out["gripper"] = g
    return out


def problems(tc: dict, stock: dict) -> list:
    out = []
    for j, v in stock.items():
        if j == "gripper":
            continue
        if tc.get(j) != v:
            out.append(f"{j} differs from follower_arm.json (the docks would need re-teaching)")
    g = tc.get("gripper", {})
    span = (g.get("range_max", 0) - g.get("range_min", 0)) * 360 / 4096
    if span < 300:
        out.append(f"gripper range spans {span:.0f} deg < 300")
    lo = g.get("range_min", 0) * 360 / 4096 + 3
    hi = g.get("range_max", 0) * 360 / 4096 - 3
    if not (lo <= ENTRY_TICK * 360 / 4096 - 260 and ENTRY_TICK * 360 / 4096 + 49 <= hi):
        out.append("entry - 260 .. entry + 49 does not fit the range")
    if tool_session.calibration_fingerprint(STOCK) and tool_session.calibration_fingerprint(STOCK) != _fp(tc):
        out.append("arm fingerprint differs")
    return out


def _fp(d: dict) -> str:
    import tempfile
    import os
    with tempfile.TemporaryDirectory() as t:
        p = os.path.join(t, "c.json")
        Path(p).write_text(json.dumps(d))
        return tool_session.calibration_fingerprint(Path(p))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("show")
    c = sub.add_parser("create")
    c.add_argument("--entry-tick", type=int, required=True)
    c.add_argument("--force", action="store_true")
    sub.add_parser("check")
    m = sub.add_parser("measure")
    m.add_argument("--port", default="COM8")
    a = ap.parse_args(argv)
    stock = json.loads(STOCK.read_text())
    if a.cmd == "show":
        tc = json.loads(TC.read_text()) if TC.exists() else None
        for j in stock:
            print(f"{j:14s} stock {stock[j]}" + (f"\n{'':14s} tc    {tc[j]}" if tc and j == 'gripper' else ""))
        print("follower_arm_tc.json:", "present" if tc else "not created yet")
        return 0
    if a.cmd == "create":
        if TC.exists() and not a.force:
            print(f"{TC} exists (use --force)")
            return 2
        tc = make_tc(stock, a.entry_tick)
        bad = problems(tc, stock)
        if bad:
            print("refusing:", bad)
            return 2
        TC.write_text(json.dumps(tc, indent=4))
        print(f"wrote {TC}: gripper {tc['gripper']}")
        return 0
    if a.cmd == "check":
        if not TC.exists():
            print("follower_arm_tc.json not created yet")
            return 2
        bad = problems(json.loads(TC.read_text()), stock)
        print("ok" if not bad else "\n".join(bad))
        return 0 if not bad else 2
    if a.cmd == "measure":
        # READ-ONLY on the motors except what connect_real itself does (torque on, stock calibration check).
        print("Arm at REST on its stops, 12 V on, nothing else on COM8.  The GRIPPER torque is released so you can turn "
              "the blade by hand to the pick-entry angle (slot of a parked rotor).  Type GO: ", end="")
        if input().strip().upper() != "GO":
            return 1
        from splat import sweep  # noqa
        robot = sweep.connect_real(a.port, "follower_arm")
        try:
            robot.bus.disable_torque(["gripper"])
            input("Turn the blade to the entry angle, then ENTER: ")
            v = int(robot.bus.read("Present_Position", "gripper", normalize=False))
            print(f"gripper raw Present_Position = {v}  ->  python gripcal.py create --entry-tick {v}")
        finally:
            robot.disconnect()                   # at rest: releasing torque is the normal end state
        return 0
    return 1


if __name__ == "__main__":
    sys.path.insert(0, str(HERE.parents[1] / "dashboard"))
    sys.exit(main())
