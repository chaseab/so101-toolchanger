# SPDX-License-Identifier: GPL-3.0-or-later
"""Teleop the follower WITH the tool changer on (leader COM7 -> follower COM8), for hand-flown dock tries.

Same as teleop.bat (robust_teleop.py retries) plus changer limits, applied right after lerobot's configure():
  - gripper (= blade) RAM Torque_Limit capped at 0.25 N.m via grip_cap (lerobot writes 1.47 N.m; at that the blade
    can twist a locked tool in the unlock direction, see HANDOFF 10/05 redteam);
  - body joints 1-4 RAM Torque_Limit BODY_LIMIT (softer push into the dock); wrist_roll left at full for the lock twist;
  - max_relative_target per step so the start-up snap to the leader and any leader jerk are rate-limited.
Prints pos/load/temp once a second and warns on load > 70 % or temp >= 55 C; CSV in swap/logs/.

    %PY% tc_teleop.py            (Ctrl-C = stop: lerobot disconnects and drops torque -> hand under the arm)
"""
from __future__ import annotations

import csv
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ARM = HERE.parents[1]
sys.path.append(str(ARM))           # append, not insert: gripper/tools/tree.py once shadowed the `tree` package
import importlib.util  # noqa: E402

import robust_teleop  # noqa: E402,F401  (patches the bus retries on import)
from lerobot.robots.so_follower.so_follower import SOFollower  # noqa: E402

_spec = importlib.util.spec_from_file_location("grip_cap", ARM / "gripper" / "tools" / "grip_cap.py")
grip_cap = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(grip_cap)

BODY = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
BODY_LIMIT = 800                    # 0..1000 raw (80 %)
MAX_REL = 8.0                       # per-step cap in lerobot's normalized units
LOAD_WARN, TEMP_STOP = 700, 55
ELBOW_MAX = 93.0                    # normalized; folded rest reads ~98.4

LOG = HERE / "logs" / f"tc_teleop_{time.strftime('%Y%m%d_%H%M%S')}.csv"
TRACE = LOG.with_name(LOG.stem + "_trace.csv")   # 10 Hz cmd + measured positions, for turning catches into waypoints
TRACE_DT = 0.1
_orig_configure = SOFollower.configure
_orig_send = SOFollower.send_action
_state = {"t": 0.0, "w": None, "peak": {}, "tt": 0.0, "tw": None}


def configure(self) -> None:
    _orig_configure(self)
    raw = grip_cap.apply_grip_cap(self.bus, grip_cap.CHANGER_MAX_NM)
    for m in BODY:
        self.bus.write("Torque_Limit", m, BODY_LIMIT, normalize=False)
    for m in self.bus.motors:         # jog.py leaves a slow Goal_Velocity in RAM; lerobot never resets it (0 = max)
        self.bus.write("Goal_Velocity", m, 0, normalize=False)
    print(f"[tc] gripper cap raw {raw} ({grip_cap.CHANGER_MAX_NM} N.m), body Torque_Limit {BODY_LIMIT}, "
          f"max_relative_target {self.config.max_relative_target}")


def send_action(self, action):
    if not _state.get("guarded"):                                # pm-eng 10/30: teleop is filmed too
        _state["guarded"] = True
        import capguard
        capguard.require("tc_teleop.py")
    # 10/27 runs 2-3: the leader folds the elbow past the follower's folded rest (3110 of max 3128 = +98.4), so the
    # elbow sat at ~99 % load pressing into its rest. Stop ~5 deg short of it.
    if "elbow_flex.pos" in action:
        action = {**action, "elbow_flex.pos": min(action["elbow_flex.pos"], ELBOW_MAX)}
    out = _orig_send(self, action)
    now = time.perf_counter()
    if now - _state["tt"] >= TRACE_DT:
        _state["tt"] = now
        try:
            obs = self.bus.sync_read("Present_Position")      # normalized, same units as the action
            ld = self.bus.sync_read("Present_Load", normalize=False)
            ld = {m: v / 10 for m, v in ld.items()}   # signed %: lerobot's sync_read already decodes the sign bit (10)
        except Exception:  # noqa: BLE001
            obs = None
        if obs is not None:
            if _state["tw"] is None:
                f = open(TRACE, "w", newline="")
                _state["tw"] = (f, csv.writer(f))
                _state["tw"][1].writerow(["t"] + [f"cmd_{m}" for m in obs] + [f"obs_{m}" for m in obs] + [f"loadpct_{m}" for m in obs])
            f, w = _state["tw"]
            w.writerow([f"{time.time():.2f}"] + [f"{out.get(m + '.pos', float('nan')):.2f}" for m in obs]
                       + [f"{obs[m]:.2f}" for m in obs] + [f"{ld[m]:.1f}" for m in obs])
            f.flush()
    if now - _state["t"] >= 1.0:
        _state["t"] = now
        try:
            pos = self.bus.sync_read("Present_Position", normalize=False)
            load = self.bus.sync_read("Present_Load", normalize=False)
            temp = self.bus.sync_read("Present_Temperature", normalize=False)
        except Exception as e:  # noqa: BLE001 - a missed monitor read must not stop teleop
            print(f"[tc] monitor read failed: {e}")
            return out
        # 10/29: lerobot already sign-decodes Present_Load (bit 10). Decoding it again turned small negative loads into
        # ~-100 % (the 10/27 "pan at ~100 % load at rest" was this bug, not a real load).
        for m, v in load.items():
            _state["peak"][m] = max(_state["peak"].get(m, 0), abs(v))
        if _state["w"] is None:
            LOG.parent.mkdir(exist_ok=True)
            f = open(LOG, "w", newline="")
            _state["w"] = (f, csv.writer(f))
            _state["w"][1].writerow(["t"] + [f"{m}_{k}" for m in pos for k in ("pos", "load", "temp")])
        f, w = _state["w"]
        w.writerow([f"{time.time():.2f}"] + [x for m in pos for x in (pos[m], load[m], temp[m])])
        f.flush()
        warn = [f"{m} load {load[m]}" for m in load if abs(load[m]) > LOAD_WARN]
        warn += [f"{m} {temp[m]}C" for m in temp if temp[m] >= TEMP_STOP]
        print("[tc] load " + " ".join(f"{m[:5]}{load[m]:+d}" for m in load)
              + "  maxT " + str(max(temp.values())) + ("   !!! " + ", ".join(warn) if warn else ""))
        # 2 reads in a row (runbook rule): a single 31 -> 67 C glitch read stopped the 10/27 14:01 run and dropped torque
        hot = {m for m in temp if temp[m] >= TEMP_STOP}
        if hot & _state.get("hot", set()):
            raise KeyboardInterrupt(f"temperature >= {TEMP_STOP} C on 2 reads: {sorted(hot)}")
        _state["hot"] = hot
    return out


SOFollower.configure = configure
SOFollower.send_action = send_action

if __name__ == "__main__":
    from lerobot.scripts.lerobot_teleoperate import main

    sys.argv = ["lerobot-teleoperate", "--robot.type=so101_follower", "--robot.port=COM8", "--robot.id=follower_arm",
                f"--robot.max_relative_target={MAX_REL}", "--teleop.type=so101_leader", "--teleop.port=COM7",
                "--teleop.id=leader_arm",
                # 10/29: Ctrl-C must HOLD the arm where it is (lerobot's default disconnect drops torque = the arm falls)
                "--robot.disable_torque_on_disconnect=false"] + sys.argv[1:]
    try:
        main()
    finally:
        print(f"\n[tc] peak |load| per joint: {_state['peak']}  log: {LOG}")
        for k in ("w", "tw"):
            if _state[k]:
                _state[k][0].close()
