# SPDX-License-Identifier: GPL-3.0-or-later
"""Offline tests for tool_session + swap_routine save/load (mock bus, no hardware).
python -m unittest test_tool_session   (cwd toolchanger/swap, lerobot venv for numpy)"""
import os
import tempfile
import unittest

import swap_routine as S
import tool_session as T


class Bus:
    def __init__(self):
        self.r = {"Torque_Limit": 500}

    def write(self, name, motor, value, normalize=True):
        self.r[name] = value

    def read(self, name, motor, normalize=True):
        return self.r.get(name, 0)


class ToolSession(unittest.TestCase):
    def test_session_cap(self):
        b = Bus()
        self.assertEqual(T.apply_session_limits(b), 85)
        self.assertEqual(b.r["Torque_Limit"], 85)
        with self.assertRaises(ValueError):
            T.apply_session_limits(Bus(), 0.3)

    def test_fingerprint_ignores_gripper(self):
        d = {"shoulder_lift": {"homing_offset": 1, "range_min": 2, "range_max": 3},
             "gripper": {"homing_offset": 9, "range_min": 9, "range_max": 9}}
        import json
        with tempfile.TemporaryDirectory() as t:
            p = os.path.join(t, "c.json")
            json.dump(d, open(p, "w"))
            a = T.calibration_fingerprint(p)
            d["gripper"]["range_max"] = 10
            json.dump(d, open(p, "w"))
            self.assertEqual(a, T.calibration_fingerprint(p))
            d["shoulder_lift"]["homing_offset"] = 5
            json.dump(d, open(p, "w"))
            self.assertNotEqual(a, T.calibration_fingerprint(p))

    def test_dock_load_guards(self):
        with tempfile.TemporaryDirectory() as t:
            p = os.path.join(t, "docks.json")
            S.save(p, {"a": S.DockTeach("a", [0, 0, 0, 0, 2.79])}, S.FrictionCal(), "abc")
            self.assertEqual(S.load(p, "abc")[0]["a"].q_seat[4], 2.79)
            for bad in ("xyz", None):
                with self.assertRaises(S.SwapError):
                    S.load(p, bad)
            S.save(p, {"a": S.DockTeach("a", [0, 0, 0, 0, 170.0])}, S.FrictionCal(), "abc")
            with self.assertRaises(S.SwapError):
                S.load(p, "abc")


if __name__ == "__main__":
    unittest.main()
