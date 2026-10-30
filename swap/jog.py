# SPDX-License-Identifier: GPL-3.0-or-later
"""Raw-bus jog for the changer arm (COM8), no lerobot connect (its configure drops torque = the arm falls).

    jog.py read                       positions (deg from mid, raw), load %, temp, torque
    jog.py hold                       goal := present on every torqued joint (kills a stall without moving)
    jog.py move J DEG [J DEG ...]     relative move, at most MAX_STEP deg per joint per call, slow profile
    jog.py goto J RAW [J RAW ...]     absolute raw goal (same per-call cap)
    jog.py off                        torque off on all 6 (support the arm first)

Joints: pan lift elbow flex roll grip (ids 1-6).  Goals are clamped to the follower_arm calibration range.
Every move polls at ~20 Hz; a joint at >= STALL_LOAD % load that moved < 2 ticks for 0.4 s, or any temp >= 55 C,
stops the move: every joint gets goal := present (hold).  Ctrl-C does the same.
RAM Torque_Limit from tc_teleop (80 % body, gripper 0.25 N.m) stays until power-off; `read` shows it.
"""
from __future__ import annotations

import json
import os
import sys
import time

from scservo_sdk import PacketHandler, PortHandler

NAMES = ["pan", "lift", "elbow", "flex", "roll", "grip"]
CAL_KEYS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
CAL = json.load(open(os.path.expanduser(r"~/.cache/huggingface/lerobot/calibration/robots/so_follower/follower_arm.json")))
MAX_STEP = 50.0                      # deg per joint per call (planned + FK-checked moves)
SPEED, ACC = 120, 10                 # Goal_Speed steps/s (~10.5 deg/s), Acceleration (x100 steps/s^2)
STALL_LOAD = 90
TPD = 4096 / 360.0
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "jog_1027.jsonl")

class _FakePort:
    """TC_FAKE_BUS=1: no serial port. Registers per id; Present_Position slews toward Goal_Position, load ~ error."""
    def openPort(self): return True
    def closePort(self): pass
    def setBaudRate(self, b): return True


class _FakePK:
    def __init__(self):
        import time as _t
        self._t = _t
        self.reg = {i: {56: 2048, 42: 2048, 40: 1, 48: 800, 46: 0, 41: 0, 63: 35, 9: 0, 11: 4095, 55: 1} for i in range(1, 7)}
        self.last = {i: _t.time() for i in range(1, 7)}
        self._state = os.environ.get("TC_FAKE_STATE")  # optional: positions shared across fake-bus processes
        self._mt = 0.0
        self._load()

    def _load(self):
        if self._state and os.path.exists(self._state) and os.path.getmtime(self._state) > self._mt:
            self._mt = os.path.getmtime(self._state)
            for k, v in json.load(open(self._state)).items():
                self.reg[int(k)][56] = self.reg[int(k)][42] = int(v)

    def _save(self):
        if self._state:
            json.dump({i: self.reg[i][42] for i in range(1, 7)}, open(self._state, "w"))   # goal: it slews there in ms
            self._mt = os.path.getmtime(self._state)

    def _tick(self, i):
        r = self.reg[i]; now = self._t.time(); dt = now - self.last[i]; self.last[i] = now
        err = r[42] - r[56]
        r[56] += int(max(-3000 * dt, min(3000 * dt, err))) if r[40] else 0
        e = r[42] - r[56]; mag = min(1000, abs(e) * 10)
        r[60] = (1024 | mag) if e < 0 else mag       # raw sign-magnitude like the STS3215

    def read2ByteTxRx(self, port, i, addr):
        self._load(); self._tick(i); return self.reg[i].get(addr, 0), 0, 0

    def read1ByteTxRx(self, port, i, addr):
        self._tick(i); return self.reg[i].get(addr, 0), 0, 0

    def write2ByteTxRx(self, port, i, addr, v):
        self._tick(i); self.reg[i][addr] = int(v); self._save(); return 0, 0

    write1ByteTxRx = write2ByteTxRx


FAKE = os.environ.get("TC_FAKE_BUS") == "1"
if FAKE:
    port, pk = _FakePort(), _FakePK()
else:
    port = PortHandler("COM8"); port.openPort(); port.setBaudRate(1000000)
    pk = PacketHandler(0)
import threading as _th
BUS = _th.RLock()                    # the 10 Hz tracer thread and the motion code share one serial port


def rd(i, addr, n):
    with BUS:
        for _ in range(5):
            v, c, e = (pk.read2ByteTxRx if n == 2 else pk.read1ByteTxRx)(port, i, addr)
            if c == 0:
                return v
    raise IOError(f"read id {i} addr {addr} failed")


_GUARD = {"done": False, "hold": False}


def _guard():
    """pm-eng 10/30: every real goal write is on a live take. First goal write per process -> capguard.require.
    hold() is exempt (an emergency hold must never be refused)."""
    if FAKE or _GUARD["done"] or _GUARD["hold"] or os.environ.get("TC_TAKE"):   # TC_TAKE: this process already passed require()
        return
    _GUARD["done"] = True
    import capguard
    capguard.require(os.path.basename(sys.argv[0]) or "jog")


def wr(i, addr, n, v):
    if addr == 42:
        _guard()
    with BUS:
        for _ in range(5):
            c, e = (pk.write2ByteTxRx if n == 2 else pk.write1ByteTxRx)(port, i, addr, int(v))
            if c == 0:
                return
    raise IOError(f"write id {i} addr {addr} failed")


_TRACE = {}


def start_trace(tag, hz=10.0):
    """10 Hz attempt trace (10/29, pm-eng: the twin needs real roll-load traces of locks + the lift-out):
    logs/trace_<attempt>_<tag>_<HHMMSS>.csv with t, take, attempt, pos/goal/load of all 6 joints every tick and temps
    every 10th tick. Attempt id = env TC_ATTEMPT (set by the top-level script, shared by its subprocesses)."""
    if _TRACE.get("on"):
        return _TRACE["path"]
    import csv as _csv, time as _t
    attempt = os.environ.setdefault("TC_ATTEMPT", _t.strftime("%Y%m%d_%H%M%S"))
    take = os.environ.get("TC_TAKE", "")               # set by capguard.require() only for a verified live take
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs",
                        f"trace_{attempt}_{tag.replace(' ', '_').replace('.py', '')}_{_t.strftime('%H%M%S')}.csv")
    f = open(path, "w", newline=""); w = _csv.writer(f)
    w.writerow(["t", "take", "attempt", "fake"] + [f"{k}_{n}" for n in NAMES for k in ("pos", "goal", "load")]
               + [f"temp_{n}" for n in NAMES])
    _TRACE.update(on=True, path=path)

    def loop():
        k = 0
        while _TRACE.get("on"):
            t0 = _t.time()
            try:
                with BUS:
                    row = []
                    for i in range(1, 7):
                        row += [rd(i, 56, 2), rd(i, 42, 2), load(i)]
                    temps = [rd(i, 63, 1) for i in range(1, 7)] if k % 10 == 0 else [""] * 6
                w.writerow([f"{t0:.3f}", take, attempt, int(FAKE)] + row + temps); f.flush()
            except Exception as e:  # noqa: BLE001 - a missed trace sample must never stop motion
                w.writerow([f"{t0:.3f}", take, attempt, int(FAKE), f"read error {e}"]); f.flush()
            k += 1
            _t.sleep(max(0.0, 1.0 / hz - (_t.time() - t0)))
        f.close()

    th = _th.Thread(target=loop, daemon=True); th.start(); _TRACE["thread"] = th
    import atexit
    atexit.register(stop_trace)
    return path


def stop_trace():
    if _TRACE.get("on"):
        _TRACE["on"] = False
        th = _TRACE.get("thread")
        if th: th.join(timeout=1.0)


def load(i):
    v = rd(i, 60, 2)
    return -(v & 0x3FF) / 10 if v & 0x400 else v / 10


def state():
    s = {}
    for k, n in enumerate(NAMES):
        i = k + 1
        s[n] = dict(pos=rd(i, 56, 2), load=load(i), temp=rd(i, 63, 1), tq=rd(i, 40, 1), lim=rd(i, 48, 2))
    return s


def show(s, label=""):
    print(label + "  ".join(f"{n}:{v['pos']}({(v['pos'] - 2048) / TPD:+.1f}d) L{v['load']:+.0f}% {v['temp']}C"
                            + ("" if v["tq"] else " OFF") for n, v in s.items()))


def hold():
    _GUARD["hold"] = True
    try:
        for k in range(6):
            if rd(k + 1, 40, 1):
                wr(k + 1, 42, 2, rd(k + 1, 56, 2))
    finally:
        _GUARD["hold"] = False


def move(goals):
    """goals: {name: raw}.  Slow profile, stall/temperature watch, hold on any trip."""
    s0 = state()
    for n, g in goals.items():
        k = NAMES.index(n); c = CAL[CAL_KEYS[k]]
        g = max(c["range_min"], min(c["range_max"], int(g)))
        if abs(g - s0[n]["pos"]) > MAX_STEP * TPD + 1:
            raise SystemExit(f"{n}: step {(g - s0[n]['pos']) / TPD:+.1f} deg > {MAX_STEP} cap")
        goals[n] = g
    far = max(abs(g - s0[n]["pos"]) for n, g in goals.items()) or 1
    for n, g in goals.items():
        i = NAMES.index(n) + 1                  # per-joint speed so every joint arrives together
        wr(i, 41, 1, ACC); wr(i, 46, 2, max(15, int(SPEED * abs(g - s0[n]["pos"]) / far)))
        if not rd(i, 40, 1):
            wr(i, 42, 2, rd(i, 56, 2)); wr(i, 40, 1, 1)
        wr(i, 42, 2, g)
    t0 = time.time(); stuck = {}; peak = {n: 0.0 for n in NAMES}; last = {n: s0[n]["pos"] for n in goals}
    try:
        while time.time() - t0 < far / SPEED + 4.0:
            time.sleep(0.05)
            done = True
            for n, g in goals.items():
                i = NAMES.index(n) + 1
                p, ld = rd(i, 56, 2), load(i)
                peak[n] = max(peak[n], abs(ld))
                if abs(p - g) > 8:
                    done = False
                if abs(ld) >= STALL_LOAD and abs(p - last[n]) < 2:
                    stuck[n] = stuck.get(n, 0) + 1
                    if stuck[n] >= 8:
                        hold(); show(state(), f"STALL {n} -> HOLD | ")
                        return False
                else:
                    stuck[n] = 0
                last[n] = p
            if done:
                break
        temps = {n: rd(NAMES.index(n) + 1, 63, 1) for n in NAMES}
        if max(temps.values()) >= 55:
            hold(); print("HOT -> HOLD", temps); return False
    except KeyboardInterrupt:
        hold(); print("Ctrl-C -> HOLD"); return False
    s = state()
    for _ in range(2):                         # sag correction: re-aim by the steady-state error (<= 8 deg)
        fix = {n: g + max(-8 * TPD, min(8 * TPD, g - s[n]["pos"])) for n, g in goals.items()
               if n in ("lift", "elbow", "flex") and abs(g - s[n]["pos"]) > 1.0 * TPD}
        if not fix:
            break
        for n, v in fix.items():
            wr(NAMES.index(n) + 1, 42, 2, int(v))
        time.sleep(1.2)
        s = state()
        if any(abs(s[n]["load"]) >= STALL_LOAD for n in fix):
            hold(); show(state(), "LOAD on sag fix -> HOLD | "); return False
    show(s, "done | ")
    with open(LOG, "a") as fh:
        fh.write(json.dumps({"t": time.strftime("%H:%M:%S"), "goals": goals, "end": {n: s[n]["pos"] for n in NAMES},
                             "peak": peak, "temp": {n: s[n]["temp"] for n in NAMES}}) + "\n")
    print("peak load % " + " ".join(f"{n}:{peak[n]:.0f}" for n in goals)
          + "  err " + " ".join(f"{n}:{(s[n]['pos'] - g) / TPD:+.1f}d" for n, g in goals.items()))
    return True


if __name__ == "__main__":
    a = sys.argv[1:]
    cmd = a[0] if a else "read"
    if cmd == "read":
        s = state(); show(s); print("Torque_Limit raw", {n: v["lim"] for n, v in s.items()})
    elif cmd == "hold":
        hold(); show(state(), "hold | ")
    elif cmd == "off":
        for i in range(1, 7):
            wr(i, 40, 1, 0)
        show(state(), "off | ")
    elif cmd in ("move", "goto"):
        import capguard; capguard.require("jog.py " + cmd)        # 10/29: no scripted motion without a live take
        pairs = dict(zip(a[1::2], a[2::2]))
        s = state()
        goals = {n: (s[n]["pos"] + float(v) * TPD if cmd == "move" else float(v)) for n, v in pairs.items()}
        move(goals)
    port.closePort()
