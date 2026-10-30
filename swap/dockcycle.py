# SPDX-License-Identifier: GPL-3.0-or-later
"""Automatic DROP + PICK on the current dock, no hardware changes (10/30, swap-desk).

Why this shape (10/29 logs, FK of jog_1027.jsonl + pick2_runs.jsonl):
- every lift that WORKED started with the face at y ~241-243 (18:39 scripted lift-out from the owner's teleop seat, load
  <= 48 %); every lift that JAMMED started at y ~255-257, after the push-to-hard-stop lock had shoved the tool ~15 mm
  deeper along its slot toward the wall (21:54, 22:08, 20:51: lift pinned 56-78 %).
- the scripted drop at 18:41 (lower pitch-held to face (52, 238, 93), unlock roll to entry, back off) worked.
- a lock only catches at the hard stop (owner 10/29), so: lock at the stop, then DRAG the locked tool back along the
  slot to the lift station (y ~241) and lift there. The drop puts the tool back at the same station.

    python dockcycle.py drop            carried tool -> parked at the station, master backed off
    python dockcycle.py pick            parked tool -> locked, dragged to the station, lifted +LIFT mm
    python dockcycle.py cycle N         N x (drop, pick) starting with the tool on the master
    options: --x 52.5 --ystation 241 --ystop 257 --zlock 101 --zdrop 93 --zhang 126 --az 0.035 --lift 30
             --no-cam (skip dock-cam checks)
Every step is checked on the encoders (FK) and, when a take is live, the dock cam. A failed check -> HOLD + exit,
never a blind retry; the pick lift has a fixed escalation ladder (re-station, pitch variants, yank). Temps: waits for
<= 50 C before each step if any joint is > 52 C; aborts at 58 C. One JSON line per step in logs/dockcycle_runs.jsonl.
Exit 0 = all steps ok, 2 = a check failed (arm held), 1 = error.
"""
import json, os, subprocess, sys, time
import numpy as np
import jog as J
import where as W

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
TPD = 4096 / 360.0
a = sys.argv[1:]
def opt(n, d): return type(d)(a[a.index(n) + 1]) if n in a else d
X, YST, YSTOP = opt("--x", 52.5), opt("--ystation", 241.0), opt("--ystop", 257.0)
ZLOCK, ZDROP, ZHANG = opt("--zlock", 101.0), opt("--zdrop", 93.0), opt("--zhang", 126.0)
AZ, LIFT, ENTRY = opt("--az", 0.035), opt("--lift", 30.0), opt("--entry", 2487)
YBACK = opt("--yback", 208.0)                 # backed-off standoff (18:41 back-off ended at y 207.7)
FAKELOCK = os.environ.get("TC_FAKE_BUS") == "1"          # the fake bus has no dock: skip the lock-load check
NOCAM = "--no-cam" in a or os.environ.get("TC_FAKE_BUS") == "1"
LOG = os.path.join(HERE, "logs", "dockcycle_runs.jsonl")
STATE = os.path.join(HERE, "logs", "dock_state.json")


class Fail(Exception):
    pass


# ---------- bus helpers (same as pick2) ----------
def run(*args):
    with J.BUS:                                          # the child owns COM8 while it runs
        J.port.closePort()
        try:
            r = subprocess.run([PY, *map(str, args)], capture_output=True, text=True, cwd=HERE)
        finally:
            J.port.openPort(); J.port.setBaudRate(1000000)
    last = (r.stdout.strip().splitlines() or [r.stderr.strip()[-200:]])[-1]
    print("   ", " ".join(map(str, args))[:70], "->", last[:150])
    return last


def face():
    t = [J.rd(i, 56, 2) for i in range(1, 6)]; d, p, z = W.where(t)
    return np.array([float(v) for v in p]), float(-z[2])


def loads():
    return {n: J.load(i) for i, n in ((2, "lift"), (3, "elbow"), (4, "flex"), (5, "roll"))}


def maxtemp():
    return max(J.rd(i, 63, 1) for i in range(1, 7))


def cool():
    t = maxtemp()
    if t >= 58:
        raise Fail(f"temp {t} C >= 58")
    if t > 52:
        print(f"    cooling: {t} C -> <= 50"); J.hold(); t0 = time.time()
        while maxtemp() > 50 and time.time() - t0 < 900:
            time.sleep(10)


def stopfile():
    if os.path.exists(os.path.join(HERE, "STOP")):
        raise Fail("STOP file")


def roll_goto(goal, speed=150, secs=15, tl=1000):
    J.wr(5, 48, 2, tl); J.wr(5, 41, 1, 10); J.wr(5, 46, 2, speed); J.wr(5, 42, 2, int(goal)); t0 = time.time(); pk = 0
    while time.time() - t0 < secs and abs(J.rd(5, 56, 2) - goal) > 10:
        pk = max(pk, abs(J.load(5))); time.sleep(0.1)
    return J.rd(5, 56, 2), pk


def twist_to_stall():
    r0 = J.rd(5, 56, 2)
    J.wr(5, 48, 2, 850); J.wr(5, 41, 1, 10); J.wr(5, 46, 2, 60); J.wr(5, 42, 2, int(r0 - 80 * TPD))
    t0, last, still, pk = time.time(), r0, 0, 0
    while time.time() - t0 < 20:
        time.sleep(0.1); p = J.rd(5, 56, 2); pk = max(pk, abs(J.load(5)))
        still = still + 1 if abs(p - last) < 2 else 0; last = p
        if still >= 6: break
    J.wr(5, 42, 2, J.rd(5, 56, 2)); J.wr(5, 48, 2, 1000)
    return (J.rd(5, 56, 2) - r0) / TPD, pk


def relax(joint, step, limit=4, n=24, wait=0.3):
    for k in range(n):
        l = J.load(joint)
        if abs(l) <= limit: return k
        p = J.rd(joint, 56, 2)
        J.wr(joint, 46, 2, 30); J.wr(joint, 42, 2, int(p - step if l > 0 else p + step)); time.sleep(wait)
    return n


def ok(line):
    return line.startswith("done")


# ---------- dock cam ----------
def grab(name):
    if NOCAM:
        return None
    import capguard, cv2
    p = os.path.join(HERE, "logs", "vis", name)
    if not capguard.live_take():
        return None
    time.sleep(1.6)                                      # the take's newest frame lags ~1.5 s
    capguard.latest_frame(p, 800)
    return cv2.imread(p, 0)


TOOL_BOX = (40, 250, 230, 390)                           # tool body in the 800 px dock frame (wiggle.py)


def tool_shift(img_a, img_b):
    """Tool-body shift (dx, dy px; +dy = down) between two dock frames, or None without frames."""
    if img_a is None or img_b is None:
        return None
    import cv2
    x0, y0, x1, y1 = TOOL_BOX
    r = cv2.matchTemplate(img_b, img_a[y0:y1, x0:x1], cv2.TM_CCOEFF_NORMED)
    _, mx, _, loc = cv2.minMaxLoc(r)
    return loc[0] - x0, loc[1] - y0, round(float(mx), 2)


# ---------- steps ----------
def drop(rec):
    """Tool on the master -> parked at the station, master backed off."""
    cool(); stopfile()
    rec["roll_start"] = J.rd(5, 56, 2)
    p, _ = face(); rec["start"] = p.round(1).tolist()
    if abs(p[0] - X) > 25 or not (YBACK - 15 < p[1] < YST + 15) or not (ZDROP < p[2] < ZHANG + 40):
        raise Fail(f"drop must start with the tool carried in front of/above the station, face is at {p.round(1)}")
    if p[2] < ZHANG - 4:                                  # come up first, straight, so the tool clears the wall
        if not ok(run("smooth.py", "z", ZHANG - p[2], "--az", AZ, "--speed", 2)):
            raise Fail("could not rise to hang height")
    if not ok(run("smooth.py", "to", X, YST, ZHANG, "--az", AZ, "--speed", 2)):
        raise Fail("could not reach the hang pose")
    img0 = grab("dc_drop0.jpg")
    # lower pitch-held; --contact stops early if the neck lands on the wall top instead of in the slot
    line = run("smooth.py", "z", ZDROP - ZHANG, "--az", AZ, "--speed", 1.5, "--contact", 4)
    p, _ = face(); rec["lowered"] = p.round(1).tolist(); rec["lower"] = line[:120]
    if p[2] > ZDROP + 8:
        J.hold(); raise Fail(f"lower stopped at z {p[2]:.1f} (> {ZDROP + 8}): landed on the wall top, not in the slot")
    rec["loads_lowered"] = loads()
    r, pk = roll_goto(ENTRY); rec["unlock_roll"], rec["unlock_peak"] = r, pk
    if abs(r - ENTRY) > 30:
        J.hold(); raise Fail(f"unlock twist stopped at {r} (entry {ENTRY}), peak {pk:.0f} %")
    img1 = grab("dc_drop1.jpg")
    if not ok(run("smooth.py", "to", X, YBACK, p[2], "--az", AZ, "--speed", 2)):
        raise Fail("back-off held (tool still on the master?)")
    img2 = grab("dc_drop2.jpg")
    s_down, s_back = tool_shift(img0, img1), tool_shift(img1, img2)
    rec["cam_down"], rec["cam_backoff"] = s_down, s_back
    if s_back is not None and np.hypot(*s_back[:2]) >= 8:
        raise Fail(f"tool moved {s_back[:2]} px with the back-off: it is still on the master")
    json.dump({"tool": "docked", "t": time.time(), "drop_face": rec["lowered"]}, open(STATE, "w"))


def try_lift(rec, tag, az):
    p, _ = face()
    line = run("smooth.py", "z", LIFT, "--az", az, "--speed", 1)
    p1, _ = face(); rec.setdefault("lifts", []).append({"tag": tag, "az": az, "from": p.round(1).tolist(),
                                                        "to": p1.round(1).tolist(), "line": line[:140]})
    return ok(line) and p1[2] - p[2] >= LIFT - 5


def back_down(p0):
    """After a held lift: return to the lift-start height so the next try starts unloaded."""
    p, _ = face()
    if p[2] - p0[2] > 0.5:
        run("smooth.py", "z", -(p[2] - p0[2]), "--az", AZ, "--speed", 1)
    relax(2, 4, limit=8)


def pick(rec):
    """Parked tool -> locked, dragged to the station, lifted."""
    cool(); stopfile()
    rec["roll_entry"], _ = roll_goto(ENTRY)
    p, _ = face()
    if abs(p[0] - X) > 40 or not (150 < p[1] < YSTOP) or not (ZLOCK - 20 < p[2] < ZHANG + 60):
        raise Fail(f"pick must start in front of the dock (master empty), face is at {p.round(1)}")
    if p[1] > YBACK + 5:                                  # never sweep sideways through the dock: back out first
        run("smooth.py", "to", p[0], YBACK, p[2], "--az", AZ, "--speed", 2)
    if not ok(run("smooth.py", "to", X, YBACK, ZLOCK + 2, "--az", 0.05, "--speed", 2)):
        raise Fail("could not reach the standoff")
    run("smooth.py", "to", X, YSTOP, ZLOCK, "--az", 0.05, "--speed", 1.5, "--pan-soft", 30)
    p, _ = face(); rec["at_stop"], rec["loads_stop"] = p.round(1).tolist(), loads()
    if p[1] < YST + 3:
        raise Fail(f"push stopped at y {p[1]:.1f}, before the station: something is in the way")
    ang, pk = twist_to_stall(); rec["twist_deg"], rec["twist_peak"] = round(ang, 1), pk
    print(f"    twist {ang:.1f} deg, peak {pk:.0f} %")
    if (ang < -75 or pk < 30) and not FAKELOCK:
        roll_goto(ENTRY); raise Fail("no lock: twist spun free")
    rec["roll_relax"] = relax(5, 6)
    img0 = grab("dc_pick0.jpg")
    # drag the locked tool back along its slot to the station where lifts worked (pan soft: the slot steers)
    line = run("smooth.py", "to", X, YST, p[2], "--az", AZ, "--speed", 1, "--pan-soft", 30)
    p, _ = face(); rec["dragged_to"], rec["drag"] = p.round(1).tolist(), line[:140]
    img1 = grab("dc_pick1.jpg")
    s_drag = tool_shift(img0, img1); rec["cam_drag"] = s_drag
    if not ok(line) and p[1] > YST + 6:
        raise Fail(f"drag-back held at y {p[1]:.1f}")
    if s_drag is not None and np.hypot(*s_drag[:2]) < 8:  # the axial drag may be hard to see: flag, the lift check decides
        print(f"    note: dock cam saw the tool move only {s_drag[:2]} px on the drag-back")
    relax(3, 4, limit=12)                                 # unclamp the axial load before lifting
    p0, _ = face()
    # lift ladder: station pitch, then tip a bit down / up (twin: -3 deg lifts, +3/+5 jams), then 2 mm further out,
    # then the 10/29 22:4x yank (tool tip up, short full-torque pulls)
    ladder = [("station", AZ), ("tip-down", AZ - 0.05), ("tip-up", AZ + 0.04)]
    lifted = False
    for tag, az in ladder:
        stopfile(); cool()
        if try_lift(rec, tag, az):
            lifted = True; break
        back_down(p0)
    if not lifted:
        run("smooth.py", "to", X, YST - 2, p0[2], "--az", AZ, "--speed", 0.8, "--pan-soft", 30)
        p0, _ = face()
        lifted = try_lift(rec, "station-2mm", AZ)
        if not lifted:
            back_down(p0); cool()
            rec["yank"] = run("yank.py", "--n", 3)
            lifted = try_lift(rec, "after-yank", AZ)
    J.hold()
    img2 = grab("dc_pick2.jpg")
    s_lift = tool_shift(img1, img2); rec["cam_lift"] = s_lift
    if not lifted:
        raise Fail("lift held on every rung: tool locked but still in the slot")
    if s_lift is not None and s_lift[1] > -20:
        raise Fail(f"master went up but the tool moved only {s_lift[:2]} px: it stayed in the dock")
    json.dump({"tool": "carried", "t": time.time()}, open(STATE, "w"))


def step(name, fn, k=0):
    rec = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "step": name, "n": k, "args": a}
    t0 = time.time(); code = 0
    try:
        fn(rec); rec["result"] = "ok"
    except Fail as e:
        J.hold(); rec["result"] = f"FAIL: {e}"; code = 2
    except KeyboardInterrupt:
        J.hold(); rec["result"] = "ctrl-c"; code = 2
    rec["secs"] = round(time.time() - t0, 1); rec["temp_end"] = maxtemp()
    with open(LOG, "a") as f:
        f.write(json.dumps(rec, default=str) + "\n")
    with open(os.path.join(HERE, "logs", "takes_index.jsonl"), "a") as f:        # for pm-reels-lab: one line per attempt
        f.write(json.dumps({"t": rec["t"], "take": os.environ.get("TC_TAKE"), "attempt": os.environ.get("TC_ATTEMPT"),
                            "script": f"dockcycle {name}", "result": rec["result"], "fake": FAKELOCK,
                            "trace": J._TRACE.get("path")}) + "\n")
    print(f"== {name} #{k}: {rec['result']} ({rec['secs']} s)")
    return code


def main():
    import capguard
    rec_take = capguard.require("dockcycle.py")
    J.start_trace("dockcycle"); print("take", rec_take, "attempt", os.environ.get("TC_ATTEMPT"))
    vals = {a[i + 1] for i, x in enumerate(a[:-1]) if x.startswith("--") and x != "--no-cam"}
    cmd = [x for x in a if not x.startswith("--") and x not in vals][:2] or ["help"]
    try:
        if cmd[0] == "drop":
            return step("drop", drop)
        if cmd[0] == "pick":
            return step("pick", pick)
        if cmd[0] == "cycle":
            n = int(cmd[1]) if len(cmd) > 1 else 1; good = 0
            for k in range(1, n + 1):
                if step("drop", drop, k) or step("pick", pick, k):
                    break
                good += 1
            print(f"CYCLES OK: {good}/{n}")
            return 0 if good == n else 2
        print(__doc__); return 1
    finally:
        J.hold(); capguard.release("dockcycle.py")


if __name__ == "__main__":
    sys.exit(main())
