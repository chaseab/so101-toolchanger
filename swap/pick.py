# SPDX-License-Identifier: GPL-3.0-or-later
"""Cold automatic PICK from the parked dock (10/29), the method that locked at 20:28:

1. standoff in front of the tool at the dock-cam-matched height (owner: line the coupling up top/bottom in the dock cam),
   roll at the entry angle, seat pitch
2. slide straight in (smooth.py) to first contact (seat.py --contact-only)
3. push 1 mm, try the twist and read the roll-load trace:
     free spin (< 8 % all the way)          -> not deep enough: roll back to entry, push 1 mm more, retry (max --maxpush)
     ramp (load bump around -20..-35 deg) and stop (>= 10 % at -45 deg) -> LOCKED
4. back off --backoff mm (unload the dock wall), lift with liftout.py (lift leads, flex holds pitch-sum --pitch)

    python pick.py [--x 51.9 --y0 234 --z 103.1 --entry 2487 --pitch 25.3 --maxpush 10 --backoff 1.5 --lift 30]
Exit 0 = tool lifted, 2 = never locked, 3 = locked but lift blocked (unlocked + backed off again), 1 = watchdog.
"""
import subprocess, sys, time
import jog as J
import where as W

PY = sys.executable
TPD = 4096 / 360.0
a = sys.argv[1:]
def opt(n, d): return type(d)(a[a.index(n) + 1]) if n in a else d
X, Y0, Z = opt("--x", 51.9), opt("--y0", 234.0), opt("--z", 103.1)
ENTRY, PITCH = opt("--entry", 2487), opt("--pitch", 25.3)
MAXPUSH, BACKOFF, LIFT = opt("--maxpush", 10.0), opt("--backoff", 1.5), opt("--lift", 30.0)


def run(script, *args):
    J.port.closePort()
    try:
        r = subprocess.run([PY, script, *map(str, args)], capture_output=True, text=True)
    finally:
        J.port.openPort(); J.port.setBaudRate(1000000)
    out = r.stdout.strip().splitlines()
    print("   ", script, " ".join(map(str, args)), "->", (out[-1] if out else r.stderr.strip()[-200:])[:150])
    return r.returncode, r.stdout


def mark(label, note=""):
    subprocess.run([PY, "mark.py", label, note]); subprocess.run([PY, "capture.py", "mark", label], capture_output=True)


def face():
    return W.where([J.rd(i, 56, 2) for i in range(1, 6)])[1]


def roll_to(goal, secs=12):
    J.wr(5, 41, 1, 10); J.wr(5, 46, 2, 150); J.wr(5, 42, 2, int(goal)); t0 = time.time()
    while time.time() - t0 < secs and abs(J.rd(5, 56, 2) - goal) > 8:
        time.sleep(0.05)


def twist():
    """Twist -50 deg from entry, return (trace [(deg, load%)], locked?)."""
    r0 = J.rd(5, 56, 2); g = int(r0 - 50 * TPD)
    J.wr(5, 41, 1, 10); J.wr(5, 46, 2, 60); J.wr(5, 42, 2, g)
    tr, t0 = [], time.time()
    while time.time() - t0 < 12:
        p = J.rd(5, 56, 2); tr.append(((p - r0) / TPD, abs(J.load(5))))
        if abs(p - g) < 8: break
        time.sleep(0.08)
    ramp = max([l for d, l in tr if -38 <= d <= -18] or [0])
    end = max([l for d, l in tr if d <= -43] or [0])
    locked = ramp >= 8 and end >= 10
    print(f"    twist: {tr[-1][0]:+.0f} deg, ramp peak {ramp:.0f} %, end {end:.0f} % -> {'LOCKED' if locked else 'free'}")
    return tr, locked


def temps_ok():
    t = max(J.rd(i, 63, 1) for i in range(1, 6))
    if t >= 50: print(f"temp {t} C -> stop"); return False
    return True


def main():
    import capguard; capguard.require("pick.py")
    mark("approach")
    roll_to(ENTRY)
    run("smooth.py", "to", X, Y0, Z, "--speed", 3)
    c, out = run("seat.py", "--max", 24, "--speed", 1.5, "--contact-only")
    if "contact at" not in out:
        print("no contact: tool not in front"); return 2
    pushed = 0.0; locked = False
    while pushed <= MAXPUSH and temps_ok():
        run("smooth.py", "axis", 1.0, "--speed", 0.8, "--pan-soft", 30); pushed += 1.0
        _, locked = twist()
        if locked: break
        roll_to(ENTRY)
    if not locked:
        print(f"never locked within {MAXPUSH} mm past contact"); mark("result", "pick fail: no lock"); return 2
    y_lock = face()[1]
    mark("latch", f"pushed {pushed:.0f} mm past contact, face y {y_lock:.1f}")
    run("smooth.py", "axis", -BACKOFF, "--speed", 0.8)
    c, out = run("liftout.py", LIFT, "--pitch", PITCH, "--step", 1.5)
    J.hold()
    if "reached" in out:
        mark("lift-out", "tool lifted"); mark("result", "pick ok")
        print("PICK OK"); return 0
    print("lift blocked -> unlock + back off")
    roll_to(ENTRY); run("smooth.py", "axis", -15, "--speed", 2)
    mark("result", "pick fail: locked but lift blocked"); return 3


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        J.hold(); print("Ctrl-C -> HOLD"); sys.exit(1)
