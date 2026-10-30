# SPDX-License-Identifier: GPL-3.0-or-later
"""Find the bore by trying insert+twist at a list of (x, z) offsets (10/29).

    python scan.py --y0 228 --y1 243 --x 51.6 --zs 99,100,101,102,103 [--xs 51.6]

For each (x, z): go to the standoff (x, y0, z) at the seat pitch, roll at entry, slide to y1 (pan soft), then twist roll
toward the lock watching the roll load. DESIGN.md: the pin climbs a ramp, ~0.92 N.m = ~30 % roll load; a free spin
(pins not through the windows) reads 3-6 %. ENGAGED = roll load >= 15 % within the twist. On a free spin: roll back to
entry, slide back to y0, next offset. On ENGAGED: stop and hold there (caller verifies with a lift).
"""
import subprocess, sys, time
import jog as J

PY = sys.executable
TPD = 4096 / 360.0
a = sys.argv[1:]
def opt(n, d): return a[a.index(n) + 1] if n in a else d
y0, y1 = float(opt("--y0", 228)), float(opt("--y1", 243))
zs = [float(v) for v in opt("--zs", "99,100,101,102,103").split(",")]
xs = [float(v) for v in opt("--xs", opt("--x", "51.6")).split(",")]
ENTRY = int(opt("--entry", 2491))


def sm(*args):
    J.port.closePort()                                   # smooth.py needs COM8 (Windows: one process per port)
    try:
        r = subprocess.run([PY, "smooth.py", *args], capture_output=True, text=True)
    finally:
        J.port.openPort(); J.port.setBaudRate(1000000)
    last = (r.stdout.strip().splitlines() or ["?"])[-1]
    if r.returncode != 0 or not last.startswith(("done", "HOLD")):
        sys.exit(f"smooth.py failed: {(r.stderr or r.stdout).strip()[-300:]}")
    print("   ", last.split("|")[0], "|", last.split("|")[2].strip() if last.count("|") >= 2 else "")
    if "HOLD" in last:
        print("   smooth HOLD:", last); return False
    return True


def roll_to(goal, watch=False, secs=12):
    J.wr(5, 41, 1, 10); J.wr(5, 46, 2, 60); J.wr(5, 42, 2, int(goal))
    peak, t0 = 0.0, time.time()
    while time.time() - t0 < secs:
        p = J.rd(5, 56, 2); l = abs(J.load(5)); peak = max(peak, l)
        if abs(p - goal) < 8:
            break
        if watch and l >= 15:
            return peak, p, True
        time.sleep(0.05)
    return peak, J.rd(5, 56, 2), False


pans = [int(v) for v in opt("--pans", "").split(",") if v]


def pan_by(dt):
    """Pan by dt ticks through its ~10-tick stiction: overshoot 6 ticks, then settle on the goal."""
    p0 = J.rd(1, 56, 2); g = p0 + dt
    J.wr(1, 46, 2, 40); J.wr(1, 42, 2, g + (6 if dt > 0 else -6)); time.sleep(1.0)
    J.wr(1, 42, 2, g); time.sleep(0.8)
    return J.rd(1, 56, 2) - p0


if pans:                                                 # pan scan: x = (pan offset), insert along the resulting axis
    x = xs[0]
    for z in zs:
        for dp in pans:
            print(f"--- z {z} pan {dp:+d} ticks (~{dp * 0.37:+.1f} mm)")
            if abs(J.rd(5, 56, 2) - ENTRY) > 10:
                roll_to(ENTRY)
            if not sm("to", str(x), str(y0), str(z), "--speed", "3"):
                sys.exit(1)
            got = pan_by(dp)
            print(f"   pan moved {got:+d} ticks")
            if not sm("axis", str(y1 - y0), "--speed", "1.5", "--pan-soft", "30"):
                print("   insert held (resistance) -> try twist anyway")
            peak, pr, eng = roll_to(ENTRY - 50 * TPD, watch=True)
            print(f"   twist: roll {(pr - ENTRY) / TPD:+.1f} deg, peak roll load {peak:.0f} % -> {'ENGAGED' if eng else 'free'}")
            if eng:
                J.wr(5, 42, 2, J.rd(5, 56, 2)); print(f"ENGAGED at z {z} pan {dp:+d}"); sys.exit(0)
            roll_to(ENTRY)
            sm("axis", str(-(y1 - y0)), "--speed", "3")
    print("no engagement in the pan scan"); sys.exit(2)

for x in xs:
    for z in zs:
        print(f"--- x {x} z {z}")
        if abs(J.rd(5, 56, 2) - ENTRY) > 10:
            roll_to(ENTRY)
        if not sm("to", str(x), str(y0), str(z), "--speed", "3"):
            sys.exit(1)
        if not sm("to", str(x), str(y1), str(z), "--speed", "1.5", "--pan-soft", "30"):
            print("   insert held (resistance) -> treat as contact, try twist anyway")
        peak, p, eng = roll_to(ENTRY - 50 * TPD, watch=True)
        print(f"   twist: roll moved {(p - ENTRY) / TPD:+.1f} deg, peak roll load {peak:.0f} %  -> {'ENGAGED' if eng else 'free'}")
        if eng:
            J.wr(5, 42, 2, J.rd(5, 56, 2))       # hold where the ramp started
            print(f"ENGAGED at x {x} z {z} (roll {(p - ENTRY) / TPD:+.1f} deg into the twist)")
            sys.exit(0)
        roll_to(ENTRY)
        sm("to", str(x), str(y0), str(z), "--speed", "3")
print("no engagement in the scan")
sys.exit(2)
