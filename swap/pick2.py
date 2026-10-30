# SPDX-License-Identifier: GPL-3.0-or-later
"""Cold PICK v2 (10/29 night), built from the runs that locked:

  1. roll to entry (wait until there)            2. standoff in front of the parked tool
  3. ONE straight push to the hard stop          (the tool slides in its slot until the dock wall stops it; the locks
                                                   at 20:28/20:51/20:57/21:35 all came after reaching the stop)
  4. twist roll to the STALL (85 % torque)        (~-57..-58 deg from entry; a fixed -47 deg left it half-turned)
  5. relax roll until its load ~0                 (owner: "reverse it until the pressure is off the motor twisting")
  6. relax the axial push (elbow) until ~0        (the plate was clamped against the wall -> slot friction)
  7. pitch: tool tip ~3 deg UP (az +0.05)         (owner: "level it where the tip points up slightly against the sag")
  8. straight vertical lift (smooth.py z, x/y/pitch held)

    python pick2.py [--x 52.5 --y0 234 --z 103 --ystop 257 --az-push 0.15 --az-lift 0.05 --lift 30]
Writes one JSON line per run to logs/pick2_runs.jsonl (the data for the fast/adaptive version).
Exit 0 = lifted, 2 = no lock (twist never stalled), 3 = locked but lift held, 1 = error.
"""
import csv, glob, json, os, subprocess, sys, time
import jog as J
import where as W

PY = sys.executable
TPD = 4096 / 360.0
a = sys.argv[1:]
def opt(n, d): return type(d)(a[a.index(n) + 1]) if n in a else d
X, Y0, Z, YSTOP = opt("--x", 52.5), opt("--y0", 234.0), opt("--z", 103.0), opt("--ystop", 257.0)
AZP, AZL, LIFT, ENTRY = opt("--az-push", 0.05), opt("--az-lift", 0.05), opt("--lift", 30.0), opt("--entry", 2487)
RUN = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "args": a}


def run(*args):
    with J.BUS:                                          # pause this process's tracer while the child owns COM8
        J.port.closePort()
        try:
            r = subprocess.run([PY, *map(str, args)], capture_output=True, text=True)
        finally:
            J.port.openPort(); J.port.setBaudRate(1000000)
    last = (r.stdout.strip().splitlines() or [r.stderr.strip()[-200:]])[-1]
    print("   ", " ".join(map(str, args))[:60], "->", last[:140])
    return last


def face():
    t = [J.rd(i, 56, 2) for i in range(1, 6)]; d, p, z = W.where(t)
    return [round(float(v), 1) for v in p], round(d[1] + d[2] + d[3], 1)


def loads():
    return {n: J.load(i) for i, n in ((2, "lift"), (3, "elbow"), (4, "flex"), (5, "roll"))}


def roll_goto(goal, speed=150, secs=15):
    J.wr(5, 48, 2, 1000); J.wr(5, 41, 1, 10); J.wr(5, 46, 2, speed); J.wr(5, 42, 2, int(goal)); t0 = time.time()
    while time.time() - t0 < secs and abs(J.rd(5, 56, 2) - goal) > 10:
        time.sleep(0.1)
    return J.rd(5, 56, 2)


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


def relax(joint, step, limit=3, n=24, wait=0.3):
    """Step a joint's goal back (toward unloading) until |load| <= limit. Direction = against the current load."""
    for k in range(n):
        l = J.load(joint)
        if abs(l) <= limit: return k
        p = J.rd(joint, 56, 2)
        J.wr(joint, 46, 2, 30); J.wr(joint, 42, 2, int(p - step if l > 0 else p + step)); time.sleep(wait)
    return n


def main():
    import capguard; RUN["take"] = capguard.require("pick2.py")
    RUN["trace"] = J.start_trace("pick2"); RUN["attempt"] = os.environ["TC_ATTEMPT"]   # subprocesses inherit TC_ATTEMPT
    RUN["roll_entry"] = roll_goto(ENTRY)
    run("smooth.py", "to", X, Y0, Z, "--az", AZP, "--speed", 2)
    run("smooth.py", "to", X, YSTOP, Z - 2, "--az", AZP, "--speed", 1.5, "--pan-soft", 30)
    RUN["at_stop"], RUN["pitch_sum_stop"] = face(); RUN["loads_stop"] = loads()
    ang, pk = twist_to_stall(); RUN["twist_deg"], RUN["twist_peak"] = round(ang, 1), pk
    print(f"    twist {ang:.1f} deg, peak {pk:.0f} %")
    if ang < -75 or pk < 30:
        RUN["result"] = "no lock (twist free)"; return 2
    RUN["roll_relax_steps"] = relax(5, 6, limit=4)
    # un-push (owner 10/29: "the goal is to undo that and get back to a neutral position"): go back to the depth where
    # the push first met the tool (elbow load rose > 8 % over its free value in the push log), so the tool floats
    push_log = max(glob.glob("logs/smooth_*.csv"), key=os.path.getmtime)
    rows = list(csv.DictReader(open(push_log)))
    base = sorted(float(r["load_elbow_flex"]) for r in rows[: max(5, len(rows) // 5)])[len(rows[: max(5, len(rows) // 5)]) // 2]
    y_c = next((float(r["face_y"]) for r in rows if float(r["load_elbow_flex"]) > base + 8), None)
    y_now = face()[0][1]
    RUN["y_first_contact"], RUN["y_stop"] = y_c, y_now
    if y_c is not None and y_now - y_c > 0.5:
        run("smooth.py", "axis", -(y_now - y_c), "--speed", 0.5)
    RUN["loads_relaxed"] = loads()
    print("    un-push:", f"contact y {y_c}, stop y {y_now}", "loads", RUN["loads_relaxed"])
    xyz, _ = face()
    run("smooth.py", "to", *xyz, "--az", AZL, "--speed", 1)
    last = run("smooth.py", "z", LIFT, "--az", AZL, "--speed", 1)
    RUN["lift_result"] = last[:160]; RUN["end"], RUN["pitch_sum_end"] = face()
    J.hold()
    if last.startswith("done"):
        RUN["result"] = "lifted"; return 0
    RUN["result"] = "locked, lift held"; return 3


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except KeyboardInterrupt:
        J.hold(); RUN["result"] = "ctrl-c"
    finally:
        with open("logs/pick2_runs.jsonl", "a") as f:
            f.write(json.dumps(RUN) + "\n")
        print("RESULT:", RUN.get("result"))
        import capguard; capguard.release("pick2.py")
    sys.exit(code)
