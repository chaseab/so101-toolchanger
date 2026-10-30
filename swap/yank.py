# SPDX-License-Identifier: GPL-3.0-or-later
"""Short high-torque lift 'yank' (10/29 22:4x: three of these, tip ~20 deg up, pulled the locked tool out of the parked
slot after slow lifts stalled at 56-78 %). owner: "how about just one strong yank upwards".

    python yank.py [--n 1] [--ticks 230] [--secs 0.9] [--tl 1000]
Each yank: lift Torque_Limit -> --tl, goal = present - ticks (up), max speed/accel, hold after --secs (< the 2 s
overload window), Torque_Limit back to 800. Logs the 10 Hz trace (jog.start_trace) and one summary line per yank.
Stops if any joint >= 59 C.
"""
import json, os, sys, time
import jog as J

a = sys.argv[1:]
def opt(n, d): return type(d)(a[a.index(n) + 1]) if n in a else d
N, TICKS, SECS, TL = opt("--n", 1), opt("--ticks", 230), opt("--secs", 0.9), opt("--tl", 1000)

import capguard; take = capguard.require("yank.py")
trace = J.start_trace("yank")
for k in range(N):
    if max(J.rd(i, 63, 1) for i in range(1, 7)) >= 59:
        print("temp >= 59 C: stop"); break
    p0 = J.rd(2, 56, 2)
    J.wr(2, 48, 2, TL); J.wr(2, 41, 1, 0); J.wr(2, 46, 2, 0); J.wr(2, 42, 2, p0 - TICKS)
    pk, t0 = 0.0, time.time()
    while time.time() - t0 < SECS:
        pk = max(pk, abs(J.load(2)))
    J.hold(); J.wr(2, 48, 2, 800)
    p1 = J.rd(2, 56, 2)
    rec = {"t": time.strftime("%H:%M:%S"), "yank": k + 1, "lift_from": p0, "lift_to": p1, "moved_deg": round((p0 - p1) / J.TPD, 1),
           "peak_load": pk, "take": take, "trace": trace}
    print(json.dumps(rec))
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "yank_runs.jsonl"), "a") as f:
        f.write(json.dumps(rec) + "\n")
    time.sleep(0.4)
J.stop_trace()
