# SPDX-License-Identifier: GPL-3.0-or-later
"""Monte-Carlo over the error budget (ERROR_BUDGET in this file; sources in toolchanger/analysis/ERROR_BUDGET.md).

    python mc.py run  --design v3 --n 500 --workers 3      -> out/mc_<design>.jsonl
    python mc.py sweep --design v4 --workers 3             -> out/sweep_<design>.jsonl (capture range: dock offsets)
    python mc.py summary                                   -> out/summary.json

Each run = drop-off then pick-up of the gripper tool at the centre post (tcsim.cycle), ~15 s simulated.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
OUT = HERE / "out"

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# measured direction-dependent half-bands (deg) from the real teleop logs (sim/physics_v2/real_data/FINDINGS.md):
# pan 0.5, lift 1.6, elbow 1.4, wflex 0.1, roll 0.3 -> Coulomb torque = band x K (22 N.m/rad)
BAND_DEG = dict(shoulder_pan=0.5, shoulder_lift=1.6, elbow_flex=1.4, wrist_flex=0.1, wrist_roll=0.3)

ERROR_BUDGET = {
    "servo stiffness K (lerobot P=16)": "22 N.m/rad x lognormal(sigma 0.15)  [PROPERTIES row 30]",
    "Coulomb friction per joint": "band_deg x 22 N.m/rad x U(0.6, 1.4)  [FINDINGS hysteresis]",
    "joint repeatability (per run)": "N(0, 0.10 deg) per joint  [robonine +-0.17 deg]",
    "v3 teach state in the friction band": "U(-1, 1) x band  [leader-arm teach, plain playback]",
    "dock moved after teaching": "N(0, 0.5 mm) per axis, yaw N(0, 0.5 deg)",
    "parked rotor disturbance": "N(0, 5 deg)",
    "friction coefficients": "all x U(0.7, 1.4)  [PETG 0.30, steel 0.15, PLA 0.30-0.35 nominal]",
    "print tolerance (P1S, sigma)": "bore/spigot/plate/skirt radii 0.08 mm, neck 0.08, slot clearance 0.08 (>= 0.05), pin z 0.10, crest 0.08, dowel z 0.05, plunger protrusion 0.10",
    "spring": "k U(3.5, 6.5) N/mm, installed length +-0.3 mm",
    "tool mass": "U(90, 115) g",
}


def sample(seed, design):
    import tcsim as T
    rng = np.random.default_rng(seed)
    K = {j: 22.0 * float(np.exp(rng.normal(0, 0.15))) for j in ARM}
    fric = {j: BAND_DEG[j] * math.pi / 180 * 22.0 * float(rng.uniform(0.6, 1.4)) for j in ARM}
    geo = dict(
        R_BORE=T.G.B.R_BORE + rng.normal(0, 0.08), R_SPIG=T.G.B.R_SPIG + rng.normal(0, 0.08),
        R_TP=T.G.B.R_TP + rng.normal(0, 0.08), R_SKIRT_IN=T.G.B.R_SKIRT_IN + rng.normal(0, 0.08),
        NECK_SQ=T.G.B.NECK_SQ + rng.normal(0, 0.08), SLOT_CLR=max(0.05, T.G.B.SLOT_CLR + rng.normal(0, 0.08)),
        TOOL_PIN_Z=T.G.B.TOOL_PIN_Z + rng.normal(0, 0.10), H_CREST=T.G.B.H_CREST + rng.normal(0, 0.08),
        H_DOWEL=T.G.B.H_DOWEL + rng.normal(0, 0.05), PL_PROTRUDE=T.G.B.PL_PROTRUDE + rng.normal(0, 0.10),
        SPRING_K=float(rng.uniform(3.5, 6.5)), SPRING_INSTALLED=8.05 + float(rng.uniform(-0.3, 0.3)),
    )
    if design.startswith("v4"):     # v4 geometry is the base; apply the same deltas to it
        base = T.G.params(**T.V4)
        for k in ("R_BORE", "R_SPIG", "R_TP", "R_SKIRT_IN", "NECK_SQ", "TOOL_PIN_Z", "H_CREST", "H_DOWEL", "PL_PROTRUDE"):
            geo[k] = geo[k] - getattr(T.G.B, k) + base[k]
        geo["SLOT_CLR"] = max(0.05, geo["SLOT_CLR"] - T.G.B.SLOT_CLR + base["SLOT_CLR"])
        geo["SPRING_INSTALLED"] = geo["SPRING_INSTALLED"] - 8.05 + base["SPRING_INSTALLED"]
    err = T.Err(K=K, fric=fric,
                q_noise_deg={j: float(rng.normal(0, 0.10)) for j in ARM},
                band_frac={j: float(rng.uniform(-1, 1)) for j in ARM},
                dock_shift_mm=tuple(float(v) for v in rng.normal(0, 0.5, 3)),
                dock_yaw_deg=float(rng.normal(0, 0.5)),
                rotor_err_deg=float(rng.normal(0, 5.0)),
                mu_scale=float(rng.uniform(0.7, 1.4)),
                geo={k: float(v) for k, v in geo.items()},
                tool_mass=float(rng.uniform(0.090, 0.115)), seed=seed)
    return err


def run_one(args):
    design, seed, override = args
    import tcsim as T
    t = time.time()
    err = sample(seed, design)
    for k, v in (override or {}).items():
        setattr(err, k, v)
    try:
        S = T.make(design, err)
        r = T.cycle(S)
    except Exception as e:  # noqa: BLE001
        r = {"ok": False, "fail": f"exception: {e!r}"}
    r.pop("roll_trace", None)
    r.update(design=design, seed=seed, override=override, wall=round(time.time() - t, 1),
             err=dict(dock_shift_mm=err.dock_shift_mm, dock_yaw_deg=err.dock_yaw_deg, rotor_err_deg=err.rotor_err_deg,
                      mu_scale=err.mu_scale, K=err.K, fric=err.fric, geo=err.geo, tool_mass=err.tool_mass))
    return r


def _pool_run(jobs, out_path, workers):
    done = set()
    if out_path.exists():
        for line in out_path.read_text().splitlines():
            try:
                j = json.loads(line)
                done.add((j["seed"], json.dumps(j.get("override"), sort_keys=True)))
            except Exception:  # noqa: BLE001
                pass
    jobs = [j for j in jobs if (j[1], json.dumps(j[2], sort_keys=True)) not in done]
    print(f"{len(jobs)} runs to do ({len(done)} cached) -> {out_path.name}", flush=True)
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    t0 = time.time()
    with Pool(workers) as pool, out_path.open("a") as f:
        for i, r in enumerate(pool.imap_unordered(run_one, jobs)):
            f.write(json.dumps(r, default=float) + "\n")
            f.flush()
            if i % 10 == 0:
                print(f"{i + 1}/{len(jobs)}  {time.time() - t0:.0f} s", flush=True)


def sweep_jobs(design):
    """Capture range: nominal everything else, dock moved after teaching along one axis at a time."""
    zero = dict(q_noise_deg={j: 0.0 for j in ARM}, band_frac={j: 0.0 for j in ARM}, rotor_err_deg=0.0, mu_scale=1.0,
                dock_yaw_deg=0.0)
    jobs = []
    for axis, vals in (("x", (-4, -3, -2, -1, 1, 2, 3, 4)), ("y", (-8, -6, -4, -3, -2, 2, 3, 4, 6, 8)),
                       ("z", (-6, -4, -3, -2, 2, 3, 4, 6))):
        for v in vals:
            sh = [0.0, 0.0, 0.0]
            sh["xyz".index(axis)] = float(v)
            jobs.append((design, 0, dict(zero, dock_shift_mm=tuple(sh), geo={}, K={j: 22.0 for j in ARM},
                                         fric={j: BAND_DEG[j] * math.pi / 180 * 22.0 for j in ARM})))
    for v in (-4, -3, -2, 2, 3, 4):
        jobs.append((design, 0, dict(zero, dock_shift_mm=(0.0, 0.0, 0.0), dock_yaw_deg=float(v), geo={},
                                     K={j: 22.0 for j in ARM}, fric={j: BAND_DEG[j] * math.pi / 180 * 22.0 for j in ARM})))
    for v in (-10, 10, 20, 30, 60, 100, 140):
        jobs.append((design, 0, dict(zero, dock_shift_mm=(0.0, 0.0, 0.0), rotor_err_deg=float(v), geo={},
                                     K={j: 22.0 for j in ARM}, fric={j: BAND_DEG[j] * math.pi / 180 * 22.0 for j in ARM})))
    return jobs


def summary():
    out = {}
    for f in sorted(OUT.glob("mc_*.jsonl")) + sorted(OUT.glob("sweep_*.jsonl")):
        rows = [json.loads(l) for l in f.read_text().splitlines() if l.strip()]
        n = len(rows)
        ok = sum(r.get("ok", False) for r in rows)
        fails = {}
        for r in rows:
            if not r.get("ok"):
                fails[r.get("fail", "?")] = fails.get(r.get("fail", "?"), 0) + 1
        p = ok / n if n else 0
        # Wilson 95 % interval
        z = 1.96
        lo = (p + z * z / (2 * n) - z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / (1 + z * z / n) if n else 0
        hi = (p + z * z / (2 * n) + z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / (1 + z * z / n) if n else 0
        loads = np.array([r["load_max"] for r in rows if "load_max" in r]) if n else np.zeros((0, 5))
        roll = [r["roll_peak_Nm"] for r in rows if "roll_peak_Nm" in r]
        out[f.stem] = dict(n=n, ok=ok, rate=round(p, 4), ci95=[round(lo, 4), round(hi, 4)], fails=fails,
                           load_p95=np.percentile(loads, 95, axis=0).round(3).tolist() if len(loads) else None,
                           load_max=loads.max(axis=0).round(3).tolist() if len(loads) else None,
                           roll_peak_p95=round(float(np.percentile(roll, 95)), 3) if roll else None)
    (OUT / "summary.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["run", "sweep", "summary"])
    ap.add_argument("--design", default="v4")
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    if a.mode == "summary":
        return summary()
    if a.mode == "run":
        jobs = [(a.design, a.seed0 + i, None) for i in range(a.n)]
        _pool_run(jobs, OUT / f"mc_{a.design}{a.tag}.jsonl", a.workers)
    else:
        _pool_run(sweep_jobs(a.design), OUT / f"sweep_{a.design}{a.tag}.jsonl", a.workers)


if __name__ == "__main__":
    main()
