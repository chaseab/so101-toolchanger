# SPDX-License-Identifier: GPL-3.0-or-later
"""Capture guard (10/29, pm-eng root-cause fix): every scripted real-arm motion must be on a live take.

    import capguard; capguard.require("smooth.py")      # first line before any real motion

require():
  - bypass only with env TC_NO_CAPTURE=1 or "--no-capture" in sys.argv -> loud WARNING row in logs/marks.csv
  - if a take is live (capture_state.json pids alive AND the newest scene + wrist mkv both growing) -> mark + continue
  - else: C: free must be >= 17 GB (refuse otherwise), start `capture.py start`, wait <= 12 s for a growing take,
    refuse (SystemExit) if none. A take this guard started is recorded in logs/capguard_owner.json; release() by the
    same top-level script stops it.
  - every call appends "motion <script> take <stamp>" to logs/marks.csv, so each attempt maps to a take.
latest_frame(out): newest dock-cam frame from the live take (the camera itself is busy while recording).
Sim/twin (sim_io/fakebus, run_swap --dry-run) never calls require().
"""
import csv, json, os, shutil, subprocess, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = Path(os.environ.get("TC_CAPTURE_OUT", "capture_out"))
STATE = OUT / "capture_state.json"
OWNER = HERE / "logs" / "capguard_owner.json"
MARKS = HERE / "logs" / "marks.csv"
GPY = os.environ.get("TC_PYTHON", "python")   # global python (cv2, PIL): from the lerobot venv a bare "python" resolves to the venv exe, whose capture start hangs (10/30)
MIN_FREE_GB = float(os.environ.get("TC_MIN_FREE_GB", "17.0"))   # override only to test the guard


def _mark(label, note):
    MARKS.parent.mkdir(exist_ok=True)
    new = not MARKS.exists()
    with open(MARKS, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["t", "label", "note"])
        w.writerow([f"{time.time():.2f}", label, note])


def _alive(pid):
    r = subprocess.run(["powershell", "-NoProfile", "-Command", f"(Get-Process -Id {pid} -ErrorAction SilentlyContinue).Id"],
                       capture_output=True, text=True)
    return bool(r.stdout.strip())


def live_take():
    """Stamp of the live take, or None. Live = state pids alive and both scene + wrist files growing."""
    if not STATE.exists():
        return None
    st = json.loads(STATE.read_text())
    stamp = st.get("stamp")
    if not stamp or not any(_alive(p) for p in st.get("pids", [])):
        return None
    files = [f for f in (OUT / f"tc_scene_1028_{stamp}.mkv", OUT / f"tc_wrist_1028_{stamp}.mkv") if f.exists()]
    if len(files) < 2:
        return None
    s0 = [f.stat().st_size for f in files]
    grown = [False] * len(files)
    for _ in range(20):                                  # mkv flushes in chunks: look up to 10 s (a 1.5-4 s look gave false "not live")
        time.sleep(0.5)
        grown = [g or f.stat().st_size > a for g, f, a in zip(grown, files, s0)]
        if all(grown):
            return stamp
    return None


def require(script):
    if os.environ.get("TC_FAKE_BUS") == "1":                      # fake bus = no arm, nothing to film
        _mark("fake", f"{script} on TC_FAKE_BUS (no arm)"); return None
    if os.environ.get("TC_NO_CAPTURE") == "1" or "--no-capture" in sys.argv:
        _mark("WARNING", f"{script}: REAL MOTION WITHOUT CAPTURE (--no-capture / TC_NO_CAPTURE=1)")
        print(f"[capguard] WARNING: {script} running real motion WITHOUT a take", file=sys.stderr)
        return None
    stamp = live_take()
    if stamp is None and STATE.exists():                 # never start a 2nd take while the recorders still run: it clobbers
        st = json.loads(STATE.read_text())               # capture_state.json and fails on the busy cameras (10/30 16:08)
        if any(_alive(p) for p in st.get("pids", [])):
            raise SystemExit(f"[capguard] REFUSED: take {st.get('stamp')} recorders alive but files not growing, no motion")
    if stamp is None:
        free = shutil.disk_usage("C:\\").free / 1e9
        if free < MIN_FREE_GB:
            raise SystemExit(f"[capguard] REFUSED: C: {free:.1f} GB free < {MIN_FREE_GB} GB, no take, no motion")
        subprocess.Popen([GPY, str(HERE / "capture.py"), "start"],    # global python (has cv2)
                         cwd=str(HERE), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0))
        t0 = time.time()
        while time.time() - t0 < 35 and stamp is None:          # start takes ~15-25 s (scene index probe + DSHOW open)
            time.sleep(1.0)
            stamp = live_take()
        if stamp is None:
            raise SystemExit("[capguard] REFUSED: could not get a live take (both cams growing) within 35 s, no motion")
        if not OWNER.exists():
            OWNER.write_text(json.dumps({"stamp": stamp, "owner": script, "t": time.time()}))
        print(f"[capguard] started take {stamp} for {script}")
    _mark("motion", f"{script} take {stamp}")
    os.environ["TC_TAKE"] = stamp                       # traces + child processes record the verified live take
    return stamp


def release(script):
    """Stop the take if this script's guard started it (top-level attempt scripts only)."""
    if not OWNER.exists():
        return
    o = json.loads(OWNER.read_text())
    if o.get("owner") != script:
        return
    subprocess.run([GPY, str(HERE / "capture.py"), "stop"], cwd=str(HERE), capture_output=True, timeout=90)
    OWNER.unlink(missing_ok=True)
    _mark("take-stop", f"{script} take {o.get('stamp')}")
    print(f"[capguard] stopped take {o.get('stamp')}")


def latest_frame(out_path, width=640):
    """Newest dock-cam frame from the live take (the camera is busy while it records)."""
    st = json.loads(STATE.read_text())
    src = OUT / f"tc_scene_1028_{st['stamp']}.mkv"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-sseof", "-1.5", "-i", str(src), "-frames:v", "1",
                    "-vf", f"scale={width}:-1", str(out_path)], capture_output=True)
    return out_path


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "status":
        print("live take:", live_take())
    elif a and a[0] == "test":                       # start (via require) + verify + release; touches no arm
        s = require("capguard-test"); print("take", s); print("live:", live_take()); release("capguard-test")
