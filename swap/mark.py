# SPDX-License-Identifier: GPL-3.0-or-later
"""Timestamp a teleop attempt event: python mark.py <label> [note...]  -> logs/marks.csv (matches the trace's unix t)."""
import csv, sys, time
from pathlib import Path

LOG = Path(__file__).resolve().parent / "logs" / "marks.csv"
new = not LOG.exists()
with open(LOG, "a", newline="") as f:
    w = csv.writer(f)
    if new:
        w.writerow(["t", "label", "note"])
    w.writerow([f"{time.time():.2f}", sys.argv[1] if len(sys.argv) > 1 else "mark", " ".join(sys.argv[2:])])
print("marked", sys.argv[1:] or ["mark"], time.strftime("%H:%M:%S"))
