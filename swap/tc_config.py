# SPDX-License-Identifier: GPL-3.0-or-later
"""Machine-specific settings for the swap/ scripts.  Copy config.example.json to config.json and edit it.
Environment variables override the file: TC_PORT, TC_CAPTURE_OUT, TC_CALIBRATION, TC_PYTHON."""
import json, os, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
_cfg = {}
for _name in ("config.json", "config.example.json"):
    if (HERE / _name).exists():
        _cfg = json.loads((HERE / _name).read_text(encoding="utf-8")); break

PORT = os.environ.get("TC_PORT", _cfg.get("port", "COM8"))
CAPTURE_DIR = Path(os.path.expanduser(os.environ.get("TC_CAPTURE_OUT", _cfg.get("capture_dir", "~/tc_capture"))))
CALIBRATION = os.path.expanduser(os.environ.get("TC_CALIBRATION", _cfg.get(
    "calibration", "~/.cache/huggingface/lerobot/calibration/robots/so_follower/follower_arm.json")))
CAPTURE_PYTHON = os.environ.get("TC_PYTHON", _cfg.get("capture_python") or sys.executable)
