# SPDX-License-Identifier: GPL-3.0-or-later
"""Scripted step-and-settle camera sweep of the SO-101 follower (Live Splat Map, T5).

Runs in the lerobot venv as a child
process of the dashboard server (``POST /splat/sweep/start``), but the plan / safety-check half
of the module is importable under the system Python (numpy 1.26, no lerobot): every lerobot and
serial import lives inside :func:`connect_real`.

Safety model (DESIGN.md §10)
----------------------------
* The plan is fixed in code (:func:`plan`) and validated by :func:`check_plan` at ``--check-only``,
  at launch and in the tests: joint limits with margin, per-step deltas, jaw-tip / camera clearance
  above the table (outside the home neighbourhood) and total runtime.
* Motion is generated in software: cosine-eased interpolation at :data:`HZ` with per-step deltas
  ≤ :data:`MAX_STEP_DEG`, plus lerobot's ``max_relative_target`` as an independent hard clamp.
* A following-error watchdog, a cooperative stop flag (server response), a server-lost timeout and a
  runtime cap all lead to the same ``finally`` block: park in the rest pose, then release torque.
  If parking itself fails the arm is left **holding torque** (exit :data:`EXIT_NEEDS_ATTENTION`).
* Exit codes: 0 ok · 1 unexpected error (parked) · 2 busy/preflight · 3 connect · 4 needs attention ·
  5 not at rest pose · 6 plan rejected · 7 following error (parked) · 9 server lost (parked).
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import threading
import time
import traceback
import urllib.request
from collections import namedtuple
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

import numpy as np

DASHBOARD_DIR = Path(__file__).resolve().parents[1]
if str(DASHBOARD_DIR) not in sys.path:
    sys.path.insert(0, str(DASHBOARD_DIR))

from splat import fk  # noqa: E402  (needs DASHBOARD_DIR on sys.path when run as a script)

#: Project root holding ``robust_teleop.py`` (MotorsBus retry patch); on sys.path so `connect_real`
#: can import it — nothing from there is imported at module scope.
ROBOTICARM = str(Path(__file__).resolve().parents[1])   # robust_teleop.py (bus retry patch) is looked up here; not included
if ROBOTICARM not in sys.path:
    sys.path.append(ROBOTICARM)

# ---------------------------------------------------------------------------
# Constants (DESIGN.md §3.6 / §10) — the plan lives here, never in config
# ---------------------------------------------------------------------------

#: Software joint limits, follower degrees: calibrated half-range − 15°.
SAFE_DEG: Dict[str, float] = {
    "shoulder_pan": 93, "shoulder_lift": 84, "elbow_flex": 83, "wrist_flex": 84, "wrist_roll": 165,
}
# wrist_flex was 73; the follower's recorded calibration range is +/-88 deg and the GS tiers need up to 84
# to look steeply down (2026-09-13).  check_plan still verifies jaw-tip / camera clearance on every step.
#: Folded rest pose the arm must be in before a sweep starts (follower degrees).
REST_EXPECTED: Dict[str, float] = {
    "shoulder_pan": 0, "shoulder_lift": -95, "elbow_flex": 95, "wrist_flex": 60, "wrist_roll": 0,
}
REST_TOL_DEG = 15
#: Looser tolerance for wrist_roll (see check_rest_pose).
REST_TOL_ROLL_DEG = 90
#: wrist_flex only tucks the jaw further under the folded arm; teleop tends to park it ~75 deg (2026-09-13).
REST_TOL_WRIST_FLEX_DEG = 22
HOME_RADIUS_DEG = 25   # clearance exemption around the folded rest pose (the tip starts at the mat there)
TIP_MIN_M = 0.030
CAM_MIN_M = 0.050
#: Lifted pose between the rest pose and the sweep tiers.
TRANSIT: Dict[str, float] = {
    "shoulder_lift": -60, "elbow_flex": 40, "wrist_flex": 60, "shoulder_pan": 0, "wrist_roll": 0,
}
#: Tiers, designed against the FITTED hand-eye (2026-09-11; the camera looks along the pan tangent, 41° from
#: vertical at NEAR/FAR).  NEAR and FAR share one orientation and differ by a ~44 mm reach — a pure sideways
#: shift for the wrist camera, i.e. the ideal stereo pair at each pan angle (rotation 0°, lateral 88 %).
#: LOW is the original flatter tier (62° from vertical, 166 mm high) kept for far-bench coverage.
#: Camera height ≈ 167-176 mm, central range ≈ 225 mm, jaw tip ≥ 48 mm above the mat on all three.
TIERS = [
    ("NEAR", {"shoulder_lift": -30, "elbow_flex": 45, "wrist_flex": 70}),
    ("FAR", {"shoulder_lift": -5, "elbow_flex": 20, "wrist_flex": 70}),
    ("LOW", {"shoulder_lift": -30, "elbow_flex": 35, "wrist_flex": 55}),
]
PAN_MIN, PAN_MAX, PAN_STEP = -60, 60, 10
STEP_MOVE_S = 1.2
DWELL_S = 0.8
TRANSIT_S = 5.0
HOME_S = 4.0
HZ = 25
MAX_STEP_DEG = 1.5
FOLLOW_ERR_DEG = 12.0
#: camarm (2026-10-05): keep torque on at home instead of releasing; the camera arm tips forward off its rest when limp
HOLD_AT_HOME = False
FOLLOW_ERR_STEPS = 3
MAX_RUNTIME_S = 300.0
#: A dwell keeps holding (in DWELL_S slices) until the server confirms the mapper accepted a frame for
#: it, up to DWELL_EXTEND_MAX_S per dwell and DWELL_EXTEND_BUDGET_S per sweep.  This turns the wrist
#: camera's USB drop-outs (it re-enumerates for ~10 s when the cable is disturbed) into a delay
#: instead of a missing view.  Dry runs never extend (their poses are untrusted by design).
DWELL_EXTEND_MAX_S = 12.0
DWELL_EXTEND_BUDGET_S = 60.0
#: Keep-out cylinders in base_link metres (vertical, from the table up to ``z_max``).  The official
#: SO-ARM100 overhead-camera tower stands on the arm-base plate beside the follower; its position
#: below is an ESTIMATE (tower on the arm's +Y / URDF-left side, ~17 cm from the shoulder axis) with
#: a generous radius - measure it on your bench and adjust before widening the sweep.
#: 2026-09-11: three real sweeps of the shipped plan (pan +-60 deg, both tiers) passed within 47 mm of this
#: estimated centre without contact, so the radius was cut from 70 to 40 mm; the centre is still a guess.
#: Measure the tower on the bench and set x/y/r from the measurement before widening the sweep.
KEEPOUT: List[Dict[str, Any]] = [
    {"name": "overhead_cam_tower", "x": -0.02, "y": 0.17, "r": 0.04, "z_max": 0.70},
]
#: Arm links whose origins (and the segments between consecutive ones) are tested against KEEPOUT.
KEEPOUT_CHAIN: Tuple[str, ...] = ("shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link")
KEEPOUT_SAMPLES_PER_LINK = 4
SERVER_LOST_S = 3.0

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_BUSY = 2
EXIT_CONNECT = 3
EXIT_NEEDS_ATTENTION = 4
EXIT_REST_POSE = 5
EXIT_PLAN = 6
EXIT_FOLLOW_ERR = 7
EXIT_SERVER_LOST = 9

#: Manual hand-eye seed (DESIGN §2 fixture, side=+1, tilt 12°) — ``sweep.py`` must not import calib.py.
DEFAULT_T_GRIPPER_CAM = np.array(
    [[-1.0, 0.0, 0.0, 0.0],
     [0.0, 0.9781, -0.2079, 0.040],
     [0.0, -0.2079, -0.9781, -0.030],
     [0.0, 0.0, 0.0, 1.0]],
    dtype=np.float64,
)
BODY: List[str] = fk.ARM_JOINTS
GRIPPER = "gripper"

#: Park verification tolerance and "far from TRANSIT" threshold used by the park path (§10.5).
PARK_TOL_DEG = 5.0
PARK_VIA_TRANSIT_DEG = 20.0
#: Reporter POST rate cap (§10.5).
REPORT_HZ = 20.0

#: Park is self-pacing ("leashed"), not a timed ramp: each step commands at most PARK_LEASH_DEG ahead
#: of the MEASURED pose, so the arm moves as fast as it can under its load and the following-error
#: watchdog (FOLLOW_ERR_DEG) can never trip on gravity sag alone.  2026-09-15: a timed park from a
#: horizontal reach aborted at 13.8 deg following error in 0.6 s because lerobot's soft P gain (16,
#: set by SO101Follower.configure() for teleop feel) cannot out-pull gravity on the shoulder.
PARK_LEASH_DEG = 8.0          # < FOLLOW_ERR_DEG; also passed as max_relative_target for --park
PARK_ARRIVE_DEG = 4.0         # per-joint arrival tolerance for a leashed leg
PARK_STALL_S = 2.5            # no progress for this long -> abort, hold torque (NEEDS_ATTENTION)
PARK_STALL_MIN_DEG = 0.3      # what counts as progress
PARK_LEG_MAX_S = 60.0         # hard cap per leg
PARK_P_COEFFICIENT = 32       # Feetech default; stiffer than lerobot's 16 so the shoulder can lift
TELEOP_P_COEFFICIENT = 16     # what lerobot's SO101Follower.configure() sets; restored after park
#: Gripper opening (lerobot 0..100, 0 = jaws closed) the standalone park (--park) settles at: the leader
#: reads ~1-2 at rest with its jaws closed, so matching it keeps the next teleop/record start from
#: snapping the gripper. Not fully 0 so the jaws never clamp on themselves.
PARK_GRIPPER_PCT = 2.0

Segment = namedtuple("Segment", "target move_s dwell_s label phase")
Segment.__doc__ = (
    "One sweep leg: ``target`` = dict of the 5 body joints in follower degrees + 'gripper' pct, "
    "``move_s`` seconds of eased motion, ``dwell_s`` seconds of hold (reported as phase 'dwell'), "
    "``label`` for logs/HUD, ``phase`` in {'transit', 'move', 'park'}."
)

Pose = Dict[str, float]


def _log(msg: str) -> None:
    """One flushed log line (stdout is redirected to sweep_output.log by the server)."""
    print(f"[sweep] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Plan
# ---------------------------------------------------------------------------

def _target(body: Mapping[str, float], gripper: float) -> Pose:
    """Full 6-key pose from a 5-joint body pose and a gripper percentage (plain floats)."""
    pose = {j: float(body[j]) for j in BODY}
    pose[GRIPPER] = float(gripper)
    return pose


def _pose6(pose: Mapping[str, float]) -> Pose:
    """Validate and copy a 6-key pose (5 body joints + gripper) as plain floats."""
    missing = [j for j in BODY + [GRIPPER] if j not in pose]
    if missing:
        raise KeyError(f"pose is missing joints {missing}")
    return {j: float(pose[j]) for j in BODY + [GRIPPER]}


def _body_dist(a: Mapping[str, float], b: Mapping[str, float]) -> float:
    """Chebyshev distance over the 5 body joints (degrees)."""
    return max(abs(float(a[j]) - float(b[j])) for j in BODY)


def _limits(home_deg: Mapping[str, float]) -> Dict[str, float]:
    """Per-joint magnitude bound used by :func:`check_plan` and the command clamp in :func:`run`.

    The REAL folded rest pose (lift −95°, elbow +95°) lies outside :data:`SAFE_DEG` (84/83), so
    ``SAFE_DEG`` alone would reject every plan and — worse — clamp the first command 11° away from
    the resting arm.  The bound is therefore ``max(SAFE_DEG[j], |home_j|)``: the sweep never
    commands a joint further from centre than the safety limit *or* the pose the arm was found
    resting in.  Every planned waypoint still obeys ``SAFE_DEG`` proper; only the eased path in and
    out of the home pose uses the extension.
    """
    return {j: max(float(SAFE_DEG[j]), abs(float(home_deg[j]))) for j in BODY}


#: Calibration plan (``--mode=calib``): the same step-and-settle mechanics, but the point is JOINT
#: DIVERSITY so a hand-eye + joint-zero-offset fit is well conditioned — every tier below changes the
#: wrist pitch, reach or roll, and each is visited at three pans.  Camera stays >= 12 cm above the mat.
CALIB_TIERS: List[Tuple[str, Dict[str, float]]] = [
    ("C1", {"shoulder_lift": -30, "elbow_flex": 45, "wrist_flex": 70, "wrist_roll": 0}),     # NEAR
    ("C2", {"shoulder_lift": -5, "elbow_flex": 20, "wrist_flex": 70, "wrist_roll": 0}),      # FAR
    ("C3", {"shoulder_lift": -30, "elbow_flex": 35, "wrist_flex": 55, "wrist_roll": 0}),     # LOW
    ("C4", {"shoulder_lift": -30, "elbow_flex": 45, "wrist_flex": 55, "wrist_roll": 0}),     # NEAR, pitched up
    ("C5", {"shoulder_lift": -35, "elbow_flex": 50, "wrist_flex": 60, "wrist_roll": 0}),     # closer, pitched up
    ("C6", {"shoulder_lift": -30, "elbow_flex": 45, "wrist_flex": 70, "wrist_roll": 30}),    # NEAR rolled +
    ("C7", {"shoulder_lift": -30, "elbow_flex": 45, "wrist_flex": 70, "wrist_roll": -30}),   # NEAR rolled -
    ("C8", {"shoulder_lift": -15, "elbow_flex": 35, "wrist_flex": 60, "wrist_roll": 20}),    # mid reach, rolled
]
CALIB_PANS: List[int] = [-40, 0, 40]

#: Gaussian-splat plan (``--mode=gs``): PERSPECTIVE DIVERSITY for the trainer — the same mat patch seen from
#: different heights, ranges and tilts.  With the SO-101 the camera always looks radially outward from the
#: base, so azimuth cannot vary for a given patch; what can is height (12-22 cm), range (18-53 cm) and tilt
#: (35-63 deg from vertical), computed with the fitted hand-eye (tier_design, 2026-09-13):
#:   STEEP  z 141 mm, tilt 35 deg, looks at x ~ 300 mm     HIGH  z 215 mm, tilt 55 deg, looks at x ~ 500 mm
#:   NEAR   z 126 mm, tilt 42 deg, looks at x ~ 290 mm     LOW   z 171 mm, tilt 63 deg, looks at x ~ 530 mm
#:   MID    z 172 mm, tilt 44 deg, looks at x ~ 380 mm
GS_TIERS: List[Tuple[str, Dict[str, float]]] = [
    ("STEEP", {"shoulder_lift": -10, "elbow_flex": 20, "wrist_flex": 84}),
    ("MID", {"shoulder_lift": -10, "elbow_flex": 10, "wrist_flex": 82}),
    ("HIGH", {"shoulder_lift": -20, "elbow_flex": 5, "wrist_flex": 84}),
    ("NEAR", {"shoulder_lift": -30, "elbow_flex": 45, "wrist_flex": 70}),
    ("LOW", {"shoulder_lift": -30, "elbow_flex": 35, "wrist_flex": 55}),
]
GS_PAN_STEP = 20


def plan_gs(home_deg: Mapping[str, float]) -> List[Segment]:
    """Splat plan: every GS_TIERS pose at pans -60..60 step GS_PAN_STEP (alternating direction per tier)."""
    home = _pose6(home_deg)
    grip = home[GRIPPER]
    segments: List[Segment] = [Segment(_target(TRANSIT, grip), TRANSIT_S, 0.5, "HOME->TRANSIT", "transit")]
    pans_up = list(range(PAN_MIN, PAN_MAX + 1, GS_PAN_STEP))
    for tier_idx, (name, tier) in enumerate(GS_TIERS):
        pans = pans_up if tier_idx % 2 == 0 else list(reversed(pans_up))
        first = dict(tier, shoulder_pan=pans[0], wrist_roll=0)
        segments.append(Segment(_target(first, grip), TRANSIT_S if tier_idx == 0 else 4.0, DWELL_S, f"{name}{pans[0]:+d}", "transit"))
        for pan in pans[1:]:
            body = dict(tier, shoulder_pan=pan, wrist_roll=0)
            segments.append(Segment(_target(body, grip), STEP_MOVE_S * 1.5, DWELL_S, f"{name}{pan:+d}", "move"))
    segments.append(Segment(_target(TRANSIT, grip), TRANSIT_S, 0.5, "TRANSIT", "transit"))
    segments.append(Segment(home, HOME_S, 0.0, "HOME", "park"))
    return segments


def plan_calib(home_deg: Mapping[str, float]) -> List[Segment]:
    """Calibration plan: every CALIB_TIERS pose at CALIB_PANS (alternating direction), same dwell mechanics."""
    home = _pose6(home_deg)
    grip = home[GRIPPER]
    segments: List[Segment] = [Segment(_target(TRANSIT, grip), TRANSIT_S, 0.5, "HOME->TRANSIT", "transit")]
    for tier_idx, (name, tier) in enumerate(CALIB_TIERS):
        pans = CALIB_PANS if tier_idx % 2 == 0 else list(reversed(CALIB_PANS))
        first = dict(tier, shoulder_pan=pans[0])
        segments.append(Segment(_target(first, grip), 4.0 if tier_idx else TRANSIT_S, DWELL_S, f"{name}{pans[0]:+d}", "transit"))
        for pan in pans[1:]:
            segments.append(Segment(_target(dict(tier, shoulder_pan=pan), grip), 3.0, DWELL_S, f"{name}{pan:+d}", "move"))
    segments.append(Segment(_target(TRANSIT, grip), TRANSIT_S, 0.5, "TRANSIT", "transit"))
    segments.append(Segment(home, HOME_S, 0.0, "HOME", "park"))
    return segments


#: Servo system-ID plan (``--mode=sysid``, 2026-09-19, sim/PLAN.md E1): single-joint cosine ramps of
#: +-SYSID_AMP at three arm poses (different gravity loads) with settling dwells, so the command/observation
#: log (``--log``) separates lag, tracking gain and static droop per joint.  Same clearance mechanics as
#: every other plan; the ramps are the fastest the MAX_STEP_DEG interpolator allows (~37 deg/s peak) plus
#: one slow pass at the first pose.  The mode runs with the RECORDING servo profile (fast, P=16) so the
#: numbers describe the arm the datasets were recorded with.
SYSID_TIERS: List[Tuple[str, Dict[str, float]]] = [
    ("S1", {"shoulder_lift": -30, "elbow_flex": 45, "wrist_flex": 70, "wrist_roll": 0}),   # NEAR: mid load
    ("S2", {"shoulder_lift": -5, "elbow_flex": 20, "wrist_flex": 70, "wrist_roll": 0}),    # FAR: max shoulder load
    ("S3", {"shoulder_lift": -30, "elbow_flex": 35, "wrist_flex": 55, "wrist_roll": 0}),   # LOW: wrist pitched out
]
SYSID_AMP: Dict[str, float] = {"shoulder_pan": 20, "shoulder_lift": -10, "elbow_flex": -12, "wrist_flex": 12, "wrist_roll": 25}
# lift and elbow ramp NEGATIVE (arm rises): the positive direction takes the jaw tip under TIP_MIN_M at every tier.
SYSID_SETTLE_S = 1.2
SYSID_FAST_S = 0.5      # floor; _min_seconds raises it to keep every step under MAX_STEP_DEG
SYSID_SLOW_S = 2.0
SYSID_GRIP_OPEN = 40.0


def plan_sysid(home_deg: Mapping[str, float]) -> List[Segment]:
    """System-ID plan: at each SYSID_TIERS pose, settle, then per joint ramp +A / settle / -A / settle.

    The first tier repeats every joint at SYSID_SLOW_S and adds a gripper open/close pair; the other
    two only run the fast ramps.  Total ~2.5 min, well under MAX_RUNTIME_S."""
    home = _pose6(home_deg)
    grip = home[GRIPPER]
    segments: List[Segment] = [Segment(_target(TRANSIT, grip), TRANSIT_S, 0.5, "HOME->TRANSIT", "transit")]
    for tier_idx, (name, tier) in enumerate(SYSID_TIERS):
        base = dict(tier, shoulder_pan=0)
        segments.append(Segment(_target(base, grip), TRANSIT_S if tier_idx == 0 else 4.0, 2.0, f"{name}_base", "transit"))
        speeds = [("fast", SYSID_FAST_S), ("slow", SYSID_SLOW_S)] if tier_idx == 0 else [("fast", SYSID_FAST_S)]
        for tag, base_s in speeds:
            for j in BODY:
                up = dict(base)
                up[j] = base[j] + SYSID_AMP[j]
                t_up = _target(up, grip)
                t_base = _target(base, grip)
                segments.append(Segment(t_up, _min_seconds(t_base, t_up, base_s), SYSID_SETTLE_S, f"{name}_{j}_{tag}+", "move"))
                segments.append(Segment(t_base, _min_seconds(t_up, t_base, base_s), SYSID_SETTLE_S, f"{name}_{j}_{tag}-", "move"))
        if tier_idx == 0:
            segments.append(Segment(_target(base, SYSID_GRIP_OPEN), 1.0, SYSID_SETTLE_S, f"{name}_gripper+", "move"))
            segments.append(Segment(_target(base, grip), 1.0, SYSID_SETTLE_S, f"{name}_gripper-", "move"))
    segments.append(Segment(_target(TRANSIT, grip), TRANSIT_S, 0.5, "TRANSIT", "transit"))
    segments.append(Segment(home, HOME_S, 0.0, "HOME", "park"))
    return segments


def plan(home_deg: Mapping[str, float]) -> List[Segment]:
    """Build the 29-segment step-and-settle plan starting and ending at ``home_deg`` (§10.3).

    ``home_deg`` is the observed rest pose (5 body joints + ``'gripper'`` pct).  The gripper is held
    at its home reading and ``wrist_roll`` is 0 in every target except HOME.
    """
    home = _pose6(home_deg)
    grip = home[GRIPPER]
    segments: List[Segment] = [
        Segment(_target(TRANSIT, grip), TRANSIT_S, 0.5, "HOME->TRANSIT", "transit"),
    ]
    pans_up = list(range(PAN_MIN, PAN_MAX + 1, PAN_STEP))        # -60 .. +60
    pans_down = list(reversed(pans_up))                            # +60 .. -60
    for tier_idx, (name, tier) in enumerate(TIERS):
        pans = pans_up if tier_idx % 2 == 0 else pans_down        # MID sweeps back to save a transit
        first = dict(tier, shoulder_pan=pans[0], wrist_roll=0)
        entry_s = TRANSIT_S if tier_idx == 0 else 4.0             # tier switch is a short lift
        segments.append(Segment(_target(first, grip), entry_s, DWELL_S, f"{name}{pans[0]:+d}", "transit"))
        for pan in pans[1:]:
            body = dict(tier, shoulder_pan=pan, wrist_roll=0)
            segments.append(Segment(_target(body, grip), STEP_MOVE_S, DWELL_S, f"{name}{pan:+d}", "move"))
    segments.append(Segment(_target(TRANSIT, grip), TRANSIT_S, 0.5, "TRANSIT", "transit"))
    segments.append(Segment(home, HOME_S, 0.0, "HOME", "park"))
    return segments


def _ease_path(a: Pose, b: Pose, seconds: float, hz: int) -> List[Pose]:
    """Cosine-eased samples from ``a`` (exclusive) to ``b`` (inclusive); no delta check."""
    n = max(1, int(round(float(seconds) * hz)))
    keys = BODY + [GRIPPER]
    out: List[Pose] = []
    for i in range(1, n + 1):
        s = 0.5 - 0.5 * math.cos(math.pi * i / n)
        out.append({k: float(a[k]) + (float(b.get(k, a[k])) - float(a[k])) * s for k in keys})
    return out


def _max_step(prev: Mapping[str, float], nxt: Mapping[str, float]) -> float:
    """Largest body-joint change between two consecutive samples (degrees)."""
    return _body_dist(prev, nxt)


def interpolate(a_deg: Mapping[str, float], b_deg: Mapping[str, float], seconds: float, hz: int = HZ) -> List[Pose]:
    """Cosine-eased path from ``a_deg`` to ``b_deg`` in ``n = max(1, round(seconds·hz))`` steps.

    Every sample carries all 6 keys.  Raises ``ValueError`` if any per-step body-joint delta
    exceeds :data:`MAX_STEP_DEG` — the caller must slow the segment down instead.
    """
    a = _pose6(a_deg)
    path = _ease_path(a, dict(b_deg), seconds, hz)
    prev: Mapping[str, float] = a
    for i, pose in enumerate(path):
        step = _max_step(prev, pose)
        if step > MAX_STEP_DEG:
            raise ValueError(
                f"interpolate: step {i + 1}/{len(path)} moves {step:.2f} deg > MAX_STEP_DEG {MAX_STEP_DEG}"
            )
        prev = pose
    return path


def _min_seconds(a: Mapping[str, float], b: Mapping[str, float], base_s: float, hz: int = HZ) -> float:
    """Duration that keeps a cosine-eased move ``a → b`` comfortably under :data:`MAX_STEP_DEG`.

    The peak per-step delta of a cosine ease over ``n`` steps is ``D·π/(2n)``; we size ``n`` for
    80 % of the limit and never go below ``base_s``.
    """
    d = _body_dist(a, b)
    n_needed = d * math.pi / (2.0 * 0.8 * MAX_STEP_DEG)
    return max(float(base_s), math.ceil(n_needed) / hz)


def _q_from_deg(pose: Mapping[str, float]) -> Dict[str, float]:
    """Follower degrees → URDF radians for the 5 body joints (``q_j = DIRECTION_j · rad(deg_j)``)."""
    return {j: fk.DIRECTION[j] * math.radians(float(pose[j])) + fk.ZERO_OFFSET_RAD.get(j, 0.0) for j in BODY}


def keepout_violations(q: Mapping[str, float], T_gripper_cam: np.ndarray,
                       zones: Optional[List[Dict[str, Any]]] = None) -> List[str]:
    """Names of KEEPOUT zones that any arm link, the jaw tip or the camera enters at pose ``q``."""
    zones = KEEPOUT if zones is None else zones
    if not zones:
        return []
    T_all = fk.fk_all(q)
    pts = [T_all[l][:3, 3] for l in KEEPOUT_CHAIN]
    samples: List[np.ndarray] = []
    for a, b in zip(pts[:-1], pts[1:]):
        for k in range(KEEPOUT_SAMPLES_PER_LINK + 1):
            samples.append(a + (b - a) * (k / KEEPOUT_SAMPLES_PER_LINK))
    T_bg = T_all["gripper_link"]
    samples.append(fk.jaw_tip_base(T_bg)[:3])
    samples.append((T_bg @ T_gripper_cam)[:3, 3])
    hits: List[str] = []
    for z in zones:
        cx, cy, r, zmax = float(z["x"]), float(z["y"]), float(z["r"]), float(z["z_max"])
        for p in samples:
            if p[2] <= zmax and math.hypot(float(p[0]) - cx, float(p[1]) - cy) < r:
                hits.append(str(z.get("name", "zone")))
                break
    return hits


def check_plan(
    segments: List[Segment], home_deg: Mapping[str, float], T_gripper_cam: Optional[np.ndarray] = None
) -> List[str]:
    """Static safety audit of a plan (§10.4).  Returns human-readable violations (``[]`` = safe).

    Walks every segment from ``home_deg`` with the same interpolation :func:`run` uses and checks,
    for every sample: (1) joint magnitude ≤ :func:`_limits` (``SAFE_DEG`` away from home, extended
    only by the observed home pose near it); (2) per-step delta ≤ :data:`MAX_STEP_DEG`; (3) outside
    :data:`HOME_RADIUS_DEG` of home, jaw-tip z ≥ :data:`TIP_MIN_M` and camera-origin z ≥
    :data:`CAM_MIN_M` through ``T_gripper_cam`` (default :data:`DEFAULT_T_GRIPPER_CAM`); and
    (4) total duration ≤ :data:`MAX_RUNTIME_S`; and (5) no link, the jaw tip or the camera enters a
    :data:`KEEPOUT` cylinder (overhead-camera tower).
    """
    T = DEFAULT_T_GRIPPER_CAM if T_gripper_cam is None else np.asarray(T_gripper_cam, dtype=np.float64)
    problems: List[str] = []
    try:
        home = _pose6(home_deg)
    except KeyError as exc:
        return [f"home pose: {exc}"]
    limits = _limits(home)
    current: Pose = home
    total_s = 0.0
    for idx, seg in enumerate(segments):
        total_s += float(seg.move_s) + float(seg.dwell_s)
        tag = f"segment {idx + 1} '{seg.label}'"
        if seg.phase not in ("transit", "move", "park"):
            problems.append(f"{tag}: bad phase {seg.phase!r}")
        try:
            target = _pose6(seg.target)
        except KeyError as exc:
            problems.append(f"{tag}: {exc}")
            continue
        for j in BODY:
            if abs(target[j]) > SAFE_DEG[j] and seg.phase != "park":
                problems.append(f"{tag}: target {j}={target[j]:.1f} exceeds SAFE_DEG {SAFE_DEG[j]}")
        path = _ease_path(current, target, seg.move_s, HZ)
        prev = current
        for i, pose in enumerate(path):
            step_tag = f"{tag} step {i + 1}/{len(path)}"
            for j in BODY:
                if abs(pose[j]) > limits[j]:
                    problems.append(f"{step_tag}: {j}={pose[j]:.2f} exceeds limit {limits[j]:.1f}")
            step = _max_step(prev, pose)
            if step > MAX_STEP_DEG:
                problems.append(f"{step_tag}: per-step delta {step:.2f} > {MAX_STEP_DEG}")
            if _body_dist(pose, home) > HOME_RADIUS_DEG:
                q = _q_from_deg(pose)
                T_bg = fk.fk_gripper(q)
                tip_z = float(fk.jaw_tip_base(T_bg)[2])
                cam_z = float((T_bg @ T)[2, 3])
                if tip_z < TIP_MIN_M:
                    problems.append(f"{step_tag}: jaw tip z {tip_z * 1000:.0f} mm < {TIP_MIN_M * 1000:.0f} mm")
                if cam_z < CAM_MIN_M:
                    problems.append(f"{step_tag}: camera z {cam_z * 1000:.0f} mm < {CAM_MIN_M * 1000:.0f} mm")
                for zone in keepout_violations(q, T):
                    problems.append(f"{step_tag}: arm enters keep-out zone '{zone}'")
            prev = pose
        current = target
    if total_s > MAX_RUNTIME_S:
        problems.append(f"plan duration {total_s:.1f} s > MAX_RUNTIME_S {MAX_RUNTIME_S}")
    return problems


def check_rest_pose(obs_deg: Mapping[str, float]) -> List[str]:
    """Violations if the body joints of ``obs_deg`` are not within :data:`REST_TOL_DEG` of REST_EXPECTED.

    Accepts plain joint keys (``'shoulder_lift'``) or lerobot keys (``'shoulder_lift.pos'``).
    """
    problems: List[str] = []
    for j in BODY:
        val = obs_deg.get(j, obs_deg.get(f"{j}.pos"))
        if val is None:
            problems.append(f"{j}: missing from observation")
            continue
        err = abs(float(val) - REST_EXPECTED[j])
        # wrist_roll is a continuous joint that does not change the folded pose's clearance,
        # and every sweep target commands it to 0, so it only gets the wide tolerance.
        tol = REST_TOL_ROLL_DEG if j == "wrist_roll" else (REST_TOL_WRIST_FLEX_DEG if j == "wrist_flex" else REST_TOL_DEG)
        if err > tol:
            problems.append(
                f"{j}: {float(val):.1f} deg is {err:.1f} deg from rest {REST_EXPECTED[j]} (tol {tol})"
            )
    return problems


def home_from_obs(obs: Mapping[str, float]) -> Pose:
    """lerobot ``get_observation()`` (``'<j>.pos'`` degrees, ``'gripper.pos'`` 0..100) → 6-key pose."""
    return _pose6({**{j: obs[f"{j}.pos"] for j in BODY}, GRIPPER: obs["gripper.pos"]})


# ---------------------------------------------------------------------------
# Robot / server doubles and the real server link
# ---------------------------------------------------------------------------

class _FakeBus:
    """Stand-in for ``robot.bus``; records ``disconnect`` calls."""

    def __init__(self) -> None:
        self.disconnect_calls: List[bool] = []
        self.writes: List[tuple] = []          # (register, {motor: value}) from sync_write

    def disconnect(self, disable_torque: bool = True) -> None:
        self.disconnect_calls.append(bool(disable_torque))

    def sync_write(self, register: str, values: Mapping[str, Any], normalize: bool = True) -> None:
        self.writes.append((register, dict(values)))


class FakeRobot:
    """``--dry-run`` robot: no serial, tracks commanded poses so ``get_observation`` follows perfectly.

    ``follow_error_after``: after that many ``send_action`` calls report shoulder_lift 30° off the
    command for :data:`FOLLOW_ERR_WINDOW` calls (a momentary obstruction: trips the watchdog, then
    clears so the park can be verified).  ``park_fails``: raise from ``send_action`` while the phase
    is ``'park'`` (exercises the hold-torque path).  :func:`run` announces phase changes through
    :meth:`set_phase`.
    """

    FOLLOW_ERR_WINDOW = FOLLOW_ERR_STEPS + 2

    def __init__(self, follow_error_after: Optional[int] = None, park_fails: bool = False,
                 lag_deg_per_step: Optional[float] = None, stalled_joint: Optional[str] = None) -> None:
        self._pose: Dict[str, float] = {f"{j}.pos": float(REST_EXPECTED[j]) for j in BODY}
        self._pose["gripper.pos"] = 10.0
        self.follow_error_after = follow_error_after
        self.park_fails = park_fails
        # a loaded servo: each send moves every joint at most this far toward its command (None = perfect)
        self.lag_deg_per_step = lag_deg_per_step
        # a joint that cannot move at all (gravity-locked shoulder): exercises the park stall detector
        self.stalled_joint = stalled_joint
        self.phase: Optional[str] = None
        self.sends = 0
        self.disconnect_calls = 0
        self.is_calibrated = True
        self.motors: List[str] = list(fk.ALL_JOINTS)
        self.bus = _FakeBus()

    def set_phase(self, phase: str) -> None:
        self.phase = phase

    def get_observation(self) -> Dict[str, float]:
        obs = dict(self._pose)
        if (self.follow_error_after is not None
                and self.follow_error_after <= self.sends < self.follow_error_after + self.FOLLOW_ERR_WINDOW):
            obs["shoulder_lift.pos"] += 30.0
        return obs

    def send_action(self, action: Mapping[str, float]) -> Dict[str, float]:
        if self.park_fails and self.phase == "park":
            raise RuntimeError("FakeRobot: simulated bus failure during park")
        self.sends += 1
        for key, val in action.items():
            if not key.endswith(".pos"):
                continue
            if self.stalled_joint is not None and key == f"{self.stalled_joint}.pos":
                continue
            if self.lag_deg_per_step is not None:
                delta = float(val) - self._pose[key]
                self._pose[key] += max(-self.lag_deg_per_step, min(self.lag_deg_per_step, delta))
            else:
                self._pose[key] = float(val)
        return dict(action)

    def disconnect(self) -> None:
        self.disconnect_calls += 1


class ServerLink:
    """Minimal stdlib HTTP client for the dashboard (``urllib.request`` only, per G10)."""

    def __init__(self, server_url: str, token: str) -> None:
        self.server_url = server_url.rstrip("/")
        self.token = token

    def get_json(self, path: str) -> Dict[str, Any]:
        """``GET <server><path>`` → parsed JSON dict (2 s timeout; raises on any failure)."""
        with urllib.request.urlopen(self.server_url + path, timeout=2) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        return data if isinstance(data, dict) else {"value": data}

    def report(self, payload: Mapping[str, Any]) -> Dict[str, Any]:
        """``POST /splat/sweep/pose`` (1 s timeout).  Non-200 (e.g. 409 bad_token) raises ``HTTPError``."""
        body = json.dumps(dict(payload)).encode("utf-8")
        req = urllib.request.Request(
            self.server_url + "/splat/sweep/pose", data=body, method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            if resp.status != 200:
                raise RuntimeError(f"report: HTTP {resp.status}")
            data = json.loads(resp.read().decode("utf-8"))
        return data if isinstance(data, dict) else {}


class FakeLink:
    """Test double for :class:`ServerLink`: records payloads, scripts stop / failure / status."""

    DEFAULT_STATUS: Dict[str, Dict[str, Any]] = {
        "/teleop/status": {"running": False},
        "/training/record/status": {"running": False},
        "/servo/status": {"port": "(simulated)"},
        "/splat/sweep/status": {"running": True},
    }

    def __init__(
        self,
        stop_after: Optional[int] = None,
        fail_after: Optional[int] = None,
        status: Optional[Mapping[str, Mapping[str, Any]]] = None,
        token: str = "test-token",
        dwell_hit_after: Optional[int] = None,
    ) -> None:
        self.stop_after = stop_after
        self.fail_after = fail_after
        self.dwell_hit_after = dwell_hit_after
        self.token = token
        self.status: Dict[str, Dict[str, Any]] = {k: dict(v) for k, v in self.DEFAULT_STATUS.items()}
        for path, val in (status or {}).items():
            self.status[path] = dict(val)
        self.payloads: List[Dict[str, Any]] = []
        self.n = 0

    def get_json(self, path: str) -> Dict[str, Any]:
        return dict(self.status.get(path, {}))

    def report(self, payload: Mapping[str, Any]) -> Dict[str, Any]:
        self.n += 1
        self.payloads.append(dict(payload))
        if self.fail_after is not None and self.n >= self.fail_after:
            raise RuntimeError("FakeLink: simulated report failure")
        stop = self.stop_after is not None and self.n >= self.stop_after
        hit = self.dwell_hit_after is None or self.n >= self.dwell_hit_after
        return {"ok": True, "stop_requested": stop, "ingest_enabled": True, "dwell_hit": hit, "camera_alive": True}


# ---------------------------------------------------------------------------
# run(): reporter thread + control loop + guaranteed park
# ---------------------------------------------------------------------------

class _Abort(Exception):
    """Controlled end of the sweep (stop request, runtime cap, following error, server lost)."""

    def __init__(self, code: int, reason: str) -> None:
        super().__init__(reason)
        self.code = code
        self.reason = reason


class _Reporter(threading.Thread):
    """Daemon thread posting the latest pose to the server at ≤ :data:`REPORT_HZ` (latest wins)."""

    def __init__(self, link: Any, clock: Callable[[], float], sleep: Callable[[float], None],
                 stop_event: threading.Event) -> None:
        super().__init__(name="sweep-reporter", daemon=True)
        self._link = link
        self._clock = clock
        self._sleep = sleep
        self._stop_event = stop_event
        self._q: "queue.Queue[Optional[Dict[str, Any]]]" = queue.Queue(maxsize=2)
        self._taken = threading.Event()          # set whenever the thread takes an item
        self._last_post: Optional[float] = None
        self.last_ok: float = clock()
        self.failed_since_ok = False
        self.failures = 0
        self.sent = 0
        self.last_resp: Dict[str, Any] = {}

    GRACE_S = 0.005

    def submit(self, payload: Dict[str, Any]) -> None:
        """Enqueue, latest wins.  Never blocks longer than :data:`GRACE_S`.

        When the queue is full the control thread waits up to ``GRACE_S`` for the reporter to take
        an item (this also hands the GIL over, which a bare ``sleep(0)`` does not guarantee) and
        only then discards the oldest pending report.
        """
        try:
            self._q.put_nowait(payload)
            return
        except queue.Full:
            pass
        self._taken.clear()
        self._taken.wait(self.GRACE_S)
        try:
            self._q.put_nowait(payload)
            return
        except queue.Full:
            pass
        try:
            self._q.get_nowait()
        except queue.Empty:
            pass
        try:
            self._q.put_nowait(payload)
        except queue.Full:
            pass

    def close(self, timeout: float = 5.0) -> None:
        """Send the sentinel and join."""
        try:
            self._q.put(None, timeout=timeout)
        except queue.Full:
            pass
        self.join(timeout=timeout)

    def _pace(self) -> None:
        now = self._clock()
        if self._last_post is not None:
            wait = self._last_post + 1.0 / REPORT_HZ - now
            if wait > 0:
                self._sleep(wait)
        self._last_post = self._clock()

    def run(self) -> None:
        while True:
            item = self._q.get()
            self._taken.set()
            if item is None:
                return
            self._pace()
            try:
                resp = self._link.report(item)
            except Exception as exc:  # network / HTTPError 409 — the main loop decides when it is fatal
                self.failures += 1
                if not self.failed_since_ok:
                    _log(f"report failed: {exc!r}")
                self.failed_since_ok = True
                continue
            self.sent += 1
            self.last_ok = self._clock()
            self.failed_since_ok = False
            self.last_resp = dict(resp)
            if resp.get("stop_requested"):
                self._stop_event.set()


class _Sweep:
    """State of one :func:`run` invocation (kept off the module namespace for clarity)."""

    def __init__(self, robot: Any, link: Any, home: Pose, dry_run: bool,
                 clock: Callable[[], float], sleep: Callable[[float], None],
                 log_path: Optional[str] = None, extend_dwells: bool = True) -> None:
        self.robot = robot
        self.extend_dwells = bool(extend_dwells)
        self.log_file: Optional[Any] = None
        if log_path:
            Path(log_path).parent.mkdir(parents=True, exist_ok=True)
            self.log_file = open(log_path, "w", encoding="utf-8", buffering=1)
            keys = BODY + [GRIPPER]
            self.log_file.write("t,phase,label," + ",".join("cmd_" + j for j in keys)
                                + "," + ",".join("obs_" + j for j in keys) + "\n")
        self.home = home
        self.dry_run = bool(dry_run)
        self.clock = clock
        self.sleep = sleep
        self.token = str(getattr(link, "token", ""))
        self.limits = _limits(home)
        self.stop_event = threading.Event()
        self.reporter = _Reporter(link, clock, sleep, self.stop_event)
        self.current: Pose = dict(home)        # last commanded pose (after clamping)
        self.t0 = clock()
        self.next_t = self.t0                  # absolute schedule of the next control step
        self.follow_bad = 0
        self.phase: Optional[str] = None
        self._notify_phase = getattr(robot, "set_phase", None)
        self.extend_used_s = 0.0
        self.extended_dwells: List[str] = []

    # -- one control step ---------------------------------------------------
    def _set_phase(self, phase: str) -> None:
        if phase != self.phase:
            self.phase = phase
            if callable(self._notify_phase):
                self._notify_phase(phase)

    def _payload(self, obs: Mapping[str, float], phase: str, label: str, progress: float) -> Dict[str, Any]:
        deg5 = {j: float(obs[f"{j}.pos"]) for j in BODY}
        gripper_pct = float(obs["gripper.pos"])
        return {
            "token": self.token,
            "joints": fk.follower_deg_to_leader_ticks(deg5, gripper_pct),
            "deg": deg5,
            "gripper_pct": gripper_pct,
            "t": float(time.time()),
            "phase": phase,
            "label": label,
            "progress": float(progress),
            "dry_run": self.dry_run,
        }

    def _step(self, pose: Mapping[str, float], phase: str, label: str, progress: float, guarded: bool) -> None:
        """Command one sample, read back, watch the following error, report, then wait for the next tick.

        Ticks are scheduled on an absolute 1/HZ grid (a slow step shortens the next wait instead of
        shifting the whole plan).  ``guarded`` enables the stop / runtime / server-lost checks (off
        while parking).
        """
        self._set_phase(phase)
        cmd = {j: max(-self.limits[j], min(self.limits[j], float(pose[j]))) for j in BODY}
        cmd[GRIPPER] = float(pose[GRIPPER])
        sent = self.robot.send_action({f"{j}.pos": v for j, v in cmd.items()})
        self.current = cmd
        obs = self.robot.get_observation()
        if self.log_file is not None:
            keys = BODY + [GRIPPER]
            # log what lerobot actually SENT (after its max_relative_target clamp), not the pre-clamp target:
            # the 2026-09-19 sysid run logged targets and the 5 deg leash made the servo lag look 2x longer
            if isinstance(sent, Mapping) and all(j + ".pos" in sent for j in keys):
                logged = {j: float(sent[j + ".pos"]) for j in keys}
            else:
                logged = cmd
            self.log_file.write("%.4f,%s,%s," % (self.clock() - self.t0, phase, label)
                                + ",".join("%.3f" % logged[j] for j in keys) + ","
                                + ",".join("%.3f" % float(obs[j + ".pos"]) for j in keys) + "\n")
        err = max(abs(float(obs[f"{j}.pos"]) - cmd[j]) for j in BODY)
        self.follow_bad = self.follow_bad + 1 if err > FOLLOW_ERR_DEG else 0
        if self.follow_bad >= FOLLOW_ERR_STEPS:
            raise _Abort(EXIT_FOLLOW_ERR, f"following error {err:.1f} deg on {FOLLOW_ERR_STEPS} steps ({label})")
        self.reporter.submit(self._payload(obs, phase, label, progress))
        if guarded:
            if self.stop_event.is_set():
                raise _Abort(EXIT_OK, "stop requested by server")
            now = self.clock()
            if now - self.t0 > MAX_RUNTIME_S:
                raise _Abort(EXIT_OK, f"runtime cap {MAX_RUNTIME_S:.0f} s reached")
            if self.reporter.failed_since_ok and now - self.reporter.last_ok > SERVER_LOST_S:
                raise _Abort(EXIT_SERVER_LOST, f"no successful report for {now - self.reporter.last_ok:.1f} s")
        self.next_t += 1.0 / HZ
        delay = self.next_t - self.clock()
        if delay > 0:
            self.sleep(delay)
        else:                                  # fell behind (serial hiccup): resync, never burst
            self.next_t = self.clock()

    def _move(self, target: Pose, seconds: float, phase: str, label: str, progress: float, guarded: bool) -> None:
        for pose in interpolate(self.current, target, seconds):
            self._step(pose, phase, label, progress, guarded)

    # -- the plan -------------------------------------------------------------
    def execute(self, segments: List[Segment]) -> None:
        n = len(segments)
        for idx, seg in enumerate(segments):
            progress = idx / n
            _log(f"segment {idx + 1}/{n} {seg.label} ({seg.phase}) move {seg.move_s:.1f} s dwell {seg.dwell_s:.1f} s")
            target = _pose6(seg.target)
            self._move(target, seg.move_s, seg.phase, seg.label, progress, guarded=True)
            steps = int(round(float(seg.dwell_s) * HZ))
            for _ in range(steps):
                self._step(target, "dwell", seg.label, progress, guarded=True)
            if steps > 0 and not self.dry_run and self.extend_dwells:
                self._extend_dwell(target, seg.label, progress)

    def _extend_dwell(self, target: Pose, label: str, progress: float) -> None:
        """Keep holding until the server reports ``dwell_hit`` for ``label`` (bounded, see DWELL_EXTEND_*)."""
        budget = min(DWELL_EXTEND_MAX_S, DWELL_EXTEND_BUDGET_S - self.extend_used_s)
        if budget <= 0:
            return
        t_start = self.clock()
        extended = False
        while not self.reporter.last_resp.get("dwell_hit", True):
            if self.clock() - t_start >= budget:
                _log(f"dwell {label}: no frame accepted after +{budget:.0f} s, moving on")
                break
            extended = True
            self._step(target, "dwell", label, progress, guarded=True)
        used = self.clock() - t_start
        self.extend_used_s += used
        if extended:
            self.extended_dwells.append(label)
            _log(f"dwell {label}: extended by {used:.1f} s waiting for the wrist camera")

    # -- park -----------------------------------------------------------------
    def _observed(self) -> Pose:
        obs = self.robot.get_observation()
        pose = {j: float(obs[f"{j}.pos"]) for j in BODY}
        pose[GRIPPER] = float(obs["gripper.pos"])
        return pose

    def _move_leashed(self, target: Pose, label: str) -> None:
        """Self-pacing park leg: every step commands ≤ PARK_LEASH_DEG beyond the measured pose, toward
        ``target``, until every body joint is within PARK_ARRIVE_DEG.  Speed comes from the servo (and
        the slow profile), never from a schedule, so a heavily loaded joint just takes longer.  Aborts
        (holding torque) only when there is no progress for PARK_STALL_S or after PARK_LEG_MAX_S."""
        t_start = self.clock()
        best = math.inf
        t_best = t_start
        while True:
            cur = self._observed()
            dist = max(abs(target[j] - cur[j]) for j in BODY)
            now = self.clock()
            if dist <= PARK_ARRIVE_DEG:
                self._step(target, "park", label, 1.0, guarded=False)   # settle exactly on the target
                return
            if dist < best - PARK_STALL_MIN_DEG:
                best, t_best = dist, now
            elif now - t_best > PARK_STALL_S:
                raise _Abort(EXIT_FOLLOW_ERR, f"park stalled {dist:.1f} deg from {label} for {now - t_best:.1f} s")
            if now - t_start > PARK_LEG_MAX_S:
                raise _Abort(EXIT_FOLLOW_ERR, f"park leg {label} exceeded {PARK_LEG_MAX_S:.0f} s ({dist:.1f} deg left)")
            cmd = {j: cur[j] + max(-PARK_LEASH_DEG, min(PARK_LEASH_DEG, target[j] - cur[j])) for j in BODY}
            cmd[GRIPPER] = target[GRIPPER]
            self._step(cmd, "park", label, 1.0, guarded=False)

    def park(self) -> None:
        """Return to home (via TRANSIT when far away) with phase 'park'; raises if the arm did not arrive.

        Both legs are leashed (see :meth:`_move_leashed`), so this works from any pose the arm can
        physically lift itself out of - including a torqued mid-air hold left by an aborted session."""
        self.follow_bad = 0
        self.current = self._observed()          # a held arm may sit far from the last commanded pose
        dist = _body_dist(self.current, self.home)
        if dist > HOME_RADIUS_DEG:
            transit = _target(TRANSIT, self.home[GRIPPER])
            if _body_dist(self.current, transit) > PARK_VIA_TRANSIT_DEG:
                _log(f"park: via TRANSIT ({_body_dist(self.current, transit):.1f} deg away, leashed)")
                self._move_leashed(transit, "TRANSIT")
            dist = _body_dist(self.current, self.home)
        if dist > 0.0:
            _log(f"park: to HOME ({dist:.1f} deg away, leashed)")
            self._move_leashed(self.home, "HOME")
        obs = self.robot.get_observation()
        arrived = _body_dist({j: obs[f"{j}.pos"] for j in BODY}, self.home)
        if arrived > PARK_TOL_DEG:
            raise RuntimeError(f"park: arm is {arrived:.1f} deg from home after parking")
        self.reporter.submit(self._payload(obs, "park", "HOME", 1.0))
        _log(f"park: at home (max error {arrived:.1f} deg)")


def run(robot: Any, link: Any, home_deg: Mapping[str, float], dry_run: bool,
        clock: Callable[[], float] = time.monotonic, sleep: Callable[[float], None] = time.sleep,
        segments: Optional[List[Segment]] = None, log_path: Optional[str] = None,
        extend_dwells: bool = True, sweep_cls: Optional[Callable[..., "_Sweep"]] = None) -> int:
    """Execute the whole sweep on an already-connected ``robot`` (§10.5); returns the exit code.

    Whatever happens inside the control loop, the ``finally`` block parks the arm and only then
    releases torque via ``robot.disconnect()``.  If parking fails the arm keeps holding torque
    (``robot.bus.disconnect(disable_torque=False)``) and the exit code is :data:`EXIT_NEEDS_ATTENTION`.

    ``sweep_cls`` (2026-10-24, toolchanger RealIO): a ``_Sweep`` subclass/factory to run instead of
    ``_Sweep`` (same constructor).  Its ``park_gain`` attribute (default :data:`PARK_P_COEFFICIENT`) is
    the P gain written before the finally-park; ``None`` skips that write (the tool changer holds in
    place while a tool is engaged with a dock, so it must not stiffen the arm against the dock).
    """
    home = _pose6(home_deg)
    sweep = (sweep_cls or _Sweep)(robot, link, home, dry_run, clock, sleep, log_path=log_path,
                                  extend_dwells=extend_dwells)
    segments = plan(home) if segments is None else list(segments)
    exit_code = EXIT_OK
    sweep.reporter.start()
    _log(f"run: {len(segments)} segments, dry_run={sweep.dry_run}")
    try:
        sweep.execute(segments)
        _log("plan complete")
    except _Abort as abort:
        exit_code = abort.code
        _log(f"abort: {abort.reason} (exit {exit_code})")
    except Exception:
        exit_code = EXIT_ERROR
        _log("unexpected error in control loop:\n" + traceback.format_exc())
    finally:
        try:
            park_gain = getattr(sweep, "park_gain", PARK_P_COEFFICIENT)
            if park_gain:
                set_body_p_gain(robot, park_gain)
            try:
                sweep.park()
            except Exception:
                _log("park failed — holding torque:\n" + traceback.format_exc())
                exit_code = EXIT_NEEDS_ATTENTION
            if park_gain:      # a sweep that holds in place (park_gain None) keeps the gain it is holding with
                set_body_p_gain(robot, TELEOP_P_COEFFICIENT)
            if not sweep.dry_run:
                # FakeRobot has no bus, and servo_profile's import pulls in lerobot/torch: a cold import in
                # the teardown of a dry run exceeded the server's 20 s silence watchdog (2026-09-19).
                restore_fast_profile(robot)
            if exit_code == EXIT_NEEDS_ATTENTION:
                try:
                    robot.bus.disconnect(disable_torque=False)
                except Exception as exc:
                    _log(f"bus.disconnect(disable_torque=False) failed: {exc!r}")
                _log("NEEDS_ATTENTION: follower left holding torque; support the arm and park it by hand")
            elif HOLD_AT_HOME and not sweep.dry_run:
                try:
                    robot.bus.disconnect(disable_torque=False)
                    _log("torque HELD at home (HOLD_AT_HOME)")
                except Exception as exc:
                    exit_code = EXIT_NEEDS_ATTENTION
                    _log(f"NEEDS_ATTENTION: disconnect failed ({exc!r}); torque state unknown")
            else:
                try:
                    robot.disconnect()
                    _log("torque released at home")
                except Exception as exc:
                    exit_code = EXIT_NEEDS_ATTENTION
                    _log(f"NEEDS_ATTENTION: disconnect failed ({exc!r}); torque state unknown")
        finally:
            sweep.reporter.close()
            if sweep.log_file is not None:
                sweep.log_file.close()
                _log(f"command/observation log written: {log_path}")
    _log(f"exit {exit_code} (reports sent {sweep.reporter.sent}, failed {sweep.reporter.failures})")
    return exit_code


# ---------------------------------------------------------------------------
# Real hardware (lerobot venv only)
# ---------------------------------------------------------------------------

def restore_fast_profile(robot: Any) -> None:
    """Undo connect_real()'s slow profile (RAM registers survive reconnects until a power cycle;
    lerobot's own connect never resets them - a stale slow profile capped the follower at ~26 deg/s
    during a recording on 2026-09-14).  No torque change, no motion; failures only logged."""
    try:
        from splat import servo_profile
        servo_profile.ensure_fast_profile(robot.bus, _log)
    except Exception as exc:
        _log(f"servo profile restore failed (continuing): {exc!r}")


def set_body_p_gain(robot: Any, value: int) -> bool:
    """Write P_Coefficient to the five body servos (RAM register; lerobot's connect() rewrites it to
    16 next session anyway).  Returns False (and logs) if the bus refused - the park then runs at
    whatever gain is in place.  Never touches torque or goal positions."""
    try:
        robot.bus.sync_write("P_Coefficient", {m: int(value) for m in BODY}, normalize=False)
        _log(f"servo P gain -> {value} on {len(BODY)} body joints")
        return True
    except Exception as exc:
        _log(f"P gain write failed (continuing): {exc!r}")
        return False


def connect_real(port: str, robot_id: str, max_relative_target: float = 5.0) -> Any:
    """Build and connect the real ``SO101Follower`` without prompts, with a slow servo profile.

    Imports ``robust_teleop`` first (its import applies the ``MotorsBus`` retry patch; its
    ``__main__`` guard keeps it from starting teleop).  ``connect(calibrate=False)`` never calls
    ``input()``; when the motors' EPROM does not match the calibration file it is written
    explicitly.  Torque is ON when this returns.
    """
    import robust_teleop  # noqa: F401  (applies the MotorsBus retry patch at import; found via ROBOTICARM)
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    cfg = SO101FollowerConfig(
        port=port, id=robot_id, use_degrees=True,
        max_relative_target=float(max_relative_target),   # float — an int raises TypeError in lerobot
        disable_torque_on_disconnect=True,
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    if not robot.is_calibrated:
        if not getattr(robot, "calibration", None):
            robot.disconnect()
            raise RuntimeError(f"no calibration file for id '{robot_id}' — refusing to move an uncalibrated arm")
        _log("motor calibration differs from file; writing calibration")
        with robot.bus.torque_disabled():
            robot.bus.write_calibration(robot.calibration)
    try:
        for m in robot.bus.motors:
            robot.bus.write("Acceleration", m, 20)
        robot.bus.sync_write("Goal_Velocity", {m: 300 for m in robot.bus.motors}, normalize=False)
        _log("slow servo profile set (Acceleration 20, Goal_Velocity 300)")
    except Exception as exc:  # cosmetic: software interpolation is the primary speed limit
        _log(f"servo profile write failed (continuing): {exc!r}")
    return robot


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _preflight(link: ServerLink, port: str) -> Optional[str]:
    """Ask the server whether we may open the follower port; returns a refusal reason or None."""
    try:
        teleop = link.get_json("/teleop/status")
        record = link.get_json("/training/record/status")
        servo = link.get_json("/servo/status")
        sweep = link.get_json("/splat/sweep/status")
    except Exception as exc:
        return f"server unreachable: {exc!r}"
    if teleop.get("running"):
        return "teleop is running"
    if record.get("running"):
        return "recording is running"
    if servo.get("port") == port:
        return f"dashboard servo bus owns {port}"
    if not sweep.get("running"):
        return "server reports no sweep in progress (launch via POST /splat/sweep/start)"
    return None


def park_only(port: str, robot_id: str, link: Any) -> int:
    """Standalone safe park (``--park``): refuses while teleop/recording own the port, then reuses
    :meth:`_Sweep.park` and the same torque-hold-on-failure rule as :func:`run`.

    With a ``--token`` (``POST /splat/sweep/park`` launches it that way) the park's poses are reported
    through ``link`` exactly like a sweep's, so ``GET /splat/sweep/status`` shows phase ``'park'`` and the
    twin follows the arm; a bare CLI run (no token) reports nothing.  Reports never gate the motion
    (``park()`` runs unguarded), so a dead server cannot strand the arm mid-air."""
    reporting = bool(getattr(link, "token", ""))
    try:
        teleop = link.get_json("/teleop/status"); record = link.get_json("/training/record/status")
        servo = link.get_json("/servo/status")
        if teleop.get("running") or record.get("running") or servo.get("port") == port:
            _log(f"refusing to park: port {port} is in use (exit {EXIT_BUSY})")
            return EXIT_BUSY
    except Exception as exc:
        _log(f"park: server check failed ({exc!r}); continuing without it")
    try:
        robot = connect_real(port, robot_id, max_relative_target=PARK_LEASH_DEG)
    except Exception:
        _log(f"connect failed (exit {EXIT_CONNECT}):\n" + traceback.format_exc())
        return EXIT_CONNECT
    set_body_p_gain(robot, PARK_P_COEFFICIENT)   # stiff enough to lift a horizontal reach (see PARK_LEASH_DEG)
    rest = _pose6(dict(REST_EXPECTED, gripper=10.0))
    sweep: Optional[_Sweep] = None
    try:
        obs = robot.get_observation()
        cur = home_from_obs(obs)
        rest[GRIPPER] = PARK_GRIPPER_PCT          # close to the leader's rest opening (see PARK_GRIPPER_PCT)
        _log("park: from " + ", ".join(f"{j}={cur[j]:.1f}" for j in BODY) + f", gripper={cur[GRIPPER]:.0f}%")
        sweep = _Sweep(robot, link if reporting else FakeLink(), rest, False, time.monotonic, time.sleep)
        sweep.current = cur
        if reporting:
            sweep.reporter.start()
        sweep.park()
        set_body_p_gain(robot, TELEOP_P_COEFFICIENT)
        restore_fast_profile(robot)
        robot.disconnect()
        _log("park: at rest, torque released (exit 0)")
        return EXIT_OK
    except Exception:
        _log("park failed - HOLDING torque:\n" + traceback.format_exc())
        set_body_p_gain(robot, TELEOP_P_COEFFICIENT)   # hold at the teleop gain, not the stiff park gain
        restore_fast_profile(robot)
        try:
            robot.bus.disconnect(disable_torque=False)
        except Exception:
            pass
        _log(f"NEEDS_ATTENTION: support the arm and park it by hand (exit {EXIT_NEEDS_ATTENTION})")
        return EXIT_NEEDS_ATTENTION
    finally:
        if sweep is not None and reporting:
            sweep.reporter.close()


#: ``--hold-pose``: the sim policies (sim/record.py START) begin from this raised pose, not the folded rest;
#: the real eval must start there too or the first observation is out of distribution.  Path checked
#: 2026-09-20: jaw tip >= 46 mm above the mat along the straight joint-space move from REST_EXPECTED.
HOLD_POSE: Dict[str, float] = {"shoulder_pan": -20.0, "shoulder_lift": -20.0, "elbow_flex": 30.0,
                               "wrist_flex": 80.0, "wrist_roll": -90.0, "gripper": 35.0}


def hold_pose(port: str, robot_id: str, link: Any, pose: Optional[Mapping[str, float]] = None) -> int:
    """Leashed move to ``pose`` (default HOLD_POSE) and exit with TORQUE HELD so the next process (the
    real-robot eval) finds the arm there.  Same refusals and gain handling as :func:`park_only`; if the
    move fails the arm also holds (NEEDS_ATTENTION) - never released mid-air."""
    target = _pose6(dict(HOLD_POSE, **(dict(pose) if pose else {})))
    for j in BODY:
        if abs(target[j]) > SAFE_DEG[j]:
            _log(f"hold: {j}={target[j]:.1f} exceeds SAFE_DEG {SAFE_DEG[j]} (exit {EXIT_PLAN})")
            return EXIT_PLAN
    try:
        teleop = link.get_json("/teleop/status"); record = link.get_json("/training/record/status")
        servo = link.get_json("/servo/status")
        if teleop.get("running") or record.get("running") or servo.get("port") == port:
            _log(f"refusing to move: port {port} is in use (exit {EXIT_BUSY})")
            return EXIT_BUSY
    except Exception as exc:
        _log(f"hold: server check failed ({exc!r}); continuing without it")
    try:
        robot = connect_real(port, robot_id, max_relative_target=PARK_LEASH_DEG)
    except Exception:
        _log(f"connect failed (exit {EXIT_CONNECT}):\n" + traceback.format_exc())
        return EXIT_CONNECT
    set_body_p_gain(robot, PARK_P_COEFFICIENT)
    try:
        cur = home_from_obs(robot.get_observation())
        _log("hold: from " + ", ".join(f"{j}={cur[j]:.1f}" for j in BODY) + " to " + ", ".join(f"{j}={target[j]:.1f}" for j in BODY))
        sweep = _Sweep(robot, FakeLink(), target, False, time.monotonic, time.sleep)
        sweep.current = cur
        # the big lift out of the folded rest pose can trip the 12 deg following-error guard once (the servo
        # lags the leash under load); a second leashed pass from the part-way pose completes it
        for attempt in range(3):
            try:
                sweep.follow_bad = 0
                sweep.current = sweep._observed()
                sweep._move_leashed(target, "HOLD")
                break
            except _Abort as abort:
                if abort.code != EXIT_FOLLOW_ERR or attempt == 2:
                    raise
                _log(f"hold: {abort.reason}; retrying from the current pose ({attempt + 1}/2)")
                time.sleep(0.5)
        obs = robot.get_observation()
        err = _body_dist({j: obs[f"{j}.pos"] for j in BODY}, target)
        set_body_p_gain(robot, TELEOP_P_COEFFICIENT)
        restore_fast_profile(robot)
        robot.bus.disconnect(disable_torque=False)
        _log(f"hold: at pose (max error {err:.1f} deg), torque HELD, bus released (exit 0)")
        return EXIT_OK
    except Exception:
        _log("hold failed - HOLDING torque:\n" + traceback.format_exc())
        set_body_p_gain(robot, TELEOP_P_COEFFICIENT)
        restore_fast_profile(robot)
        try:
            robot.bus.disconnect(disable_torque=False)
        except Exception:
            pass
        _log(f"NEEDS_ATTENTION: support the arm and park it by hand (exit {EXIT_NEEDS_ATTENTION})")
        return EXIT_NEEDS_ATTENTION


def main(argv: Optional[List[str]] = None) -> int:
    """Entry point: ``--check-only`` audits the plan; otherwise preflight → connect → sweep."""
    parser = argparse.ArgumentParser(description="SO-101 follower splat-map sweep")
    parser.add_argument("--port", default=__import__("tc_config").PORT)
    parser.add_argument("--id", default="follower_arm")
    parser.add_argument("--server", default="http://127.0.0.1:8000")
    parser.add_argument("--token", default="")
    parser.add_argument("--mode", default="map", choices=["map", "calib", "gs", "sysid"])
    parser.add_argument("--log", default="", help="sysid: CSV of every 25 Hz command/observation pair")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--hold-pose", action="store_true",
                        help="no sweep: leashed move to HOLD_POSE (the sim policies' start pose) and exit with torque HELD")
    parser.add_argument("--park", action="store_true",
                        help="no sweep: bring the arm from wherever it is to the rest pose via TRANSIT using the "
                             "run() park path (torque is HELD if parking fails), then release torque")
    args = parser.parse_args(argv)

    if args.check_only:
        home = dict(REST_EXPECTED, gripper=10.0)
        segments = _plan_for_mode(args.mode, home)
        problems = check_plan(segments, home)
        if problems:
            for p in problems:
                print(p, flush=True)
            print(f"plan rejected: {len(problems)} violation(s)", flush=True)
            return EXIT_PLAN
        print(f"plan ok: {len(segments)} segments", flush=True)
        return EXIT_OK

    link = ServerLink(args.server, args.token)
    if args.park:
        return park_only(args.port, args.id, link)
    if args.hold_pose:
        return hold_pose(args.port, args.id, link)
    reason = _preflight(link, args.port)
    if reason is not None:
        _log(f"refusing to start: {reason} (exit {EXIT_BUSY})")
        return EXIT_BUSY

    if args.dry_run:
        robot: Any = FakeRobot()
        _log("dry run: FakeRobot, no serial")
    else:
        try:
            robot = connect_real(args.port, args.id)
        except Exception:
            _log(f"connect failed (exit {EXIT_CONNECT}):\n" + traceback.format_exc())
            return EXIT_CONNECT
        _log(f"connected to {args.port} ({args.id})")

    try:
        home = home_from_obs(robot.get_observation())
    except Exception:
        _log(f"observation failed (exit {EXIT_CONNECT}):\n" + traceback.format_exc())
        robot.disconnect()
        return EXIT_CONNECT
    _log("home pose: " + ", ".join(f"{j}={home[j]:.1f}" for j in BODY + [GRIPPER]))

    problems = check_rest_pose(home)
    if problems:
        for p in problems:
            _log(f"rest pose: {p}")
        _log(f"arm is not in the rest pose; releasing torque where it is (exit {EXIT_REST_POSE})")
        robot.disconnect()
        return EXIT_REST_POSE

    segments = _plan_for_mode(args.mode, home)
    problems = check_plan(segments, home)
    if problems:
        for p in problems:
            _log(f"plan: {p}")
        _log(f"plan rejected before any motion (exit {EXIT_PLAN})")
        robot.disconnect()
        return EXIT_PLAN

    if args.mode == "sysid" and not args.dry_run:
        restore_fast_profile(robot)     # measure the arm the datasets see (Goal_Velocity 0, Acceleration 254, P=16)
        _log("sysid: recording servo profile in effect for the ramps")
    return run(robot, link, home, args.dry_run, segments=segments,
               log_path=(args.log or None), extend_dwells=(args.mode != "sysid"))


def _plan_for_mode(mode: str, home: Mapping[str, float]) -> List[Segment]:
    """Plan builder for a CLI ``--mode``."""
    if mode == "calib":
        return plan_calib(home)
    if mode == "gs":
        return plan_gs(home)
    if mode == "sysid":
        return plan_sysid(home)
    return plan(home)


if __name__ == "__main__":
    sys.exit(main())
