# P11 cycle runner: the code behind the Nov 2 100-cycle run

This is the scripted pick + put-back runner that produced the results in `../data/` (runs `c1102_205018` and `c1102_215723`, Nov 2 2026, 20:50-22:07). It is the code as it stood at the end of that run. It was rebuilt from timestamped backups and then scrubbed of machine-specific settings, which moved into `config.json`.

## Files
| File | Role |
|---|---|
| `nudge_web.py` (+ `nudge_web.html`) | the controller: owns the serial bus, runs path steps, safety freeze (load, temperature, tracking error, STOP file), serves `/state`, `/frame`, `/cmd` on 127.0.0.1:8765 |
| `nudge.py` | control loop (encoder servo, chained moves, feed-forward from `ctl_tune.json`, temperature filter) used by `nudge_web.py` |
| `jog.py` | raw Feetech bus access (scservo_sdk) without a LeRobot connect |
| `run_path.py` | sends one path file to the controller and waits for its result |
| `cycle.py` | back-to-back pick + put-back cycles with dock-camera self-labels, wiggle test, rests, scoreboard rows |
| `programs.py` | program registry: `programs/P11/` holds the frozen P11 paths; `run` replays a program through `cycle.py` |
| `paths/prog_P11_*.json` | the P11 pick, pick retry and put-back paths (arm-frame mm, step types `to` / `via` / `twist_lock` / `roll` ...) |
| `real_io.py`, `where.py`, `plan.py`, `swap_routine.py`, `tool_session.py` | kinematics (face pose, IK), tick/degree conversion |
| `splat/fk.py`, `splat/sweep.py`, `grip_cap.py` | SO-101 forward kinematics, sweep constants and the gripper torque cap (vendored here) |
| `camcheck.py`, `capture.py`, `capture_cv.py`, `capguard.py` | camera discovery, recording and frame sanity checks |
| `tc_config.py`, `config.example.json` | machine-specific settings |

## Run
Needs Python 3.10, numpy, opencv-python, Pillow, `feetech-servo-sdk` (scservo_sdk), and on Windows `pygrabber` for camera names. A LeRobot follower calibration for your arm is also required (`calibration` in the config).

1. `cp config.example.json config.json` and set `port`, `capture_dir` and `calibration`.
2. `mkdir logs refs`
3. Start the controller in its own terminal (no motion on start): `python -u nudge_web.py`
4. With the tool docked and the arm at its ready pose, run the program:
   `python programs.py run P11 100 --wiggle --fresh-refs --profile fast --rest-above 51`
   This calls `cycle.py prog_P11_pick prog_P11_unpick 100 ...`. The flags are as recorded in the session notes; the exact command line was not logged.
5. Stop at any time: create the file `STOP` in this folder (the controller freezes and holds) or Ctrl-C the runner.

`TC_FAKE_BUS=1` runs everything against a simulated bus (no serial port) for dry runs.

**Safety:** this moves a real arm at speed. The paths are taught for one dock pose on one bench; on another setup they must be re-taught or regenerated before any real run. Keep a hand near the power switch for the first cycles.

## Differences from what ran
- Run `c1102_205018` used temperature filter v1 in `nudge.py`: a reading counted once two reads ~0.6 s apart agreed. Its one abort came from a misread that passed that filter (elbow 56 C, then 150 C; real ~47 C). Filter v2 was added at 21:56-21:58 and is the one committed here: a jump of more than 5 C must repeat for 3 reads, and anything over 90 C is ignored. `c1102_215723` ran with v2. The v1 lines are:
  ```python
  prev = getattr(self, "_traw", raw); self._traw = raw
  conf = np.minimum(raw, prev); self.temps = [int(x) for x in conf]; self.temps_raw = [int(x) for x in raw]
  if (raw - conf).max() >= 8: ev("temp_spike", raw=self.temps_raw, confirmed=self.temps)
  ```
  `nudge_web.py` reporting `temps` / `temps_raw` in `/state` came with the same change.
- The camera-frame sanity check in `cycle.py` (`camcheck.frame_problem`, `camera_invalid` aborts) was added after an earlier short run recorded a virtual-camera placeholder, before the 100-cycle run started. Its timing comes from the session notes, not from a file timestamp.
- Removed for publication: local paths, the COM port, user names in comments, and two site-specific `capture.py` commands (an offload to another PC and a short promo clip). None of them affect motion.
- Not included: the `qtraj_vendor` trajectory player, which `nudge_web.py` imports only for hand-handoff (`contact`) steps and expressive moves (P11 uses neither), and `robust_teleop.py` (the LeRobot bus retry patch, used only by `splat/sweep.py`'s `connect_real`, which this runner does not call).
- Later changes not in this version: episode logging for training data, a moving-dock frame (`dockframe.py`, `find_dock.py`) and a critic round of fixes. They will be published with the moving-dock results.
