# Trial data: 100-cycle run, 2026-11-02

Raw logs from the scripted 100-cycle pick + put-back run (program P11) on one SO-101. One dock at one known pose, tool GT1 on the v4 coupling, scripted controller (no learned policy). Every row is kept, including aborts. License: CC BY 4.0.

## Files
- `n100_scoreboard_2026-11-02.csv`: one row per dock operation (a pick or a put-back), 206 rows. Covers the two main runs (`c1102_205018`, `c1102_215723`) and the three short runs started just before them (`c1102_203210`, `c1102_203605`, `c1102_204628`).
- `n100_cycles_P11_2026-11-02.jsonl`: one line per cycle with step timings, 102 lines (100 main-run cycles + 2 from `c1102_204628`).

## Summary (main runs only)
| Run | Picks OK | Put-backs OK | Aborts |
|---|---|---|---|
| c1102_205018 | 87 | 86 | 1 put-back: `temp_spike` |
| c1102_215723 (resumed) | 13 | 13 | 0 |
| Total | 100/100 | 99/100 | 1 |

All 100 picks locked first try and passed the wiggle test. The one abort was a false servo temperature reading: the elbow read 56 C, then 150 C, while its real temperature was about 47 C. The watchdog stopped the arm, the temperature filter was fixed, and the run was resumed. It was not a docking failure. The median cycle was 29.6 s (range 26.4-34.4 s).

Short runs before the main run (not counted above): `c1102_203210` stopped by the operator before recording started (`user_stop`, after a good lock); `c1102_203605` aborted because the dock camera was a virtual-camera placeholder (`dockcheck_stale`, pick itself good); `c1102_204628` did 2 picks and 1 put-back, then aborted on a single false temperature reading (`temp_spike`, real 43 C).

## Scoreboard columns
| Column | Meaning |
|---|---|
| ts | local time of the row (EST) |
| run_id | one invocation of the cycle runner, `c<MMDD>_<HHMMSS>` |
| kind | streak / n100 / dockpos / policy_data / policy_eval |
| controller | scripted / teleop / policy |
| tool | tool on the dock (GT1_v4 = gripper tool, v4 coupling) |
| action | pick (take the tool from the dock) or putback (return it) |
| attempt_no | try number within this operation (1 = first try) |
| in_a_row | consecutive successes so far in this run |
| result | success / fail / abort |
| wiggle | after a pick: pass = the tool stayed locked under a small wrist wiggle; na = not tested (put-backs) |
| failure_mode | reason code for a fail or abort (blank on success) |
| dock_pos_id | dock position (main = the single known pose) |
| dock_offset_mm | dock offset from the main pose (blank = 0) |
| servo_temp_max_C | hottest servo reading during the operation |
| bus_V_min | lowest supply voltage seen |
| clip_dockcam, clip_hero | video file names for the operation, if recorded (videos not included) |
| cycle_log_ref | the controller log for the operation |
| notes | free text from the run |
| light_id ... infer_ms | reserved for later moving-dock and learned-policy runs; blank here |

## Cycle log fields (jsonl)
`t` start time; `pick_s`, `putback_s`, `cycle_s` durations in s; `pick_tries`; `stall` = roll servo position where the twist-lock stalled (a locked twist stalls near 2544-2548 ticks); `lock_ok`, `picked`, `put_back` booleans; `temp` hottest servo C; `vmin` supply V; `peak_elbow` peak elbow load (% of servo max); `twist` = [stall position, roll load %] per twist; `pick_steps` / `unpick_steps` = [step index, step type, seconds].
