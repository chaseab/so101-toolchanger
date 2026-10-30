# SO-101 tool changer

Hardware: free for personal/non-commercial use (CC BY-NC-SA 4.0). Code: open source (GPL-3.0).
Commercial use of the hardware / kits: contact Logic Frame Co ([logicframeco.com](https://logicframeco.com)).

A tool changer for the SO-101 robot arm: a master on the wrist, a coupler on each tool, and docks on the bench. Parts are 3D printed with a few dowels, magnets, balls and springs. The control code drives the arm through LeRobot.

## Status
Work in progress: scripted locks achieved; no full automatic swap cycle yet. Scripted dock cycles and seat tests are logged; one lock was wiggle-verified by hand. 0 clean picks so far, and the lift-out from the dock still jams. Treat everything here as unproven design files.

## The changer
Exploded view of the master (arm side), the tool coupler and the dock, with every printed part, screw, insert, dowel, ball, spring and magnet numbered and sized. Concept render from the current CAD, not a photo.

![SO-101 tool changer, exploded](docs/diagrams/changer_exploded.png)

([SVG version](docs/diagrams/changer_exploded.svg))

## How the twist-lock works
The master on the wrist locks onto the tool coupler with three pins in three steel V-seats (a kinematic coupling). The arm approaches with the wrist rolled 45 deg, the pins pass through three windows, then a 45 deg wrist roll drives each pin up a ramp (spring plungers with steel balls push the tool plate back), over a crest and into the V. A magnet pair keeps the parked tool from being dragged out of the dock. Release is the same motion in reverse.

![Twist-lock sequence: approach, engage, twist and lock, release](docs/diagrams/twist_lock_sequence.png)

([SVG version](docs/diagrams/twist_lock_sequence.svg))

## Dock
The v5 dock retrofit (stop pads, cradle columns, T-gauge and the R2 head option), drawn with the old post ghosted so the parts behind it show.

![Dock v5 retrofit](docs/diagrams/dock_v5_retrofit.png)

([SVG version](docs/diagrams/dock_v5_retrofit.svg))

## Video
Real bench takes, 10/28, scene camera, low-res. They are attached to the [media-2026-10-30 release](https://github.com/chaseab/so101-toolchanger/releases/tag/media-2026-10-30). Failures stay in.

No clip yet shows a lock clearly; the next bench session records every attempt.

**real_take1_hand_touch_arm_lifts.mp4**: Real bench take, 10/28, scene camera. A hand touches the tool body at the dock, then the arm jerks up and away. This session was hand-flown with the leader arm (teleop), not an automatic swap. Whether this was a lock or just contact is unclear, and the yellow blow-out is the camera exposure. Failures stay in.

**real_take2_swing_away.mp4**: Real bench take, 10/28, scene camera. The arm sits at the dock, then swings away and back. Scripted run, no cut and no sound. A lock is not visible here and the swing away may be a release, a jam or a test pull. The arm has done 0 full automatic swaps so far.

**real_take3_wrist_snap.mp4**: Real bench take, 10/28, scene camera. The wrist snaps over fast and a small black piece with a cable ends up on the mat. I am not claiming whether this run was hand-flown or scripted, and it is unclear whether the piece was dropped or let go. Not a clean swap. The arm has done 0 full automatic swaps so far.

## Bill of materials
Full table: [docs/BOM.md](docs/BOM.md) (and `docs/BOM.csv`): every screw, insert, magnet, dowel, spring and printed part with size, quantity, source and what it is used for. Changer-core parts come first, future tools last. Bought parts for the changer core:

| Part | Qty (starter set) | Used for |
|---|---|---|
| [Steel dowel pin, plain cylindrical 3 mm x 12 mm (pack of 50)](https://logicframeco.com/go/tc-dowel-3x12/github) | 28 | arm-side coupler, tool-side coupler, future tools |
| [N52 neodymium disc magnet 6 mm x 3 mm (pack of 50)](https://logicframeco.com/go/tc-magnet-n52-6x3/github) | 8 | tool-side coupler, dock, future tools |
| [Chrome steel ball 5/32 in (3.97 mm), G25 (pack of 100)](https://logicframeco.com/go/tc-ball-5-32/github) | 3 | arm-side coupler |
| [Compression spring 5 mm OD x 0.7 mm wire x 10 mm free (pack of 10)](https://logicframeco.com/go/tc-spring-5x0.7x10/github) | 3 | arm-side coupler |
| [M3 x 10 countersunk (flat head) hex socket screw, 90 deg (pack of 50)](https://logicframeco.com/go/tc-m3x10-csk/github) | 20 | tool-side coupler, future tools |
| [M3 brass heat-set insert, 4.0 bore x 6.7 L](https://logicframeco.com/go/tc-m3-heat-insert-4.0x6.7/github) | 39 | arm-side coupler, tool-side coupler, future tools |
| [M3 screws for the master (button head unless noted)](https://logicframeco.com/go/tc-m3-button-master/github) | 15 | arm-side coupler |
| [M3 brass heat-set insert, 5.0 OD x 4 L (dock option R2 only)](https://logicframeco.com/go/tc-heat-insert-m3-5.0x4/github) | 4 | dock (R2 option) |
| [Double-sided / VHB tape disc D10 x 0.4 mm](https://logicframeco.com/go/tc-vhb-tape-disc/github) | 2 | dock |
| [Double-sided / VHB tape strip 23.6 x 14.8 x 0.4 mm](https://logicframeco.com/go/tc-vhb-tape-strip/github) | 2 | dock |
| [PTFE grease, 0.75 oz tube](https://logicframeco.com/go/tc-ptfe-grease/github) | 1 | arm-side coupler, tool-side coupler |

## Build
- Print the arm-side coupler, the tool-side coupler and the dock parts from `stl/` (`master_v4`, `tool_gt1`, `dock_v5`); the BOM lists material per part. STEP files are in `step/`.
- Assemble with the hardware in the BOM: dowels glued into the ring's V grooves and the tool plate, ball plungers with springs, heat-set inserts, M3 screws, and one N52 6 x 3 magnet in the tool neck and one in the dock floor.
- The dock retrofit kit (R1) prints with no cutting; the R2 head needs one jigsaw cut. See `docs/diagrams/dock_v5_retrofit.png`.
- Design notes, lock-torque numbers and test results: `docs/DESIGN_v4.md`; bench procedure: `docs/RUNBOOK.html`.

## Folder map
- `cad/` CAD-as-code (build123d) for the master, docks and gripper tool, plus `cad/heads/` and `cad/tools/` for the future tools. Interference and lock-path checks are here too.
- `stl/`, `step/` exported meshes of the current parts: `master_v4`, `dock_v5`, `tool_gt1`, plus `heads_*` and `tool_fr1` for the future tools.
- `swap/` arm control and test code (capguard, jog, real_io, dockcycle, pick2, fakebus/sim_io, tests).
- `sim/` MuJoCo contact model used by the sim twin.
- `docs/` design notes (`DESIGN_v4.md`), runbook, `BOM.md` / `BOM.csv`, `diagrams/`, and `future_tools/` (concept tools, not the focus yet).
- `LICENSES/` license texts. `NOTICE` upstream credits.

Not included: SO-ARM100 / Feetech servo meshes (get them upstream), calibration files, logs, footage, print files (gcode/3mf).
Some swap/ scripts import modules that live outside this repo (`splat`, `grip_cap`) and `swap/capture.py` (camera capture used by capguard) is not included; they will not run unmodified. Paths and serials were scrubbed; set `TC_PYTHON`, `TC_CAPTURE_OUT` as needed.

## Licenses (split)
| What | Where | License |
|---|---|---|
| Hardware, CAD source, STEP/STL, docs | `cad/`, `stl/`, `step/`, `docs/`, this README | CC BY-NC-SA 4.0 |
| Code | `swap/`, `sim/` | GPL-3.0-or-later |

Full texts are in `LICENSES/`. The hardware, CAD, STEP/STL and docs are for personal and non-commercial use only; selling kits or parts, or using them in a commercial product, needs written permission from Logic Frame Co. The code is GPL-3.0-or-later. No file in this repo is upstream code or geometry, so no file here is relabeled; upstream Apache-2.0 work is credited in `NOTICE` and is not redistributed.

Upstream Apache-2.0 work (TheRobotStudio SO-ARM100, Hugging Face LeRobot) is credited in `NOTICE`. SO-101 is a TheRobotStudio design; this project is not affiliated. Provided as-is.

(c) 2026 Chase Bonfiglio / Logic Frame Co. If you build or adapt this, please credit Chase Bonfiglio / Logic Frame Co and link back to this repo.
