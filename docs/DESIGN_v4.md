# SO-101 tool changer: design v4 (2026-10-04, agent toolchanger-3)

v4 = v3.1 (DESIGN.md) plus the changes a physics sim of the real arm showed were needed. Nothing is printed, ordered
or run on the arm. The viewable summary is TOOLCHANGER_v4.html.

## Sources
- Error budget: `analysis/error_budget.py` → `error_budget.json`. Linearised at the dock seat on the real SO-101 MJCF,
  using the servo model fitted to real teleop logs (`sim/physics_v2`).
- Sim: `sim/geom.py` builds the collision geometry from the CAD constants. `sim/tcsim.py` runs the arm, the servos,
  the drop-off + pick-up cycle and the failure classes. `sim/mc.py` runs the Monte Carlo over the error budget.
- Results: `sim/out/mc_*.jsonl`, `sweep_*.jsonl`, `summary.json`, `swap_v4.mp4`.
- Swap routine: `swap/swap_routine.py` (state machine + calibration) and `swap/sim_io.py` (the MuJoCo adapter; the
  routine was tested through it).
- CAD: `cad/build_changer_v4.py` → `cad/v4/` (imports v3.1 and overrides only what changed). Checks:
  `cad/v4_checks.py` → `v4/checks_v4.json`.

## 1. Error budget (what the arm really does)
Measured inputs, from real logs and benches (not guesses):

| Input | Value | Source |
|---|---|---|
| STS3215 at lerobot P = 16 | spring of ~22 N.m/rad | PROPERTIES row 30 |
| Direction-dependent friction half-band | pan 0.5, lift 1.6, elbow 1.4, wrist_flex 0.1, roll 0.3 deg | FINDINGS: teleop hysteresis |
| Repeatability | +-0.17 deg | robonine |
| Encoder | 0.088 deg | |

At the coupling face (dock frame: x along the tool axis, z up):

| Quantity | x | y | z |
|---|---|---|---|
| Plain lerobot playback of a taught pose, 1 sigma (mm) | 1.8 | 1.3 | 4.1 |
| Plain playback, worst case (mm) | 3.6 | 2.1 | 10.1 |
| Sag change, tool vs no tool (mm) | 1.4 | 0 | 4.8 |
| Encoder-servoed (outer integral loop), 3 sigma (mm) | 1.2 | 2.2 | 2.8 |
| Required capture, 3 sigma incl. dock moved 0.5 mm + print tolerance: plain (mm) | 7.1 | 4.1 | 17.2 |
| Required capture, same basis: servoed (mm) | 2.0 | 2.7 | 3.2 |
| Arm stiffness at the face (N/mm) | 1.34 | 0.40 | 0.26 |

- The sag change also tilts the tool axis by 1.6 deg.
- The arm is very soft at the face, which makes it a built-in RCC. A floating dock is not needed and would only add a
  second soft axis under the twist reaction.

Conclusion: plain playback cannot hit a ±3-4 mm funnel, and v3.1's "teach the pose, play it back" assumed it could. The
fix is software (encoder servo), not bigger funnels.

## 2. What the sim found in v3.1, and the v4 fix for each
Each row was found in the sim, fixed, and re-checked in the sim; there is also a CAD check where one applies.

### Control and software (S)

| # | v3.1 problem (sim evidence) | v4 fix |
|---|---|---|
| S1 | Plain playback leaves the master 3-10 mm off. Pins stop axially short and jam on the lip | Encoder-servoed standoff: outer integral loop on Present_Position |
| S2 | Joints lag their command by their friction band whenever they move. A perfectly servoed standoff still drifts ~4.5 mm low by contact | Integral stays on during contact, CLAMPED to 1.3 x the measured band (+0.3, floor 0.6 deg). It cancels the lag but adds at most ~2 N, so the chamfers still steer. Needs a per-robot friction calibration (`calibrate_friction`, ~25 s, no contact) |
| S3 | Lock torque in the sim: 1.5 N.m nominal, 2.3 worst corner (ball-on-PLA friction 0.3, not the 0.15 assumed). The v3.1 cap was 55 % = 1.6 N.m, and v3.1 runs stalled on the ramp | Roll Torque_Limit 85 % (2.5 N.m) for the 1.5 s twist, +4 deg overtravel to the stop, relax. PTFE dry lube on the ramps, V and ball track |
| S4 | The arm droops ~2 deg nose-down when the tool's weight transfers at lift-off. The neck then jammed across the 12 mm wall (shoulder at stall) | Back off to the centre of the axial gap, then a slow 3 mm "take the weight" lift with the full integral, then lift |
| S5 | Stored press-down deflection springs back at unlock and drags the tool | Release the press-down before unlocking |

### Dock (D)

| # | v3.1 problem (sim evidence) | v4 fix |
|---|---|---|
| D1 | Parked tool tips 3.7 deg nose-down: CoM 38 mm past the wall | Tail-rest rib under the tool body, 0.3 mm below level (r2) |
| D2 | Pitch jam across the 12 mm wall on lift-off, and the plate hanging on the flat wall top on drop-off | 8 mm slot land (6 in r0/r1) (sides and floor, nothing behind it under the neck or body). Plate gap 1 → 2 mm. 5 mm 45-deg bevel on the wall's top edge. Slot clearance 0.25 → 0.4 per side |
| D3 | The departing master drags the light parked tool along the slot | (r3) one N52 6x3 magnet pair: dock magnet 1 mm deep in the floor land, tool magnet flush in the neck underside, opposite poles (~3-4 N) |

### Coupling (C)

| # | v3.1 problem (sim evidence) | v4 fix |
|---|---|---|
| C1 | The ramps start at the bore radius and the V dowels at r 14.5, only 0.3-0.5 mm outside the spigot. Any offset made the spigot rub them and drag the tool out on retraction | Relief ring: ramps / crest / stop wall start at r 15.6, dowels at r 15.8 |
| C2 | A 0.3 mm bore clearance plus the arm's 0.5-1 mm retract lag dragged the tool | Bore R 14.3 → 14.8. Windows R 23.7 → 24.5 so offset pins still pass (cavity stays 24.2; pin tip + offset = 23.8) |
| C3 | A rotor parked >~6 deg off blocks the blade. The insertion stops short and the pins jam | (r2) Blade search at the insertion pose, toward the rotor's open stop (260 deg sweep) until the blade stalls; the still-pressing arm finishes the insertion when it drops in. The stall angle becomes the new blade zero. (r1's spring-loaded rotor was impossible, see 2c) |

### Lock check (L)

| # | v3.1 problem | v4 fix |
|---|---|---|
| L1 | The "seat wiggle" threshold of 0.5 deg would reject good locks: the tool turns +-1.3 deg in the 0.4 mm slot | 4 deg wiggle at 15 % torque, pass under 2.2 deg. A tool on a flank or crest moves the full 4 deg |

## 2b. Critic round 1 (sonnet, 2026-10-04): BLOCKING -> fixed
| Finding | Fix |
|---|---|
| B1 spring rotor not buildable (Ø8 shaft hits the rotor after 0.4 mm) | pinion_v4: shaft top at z -12, Ø16 collar = spring seat, 8 mm Oldham tongue (z -12..-4): 3.0 / 5.3 / 7.5 mm engagement at 0 / 2.25 / 4.5 mm travel, 0 mm3 overlap (v4_checks). (superseded in r2: no rotor spring) |
| B2 routine dropped v3.1 limits | gripper Torque_Limit 0.25 N.m set before every swap; roll 25 % free / 55 % insert / 85 % twist; temperature stop > 55 C; every swap wrapped in Swapper.run -> holds torque + NEEDS_ATTENTION on any exception; drop() and pick() check the tool's weight at pose F (Present_Load with vs without tool, baselines refreshed every swap) |
| S3 magnets were mirrored (sim ran with them inert) | CAD + sim: washer pocket on the underside (-x face; +x is up at the lock roll) at tool z -31 (inside the -21..-37 body), magnets at dock x 21.4 / 26.6 under it. PRINT PAUSE at the magnet pocket top layer |
| S4 claims ahead of files | the page is generated from the result files only; the first v4 run (magnets inert) is reported as such |
| S5 lock torque margin 2-8 % | plunger preload 14.8 -> ~10 N (r2: guide stays 6 mm, installed 8.05 mm, spring FREE LENGTH 10 mm instead of 11; pick by measured k); lowers lock torque AND ball-track friction; hold torque still > 2x the 0.25 N.m gripper cap |
| S6 sim vs CAD differences | sim cavity 25.0 -> 24.2 (= CAD), tool body 16 mm / CoM -38 (= CAD, reach_torque), blade servo K 22 (was 5), roll 10 %/25 % during the first descent |
| N ring sliver 0.6-0.8 mm at the bore | V trough now starts at the relief ring (r 15.3), not at the bore |
| N 1.1 mm wall dowel groove <-> plunger pocket | plungers 60/180/300 -> 75/195/315 |
| S9 slot-land crushing at 2.5 N.m | coupon C0 test: 500 x 2.5 N.m twists on the land; fallback = 8 mm land or a steel strip insert |
| S7/S8 narrow budget, Present_Load is duty-based, single-turn blade servo | reported as limits; weight thresholds are relative (with vs without tool, same pose); mount the blade so its parked angle is ~250 deg of the servo's 0-360 so the 190-deg search never wraps |

## 2c. Critic round 2 (sonnet): BLOCKING -> fixed (r2)
| Finding | Fix |
|---|---|
| B1/B2 rotor spring impossible beside the Oldham tongue; 4.5 mm travel < 4.6 needed | **Spring removed.** v3.1 rotor/pinion unchanged. A mis-parked rotor stops the insertion ~4 mm short with the arm still pressing (axial stiffness ~1.3 N/mm + clamped integral = the spring). Blade search runs AT THE INSERTION POSE toward the rotor's open stop (260 deg sweep, slot repeats every 180 deg, rotor range ~144): sim with 10x jaw drag, rotor parked 30/60/90/120 deg: insertion stops at -3.95 mm, search stalls at -234 deg, master goes in to -0.35, lock OK. With normal drag the 45-deg slot lead-in cams the rotor into line by itself. |
| B3 washer pocket in the open jaw channel (removed 11.6 of 98 mm3) | M8 steel washer (16 x 1.6) flush in the NECK underside (-x face, tool z -13): pocket removes 331 of 334 mm3 (v4_checks) |
| B4 magnet pockets overlap | magnets in the 8 mm FLOOR LAND at dock (4.5, +-3.6): 1.05 mm wall between, both inside the washer, open pockets, pressed + CA flush (no print pause, no push-out hole) |
| tail rest + floor hyperstatic at 0.0 | tail rest 0.3 mm below level; seated pitch +1 deg now 0 mm3 |
| weight check could pass without a tool (no baseline / mixed approach directions) | weigh at F always arriving from 10 mm above; drop refreshes load_no_tool, pick refreshes load_tool |
| gripper cap only set once | set at the start of every pick / drop |
| blade servo window | mount the blade so the entry angle sits >= 270 deg on the servo's 0-360 scale (260-deg sweep never crosses 0); confirm on coupon C2 |
| guide L/D 1.0 | guide back to 6 mm; ~10 N preload from a 10 mm free-length spring |
| friction calibration under-read high-friction joints 4x (found while testing r2) | +-8 deg triangle, lag on the middle half of each ramp: lift/elbow now within ~10 % (slightly high = safe) |

## 2d. Critic round 3 (sonnet): no blocking issues; should-fix applied (r3)
| Finding | Fix |
|---|---|
| r2 Monte Carlo: 491/500 = 98.2 %; failures 3 "hook" (spigot ~3 mm low lands beside a window: dock moved up 0.7-1.4 mm + standoff servo not converged), 5 "mis-seated" (lat 0.86-1.15 mm vs the 0.8 mm criterion, pin ~1 deg up a V flank), 1 drop | r3: taller skirt (5.5 mm, mouth r 40, capture +-4 -> +-6; 0.7 mm from the wall even with the plate on it), settle-until-converged at the standoff (sim AND routine), routine retries search -0.8 / +0.8 mm vertically |
| F1 magnet pocket 0.43 mm rear wall | x 4.0: 0.92 mm front and back |
| washer hole under half of each magnet | one magnet pair (neck <-> land), dock magnet 1 mm deep (gap sets the pull) |
| F4 retry +-1.5 mm > pin/cavity margin | +-0.8 mm |
| F6 first pick / drop skipped the weight check | pick takes the no-tool baseline at F first; drop weighs the tool first if needed |
| F7 stale docs / claims | lock torque margin: quasi-static probe 1.20 / 1.56 N.m (nominal / worst corner); in the r2 Monte Carlo the twist peak is p50 1.53, p99 1.90, max 2.02 N.m vs the 2.50 N.m cap = 19 % margin at the worst run (the dynamic peak includes the overtravel push onto the stop) |
| F5 pitch claim | on the land, +1 deg touches the tail rib (2.1 mm3): the rib catches at ~0.8 deg, as intended |

## 2e. Final results (r3 code, sim/out)

### Monte Carlo
"Strict" = seated within 0.8 mm. "Held" = picked and held within 1.5 mm / 2 deg tilt / 3 deg roll; the extra held runs
have a pin ~1 deg up a V flank.

| Design | Strict | Held | Failure modes (count) |
|---|---|---|---|
| v4 | 492/500 = 98.4 % | 498/500 = 99.6 % | 2 hook (dock high + stiction), 6 mis-seated |
| v4 without lube | 196/200 = 98.0 % | 198/200 | |
| v3.1 as designed | 131/500 = 26.2 % | 131/500 = 26.2 % | 239 roll stall / not locked, 75 drop hang-ups, 55 pins jammed at the lip |

### Roll torque in the v4 twist
| p50 | p99 | Max | Cap |
|---|---|---|---|
| 1.55 N.m | 2.23 N.m | 2.35 N.m | 2.50 N.m (85 %) |

The dynamic peak includes the overtravel push onto the stop block. If the coupon shows margin trouble, drop the
overtravel from 4 deg to 2 deg.

### Capture sweep
How far the dock may move after teaching, one axis at a time. Bands are "held" unless marked strict.

| Axis | v4 | v3.1 |
|---|---|---|
| Along the axis (x) | ±4 mm, strict | ±1 mm |
| Sideways (y) | ±4 mm | -2 to +3 mm |
| Up/down (z) | -4 to +2 mm | -4 to +2 mm |
| Yaw | ±3 deg | none |
| Parked rotor | any angle (-10 to 140 deg) | 10-20 deg only |

- The weak axis is the dock moving UP 3 mm or more: the spigot hooks beside a window.
- Mitigations: dock screwed down, re-teach after any bump, and the routine's ±0.8 mm vertical retries (untested past
  0.8 mm).

### Routine Monte Carlo
`sim/out/routine_v4.jsonl` runs the real `swap_routine.py` through `sim_io.py`.

## 3. Kept from v3.1 (traceability)
- Passive tools.
- Master carries servo ID 6 on the roll axis.
- 45-deg twist bayonet.
- Steel-dowel V's and Maxwell coupling.
- Ball plungers: 14.8 N each, k 5.
- Floating rotor + Oldham.
- Horizontal docks: a vertical approach is impossible.
- Lock at roll +2.79 deg.
- Separate skirt.
- 90-deg countersunk M3x10.
- Raw-tick per-tool gripper calibration.
- Camera tool not for v1.
- Teach docks by hand; never move the real arm outside sweep.py's torque-hold path.
- All v3.1 critic fixes (R1-R3) stand: the ring was regenerated with the same features plus the relief ring.

## 4. Changed on purpose
- Bore 14.3 → 14.8 and windows 23.7 → 24.5. Was R2 #9: the rotor float numbers are unaffected.
- Slot clearance 0.25 → 0.4. Rotation slop 0.8 → 1.3 deg. The twist torque is still taken by the slot lands.
- Plate gap 1 → 2. The v3.1 tilt limit came from the plate touching the wall; v4 uses the tail rest instead.
- Roll torque 55 % → 85 % during the twist only. Thermal: 1.5 s per swap.
- Wiggle threshold 0.5 → 2.2 deg.

## 5. Open (honest)
- **The sim is a model.**
  - The 22 N.m/rad + Coulomb servo model was fitted on real teleop logs, but no SO-101 whole-arm repeatability has been
    measured.
  - The contact geometry is convex hulls of the CAD (1-3 deg bins), not the STL.
  - Friction values are handbook numbers ±40 %.
  - Lube durability is unknown. A 200-run no-lube Monte Carlo shows the margin without it.
- **Blade search:** relies on the jaw (rotor) open stop being a solid stop and on the gripper servo's single-turn range
  (mount rule in 2c). Test on coupon C2.
- **Gripper body:** still concept-level.
- **Measure before printing:** the horn height and pattern, and any rear servo boss (R2 #13, open since v3).
- **Not simulated:** wear, dust, temperature, and the arm torsion of the printed links. The SO-101 link flex is
  inside the measured hysteresis but was not modelled separately.

## 6. Sensing section: dock-side alignment hall array (proposal, 2026-10-29, pm-eng; design note only, no CAD edited)
Goal: a continuous alignment signal (coupler x/y/z offset in the last ~24 mm of approach) for a learned docking policy, with no wires or
electronics on the arm. Full numbers and the buy list: (internal note, not included).
- **Current magnets (as built in `cad/build_changer_v4.py`):** one N52 6x3 pair per dock: tool magnet flush in the neck underside, dock
  magnet 1 mm deep in the 8 mm floor land at `MAG_XY = (4.0, 0)`, opposite poles. **There is no steel washer** (R3 removed it; the B3/B4
  rows in section 3 are history). The master (arm side) carries no magnet.
- **Magnet choice: add one N52 6x3 magnet to the master face (option b), do not use the existing pair as the target (option a).** The pair
  is fixed in the dock frame, so a dock-side sensor cannot see the master approach a parked tool (the pick); it would see only the tool
  magnet moving in during a drop. A master magnet is seen in both. The master magnet must sit at least 25 mm (prefer 40 mm) from the pair's
  axis in the seated pose so the pair's field, and its reseat uncertainty of about 0.4 mm, stay small at the sensors: modelled worst-case bias
  0.76 mm at 25 mm and 0.06 mm at 40 mm (z = 24 mm); a ring centred on the pair gives 0.5 to 4.9 mm and is rejected.
- **Bracket:** a small bracket off the dock post or base (the 8 mm floor land is too narrow for the sensors) holding four MLX90393 boards on
  a square of half-width 10 mm (15 mm acceptable) around the master magnet's approach axis, sensor plane about level with the land.
  Pockets with strain-relief slots for 100 mm STEMMA QT cables, and a mount for the QT Py RP2040. Board outlines are not yet measured.
  The bracket must clear the swept tool and master volume (`interference_check.py`) and stay out of the slot.
- **Master change:** one 6x3 pocket in the master coupler face, axis along the insertion direction, pressed + CA glued like the tool magnet,
  clear of the steel balls and springs, at least 25 mm from the pair axis in the seated pose. Polarity chosen on the bench.
- **I2C wiring:** QT Py RP2040 STEMMA QT port -> sensor 1 -> sensor 2 -> sensor 3 -> sensor 4 (daisy chain, 4 x 100 mm cables). The Adafruit
  MLX90393 board has two I2C address-select pins giving four addresses (base 0x18 on the current board revision; expect 0x18-0x1B, confirm with
  an I2C scan), so all four sit on one bus. QT Py to PC over USB serial; serial stays dock-side, nothing crosses the arm.
- **Not covered here:** steel parts in the master (balls, dowels, springs) will bend the field near the seated pose; the learned policy or a
  per-dock calibration table absorbs that. Bench test needed (magnet on a ruler at known offsets) before the bracket is final.

### 6b. Variant (2026-10-29, owner: "can we use the magnets already in the design"): one arm-side sensor, no new magnets
- **One Adafruit MLX90393 on the master** (arm side) reads the EXISTING magnets: on a pick, the tool-neck N52 plus the dock-floor N52 (a stacked pair);
  on a drop, the dock-floor N52 alone, with the carried tool's own neck magnet as a rigid constant baseline that is subtracted. Nothing is added to
  the tools, the docks or the master (no extra master magnet, no dock bracket). One sensor serves every tool, not one per gripper.
- **Position:** sensor on the master at 10-12 mm lateral from the magnet axis (peak 6-10 mT, under the 50 mT range; 8 mm gives 30 mT, below 10 mm is
  unsafe), and at least 25 mm from the carried tool's neck magnet so a 0.4 mm tool reseat does not bias the baseline.
- **Wiring:** 4-wire STEMMA QT from the sensor along the existing wrist-cam cable, one 400 mm cable (Adafruit's longest) to a QT Py RP2040 on the
  forearm near the elbow; the QT Py's USB-C cable follows the wrist-cam USB cable to the PC. Single device, so no address change needed.
- **Modelled error (1 sigma, s = 12 mm, last 24 mm):** pick about 0.1-0.6 mm; drop about 0.1-1.4 mm (worse at long range). The 4-sensor dock ring in
  section 6 is more accurate at range and kept as the fallback. Model and numbers: (internal note, not included).
- **Sensor mount on the master (design intent, not yet in CAD):** a small printed PLA clip or plate on the master housing (not on the coupling
  ring), fixing the MLX90393 board with its own mounting holes (size not yet measured) or a printed pocket, using owned M3 button screws and heat-set
  inserts. Placement rules: 10-12 mm lateral from the magnet axis at the seated pose, at least 25 mm from the carried neck magnet, as far as practical
  from the steel balls, dowels and springs, and flat to the approach axis so the sensor axes line up with the dock frame. The master CAD owner picks the
  spot; a bench test (magnet on a ruler) comes before it is final. The 400 mm cable follows the wrist-cam cable with strain relief at both ends.
- **USB-C cable:** the QT Py needs one data cable to the PC; owner to confirm one is in hand (not bought).
