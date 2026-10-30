# Bill of materials

**One tool changer = 20 printed parts (23 pieces, about 666 g of filament: 665 g PLA + 1.2 g TPU) plus the hardware below.** That is the arm-side master + one tool coupler + one dock (v5, with the R1 retrofit kit). Older versions, alternates, test coupons, the optional R2 head and the future tools are not counted.

How to read it: "Exact (what we used)" is the listing we actually bought (Amazon /dp link). Where we do not have the exact listing it says "none found" and only the Generic search link is given. Links go through a Logic Frame Co redirect (affiliate links may be used). Price is the pack price from our order line, or `~` if unknown. Printed-part grams are approximate (CAD volume x solid PLA density). The same data is in `BOM.csv` (with `exact_url`, `generic_url`, `search_q`).

The SO-101 arm itself is TheRobotStudio's design: https://github.com/TheRobotStudio/SO-ARM100. No bearings are used. The tool body (the part with the neck that holds the magnet and four inserts) is whatever tool you build; the example is the GT1 gripper in `stl/tool_gt1/`.

## A. One tool changer (master + 1 coupler + 1 dock)

### Bought parts (quantities for one build)

| part_id | part | qty | used for | Exact (what we used) | Generic | spec |
|---|---|---|---|---|---|---|
| tc-so101-arm | SO-101 arm (follower), TheRobotStudio design | 1 | the arm the changer mounts on | [upstream repo](https://github.com/TheRobotStudio/SO-ARM100) |  | SO-ARM100 / SO-101 repo: printed parts, STS3215 servos, controller board |
| tc-dowel-3x12 | Steel dowel pin, plain cylindrical 3 mm x 12 mm (pack of 50) | 9 | steel V-seats on the arm-side ring (6) and the pins on the tool coupler plate (3) | [Exact (what we used)](https://logicframeco.com/go/tc-dowel-3x12/github) | [Generic](https://logicframeco.com/go/tc-dowel-3x12-search/github) | 3 mm dia x 12 mm, 304 stainless, unthreaded |
| tc-magnet-n52-6x3 | N52 neodymium disc magnet 6 mm x 3 mm (pack of 50) | 2 | one in the tool neck and one in the dock floor, so a parked tool is not dragged out | [Exact (what we used)](https://logicframeco.com/go/tc-magnet-n52-6x3/github) | [Generic](https://logicframeco.com/go/tc-magnet-n52-6x3-search/github) | 6 x 3 mm, N52 |
| tc-ball-5-32 | Chrome steel ball 5/32 in (3.97 mm), G25 (pack of 100) | 3 | tips of the three spring plungers on the arm side | [Exact (what we used)](https://logicframeco.com/go/tc-ball-5-32/github) | [Generic](https://logicframeco.com/go/tc-ball-5-32-search/github) | 5/32 in, G25 chrome steel, non-magnetic |
| tc-spring-5x0.7x10 | Compression spring 5 mm OD x 0.7 mm wire x 10 mm free (pack of 10) | 3 | push the three plungers (and the tool plate back) in the twist-lock | [Exact (what we used)](https://logicframeco.com/go/tc-spring-5x0.7x10/github) | [Generic](https://logicframeco.com/go/tc-spring-5x0.7x10-search/github) | 5 mm OD, 0.7 mm wire, 10 mm free, 304 stainless; about 10 N at 8.05 mm |
| tc-m3x10-csk | M3 x 10 countersunk (flat head) hex socket screw, 90 deg (pack of 50) | 8 | hold the coupling plate to the tool neck (4) and the clamp bars to the dock frame (4) | [Exact (what we used)](https://logicframeco.com/go/tc-m3x10-csk/github) | [Generic](https://logicframeco.com/go/tc-m3x10-csk-search/github) | M3 x 10, 304 stainless, countersunk |
| tc-m3-heat-insert-4.0x6.7 | M3 brass heat-set insert, 4.0 bore x 6.7 L | 15 | threaded holes in the arm-side hub, ring and cage (7), the tool neck (4) and the dock frame (4) | none found | [Generic](https://logicframeco.com/go/tc-m3-heat-insert-4.0x6.7-search/github) | M3 thread, 4.0 mm bore, 6.7 mm long |
| tc-m3-button-master | M3 button-head socket screw, 5 to 8 mm (kit) | 13 | hold the arm-side parts together | none found | [Generic](https://logicframeco.com/go/tc-m3-button-master-search/github) | M3 ISO 7380 button head: 4x M3x6 (hub to roll horn), 5x M3x8 (ring to cage, strap), 4x M3x5 (carrier to gripper horn) |
| tc-m3x40-socket | M3 x 40 socket head cap screw | 2 | hold the servo cage up into the hub (2) | none found | [Generic](https://logicframeco.com/go/tc-m3x40-socket-search/github) | M3 x 40 hex socket |
| tc-vhb-tape-disc | Double-sided tape disc D10 x 0.4 mm | 2 | stick the dock stop pads to the dock wall | none found | [Generic](https://logicframeco.com/go/tc-vhb-tape-disc-search/github) | 10 mm dia x 0.4 mm, cut from 1 in tape |
| tc-vhb-tape-strip | Double-sided tape strip 23.6 x 14.8 x 0.4 mm | 2 | stick the dock cradle columns to the dock foot | none found | [Generic](https://logicframeco.com/go/tc-vhb-tape-strip-search/github) | 23.6 x 14.8 x 0.4 mm, cut from 1 in tape |
| tc-ptfe-grease | PTFE grease, 0.75 oz tube | 1 | light coat on the locking surfaces | none found | [Generic](https://logicframeco.com/go/tc-ptfe-grease-search/github) | all-purpose PTFE grease |
| tc-ca-glue | Cyanoacrylate gel glue | 1 | glue the steel dowels and the lead-in skirt | none found | [Generic](https://logicframeco.com/go/tc-ca-glue-search/github) | clear CA gel |
| tc-pla-black | PLA filament 1.75 mm, black (2 x 1 kg) | 1 | most printed parts | [Exact (what we used)](https://logicframeco.com/go/tc-pla-black/github) | [Generic](https://logicframeco.com/go/tc-pla-black-search/github) | 1.75 mm PLA |
| tc-pla-white | PLA filament 1.75 mm, white | 1 | dock retrofit kit prints | none found | [Generic](https://logicframeco.com/go/tc-pla-white-search/github) | 1.75 mm PLA |
| tc-tpu-95a | TPU 95A filament 1.75 mm | 1 | the two small pads on the arm-side servo cage | none found | [Generic](https://logicframeco.com/go/tc-tpu-95a-search/github) | 1.75 mm TPU 95A |

### Printed parts

| part_id | part | qty | grams each | filament | used for | STL |
|---|---|---|---|---|---|---|
| **Master (arm side)** | 9 distinct, 11 pieces, ~112.1 g | | | | | |
| tc-master-master-hub-v4 | Hub | 1 | 11.6 | PLA | arm side: bolts to the wrist-roll horn | [stl/master_v4/master_hub_v4.stl](../stl/master_v4/master_hub_v4.stl) |
| tc-master-master-cage-v4 | Servo cage | 1 | 35.6 | PLA | arm side: holds the gripper servo | [stl/master_v4/master_cage_v4.stl](../stl/master_v4/master_cage_v4.stl) |
| tc-master-master-strap-v4 | Hold-down strap | 1 | 2.4 | PLA | arm side: holds the servo in the cage | [stl/master_v4/master_strap_v4.stl](../stl/master_v4/master_strap_v4.stl) |
| tc-master-master-carrier-v4 | Blade carrier | 1 | 1.8 | PLA | arm side: the blade that turns the tool rotor | [stl/master_v4/master_carrier_v4.stl](../stl/master_v4/master_carrier_v4.stl) |
| tc-master-master-ring-v4q | Coupling ring | 1 | 45.5 | PLA | arm side: the three V-seats and windows of the twist-lock | [stl/master_v4/master_ring_v4q.stl](../stl/master_v4/master_ring_v4q.stl) |
| tc-master-master-skirt-v4 | Lead-in skirt | 1 | 13.4 | PLA | arm side: guides the tool plate in | [stl/master_v4/master_skirt_v4.stl](../stl/master_v4/master_skirt_v4.stl) |
| tc-master-master-backpad-tpu-v4 | Back pad | 1 | 1.0 | TPU | arm side: stops the servo lifting | [stl/master_v4/master_backpad_TPU_v4.stl](../stl/master_v4/master_backpad_TPU_v4.stl) |
| tc-master-master-farpad-tpu-v4 | Far pad shim | 1 | 0.2 | TPU | arm side: shim under the strap | [stl/master_v4/master_farpad_TPU_v4.stl](../stl/master_v4/master_farpad_TPU_v4.stl) |
| tc-master-master-plunger0-v4b | Ball plunger (3 identical; the other two STLs are the same part at the other seats) | 3 | 0.2 | PLA | arm side: carries the steel ball and spring | [stl/master_v4/master_plunger0_v4b.stl](../stl/master_v4/master_plunger0_v4b.stl) |
| **Tool coupler** | 3 distinct, 3 pieces, ~30.6 g | | | | | |
| tc-gt1-gt-plate | Coupling plate | 1 | 25.3 | PLA | tool coupler: the plate with the three pins that locks into the ring | [stl/tool_gt1/gt_plate.stl](../stl/tool_gt1/gt_plate.stl) |
| tc-gt1-gt-rotor | Floating rotor | 1 | 3.8 | PLA | tool coupler: the arm blade drives this | [stl/tool_gt1/gt_rotor.stl](../stl/tool_gt1/gt_rotor.stl) |
| tc-gt1-gt-shaft | Drive shaft | 1 | 1.5 | PLA | tool coupler: carries the rotor turn into the tool | [stl/tool_gt1/gt_shaft.stl](../stl/tool_gt1/gt_shaft.stl) |
| **Dock** | 4 distinct, 4 pieces, ~455.8 g | | | | | |
| tc-dock-dock-post-asbuilt | Dock post (the same part is also exported as dock_post_v4) | 1 | 326.3 | PLA | dock: the slotted wall that parks the tool | [stl/dock_v5/dock_post_asbuilt.stl](../stl/dock_v5/dock_post_asbuilt.stl) |
| tc-dock-db-frame | Dock table frame | 1 | 86.9 | PLA | dock: base that clamps to the table | [stl/master_v4/db_frame.stl](../stl/master_v4/db_frame.stl) |
| tc-dock-db-bar-front | Front clamp bar | 1 | 25.1 | PLA | dock: clamps the post, arm side | [stl/master_v4/db_bar_front.stl](../stl/master_v4/db_bar_front.stl) |
| tc-dock-db-bar-back | Back clamp bar | 1 | 17.5 | PLA | dock: clamps the post, back side | [stl/master_v4/db_bar_back.stl](../stl/master_v4/db_bar_back.stl) |
| **Dock R1 kit** | 4 distinct, 5 pieces, ~67.4 g | | | | | |
| tc-dock-v5-cradle-col-l | R1 cradle column, left | 1 | 23.7 | PLA | dock R1 kit: guides the tool body into the dock | [stl/dock_v5/cradle_col_L.stl](../stl/dock_v5/cradle_col_L.stl) |
| tc-dock-v5-cradle-col-r | R1 cradle column, right | 1 | 23.7 | PLA | dock R1 kit: guides the tool body into the dock | [stl/dock_v5/cradle_col_R.stl](../stl/dock_v5/cradle_col_R.stl) |
| tc-dock-v5-stop-pad | R1 stop pad, D10 round (2 needed) | 2 | 0.2 | PLA | dock R1 kit: sets the plate gap to the wall | [stl/dock_v5/stop_pad.stl](../stl/dock_v5/stop_pad.stl) |
| tc-dock-v5-t-gauge | R1 T-gauge (one-time alignment template) | 1 | 19.6 | PLA | dock R1 kit: lines the dock up with the arm | [stl/dock_v5/t_gauge.stl](../stl/dock_v5/t_gauge.stl) |
| **Total** | 20 distinct, 23 pieces, ~666 g | | | | | |

## B. Per extra tool

Each added tool needs its own coupler. Each extra tool also wants its own dock: repeat the dock rows of section A.

| part_id | part | qty | used for | Exact (what we used) | Generic | spec |
|---|---|---|---|---|---|---|
| tc-dowel-3x12 | Steel dowel pin, plain cylindrical 3 mm x 12 mm (pack of 50) | 3 | steel V-seats on the arm-side ring (6) and the pins on the tool coupler plate (3) | [Exact (what we used)](https://logicframeco.com/go/tc-dowel-3x12/github) | [Generic](https://logicframeco.com/go/tc-dowel-3x12-search/github) | 3 mm dia x 12 mm, 304 stainless, unthreaded |
| tc-magnet-n52-6x3 | N52 neodymium disc magnet 6 mm x 3 mm (pack of 50) | 1 | one in the tool neck and one in the dock floor, so a parked tool is not dragged out | [Exact (what we used)](https://logicframeco.com/go/tc-magnet-n52-6x3/github) | [Generic](https://logicframeco.com/go/tc-magnet-n52-6x3-search/github) | 6 x 3 mm, N52 |
| tc-m3x10-csk | M3 x 10 countersunk (flat head) hex socket screw, 90 deg (pack of 50) | 4 | hold the coupling plate to the tool neck (4) and the clamp bars to the dock frame (4) | [Exact (what we used)](https://logicframeco.com/go/tc-m3x10-csk/github) | [Generic](https://logicframeco.com/go/tc-m3x10-csk-search/github) | M3 x 10, 304 stainless, countersunk |
| tc-m3-heat-insert-4.0x6.7 | M3 brass heat-set insert, 4.0 bore x 6.7 L | 4 | threaded holes in the arm-side hub, ring and cage (7), the tool neck (4) and the dock frame (4) | none found | [Generic](https://logicframeco.com/go/tc-m3-heat-insert-4.0x6.7-search/github) | M3 thread, 4.0 mm bore, 6.7 mm long |
| tc-gt1-gt-plate | Coupling plate (printed, 25.3 g PLA) | 1 | tool coupler: the plate with the three pins that locks into the ring | [stl/tool_gt1/gt_plate.stl](../stl/tool_gt1/gt_plate.stl) | | |
| tc-gt1-gt-rotor | Floating rotor (printed, 3.8 g PLA) | 1 | tool coupler: the arm blade drives this | [stl/tool_gt1/gt_rotor.stl](../stl/tool_gt1/gt_rotor.stl) | | |
| tc-gt1-gt-shaft | Drive shaft (printed, 1.5 g PLA) | 1 | tool coupler: carries the rotor turn into the tool | [stl/tool_gt1/gt_shaft.stl](../stl/tool_gt1/gt_shaft.stl) | | |

## C. Optional and future

| part_id | part | qty | used for | Exact (what we used) | Generic | spec |
|---|---|---|---|---|---|---|
| tc-heat-insert-m3-5.0x4 | M3 brass heat-set insert, 5.0 OD x 4 L | 4 | attach the optional R2 dock head (4) | none found | [Generic](https://logicframeco.com/go/tc-heat-insert-m3-5.0x4-search/github) | M3 thread, 5.0 OD, 4 long |
| tc-m3x10-button | M3 x 10 button-head socket screw | 4 | attach the optional R2 dock head (4) | none found | [Generic](https://logicframeco.com/go/tc-m3x10-button-search/github) | M3 x 10 ISO 7380 |
| tc-dock-v5-head-r2 | Optional R2 dock head (needs one jigsaw cut) (printed, 79.3 g PLA) | 1 | optional dock head that replaces the old wall top | [stl/dock_v5/head_R2.stl](../stl/dock_v5/head_R2.stl) | | |

The other tool designs (JG2, HI5F, STAMP2, FR1, and the GT1 gripper jaws and cover) are concept or example tools and are not part of the changer BOM. See [docs/future_tools](future_tools/README.md) and the `stl/heads_*` and `stl/tool_*` folders.
