# Bill of materials

The changer core is the arm-side coupler (the master), the tool-side coupler and the dock. Those parts come first. Parts for tools we have designed but are not focused on yet are in the last section, "Future tools". Quantities are totals for the starter set (1 master, 3 docks, 5 tools/heads), so some shared hardware counts include the future tools. The `used for` column says where each part goes. `docs/BOM.csv` has the same table plus the raw vendor URL; its link column is the plain /go/<part_id> link.

Source: `bought` = we bought this exact product, `generic` = any equivalent part works (our brand noted), `printed`, `upstream`. Links go to a Logic Frame Co redirect (affiliate links may be used). Price is the price of the listed pack from our own order line, or `~` if unknown. Printed-part grams are approximate (CAD volume x density).

The SO-101 arm itself is TheRobotStudio's design: https://github.com/TheRobotStudio/SO-ARM100

No bearings are used in the current designs. Fasteners for the gripper tool beyond its coupling set were not itemised in the CAD sidecars.

## Changer core: hardware and consumables

| part_id | part | qty | spec | used for | vendor | source | price (USD) |
|---|---|---|---|---|---|---|---|
| [tc-so101-arm](https://logicframeco.com/go/tc-so101-arm/github) | SO-101 arm (follower), TheRobotStudio design | 1 | SO-ARM100 / SO-101 repo: printed parts, STS3215 servos, controller board | the arm the changer mounts on | TheRobotStudio | upstream | ~ |
| [tc-dowel-3x12](https://logicframeco.com/go/tc-dowel-3x12/github) | Steel dowel pin, plain cylindrical 3 mm x 12 mm (pack of 50) | 28 | 3 mm dia x 12 mm, 304 stainless, unthreaded | arm-side coupler, tool-side coupler, future tools | Amazon (MroMax) | bought | 5.99 |
| [tc-magnet-n52-6x3](https://logicframeco.com/go/tc-magnet-n52-6x3/github) | N52 neodymium disc magnet 6 mm x 3 mm (pack of 50) | 8 | 6 x 3 mm, N52, one per tool neck + one per dock floor | tool-side coupler, dock, future tools | Amazon | bought | 14.59 |
| [tc-ball-5-32](https://logicframeco.com/go/tc-ball-5-32/github) | Chrome steel ball 5/32 in (3.97 mm), G25 (pack of 100) | 3 | 5/32 in, G25 chrome steel, non-magnetic | arm-side coupler | Amazon (PGN) | bought | 7.95 |
| [tc-spring-5x0.7x10](https://logicframeco.com/go/tc-spring-5x0.7x10/github) | Compression spring 5 mm OD x 0.7 mm wire x 10 mm free (pack of 10) | 3 | 5 mm OD, 0.7 mm wire, 10 mm free length, 304 stainless; about 10 N at 8.05 mm | arm-side coupler | Amazon (MECCANIXITY) | bought | 6.59 |
| [tc-m3x10-csk](https://logicframeco.com/go/tc-m3x10-csk/github) | M3 x 10 countersunk (flat head) hex socket screw, 90 deg (pack of 50) | 20 | M3 x 10, 304 stainless, countersunk | tool-side coupler, future tools | Amazon | bought | 5.99 |
| [tc-m3-heat-insert-4.0x6.7](https://logicframeco.com/go/tc-m3-heat-insert-4.0x6.7/github) | M3 brass heat-set insert, 4.0 bore x 6.7 L | 39 | M3 thread, 4.0 mm bore, 6.7 mm long (shallow locating counterbore in the print) | arm-side coupler, tool-side coupler, future tools | Amazon (FFVRVSS 600-pc set) | generic | ~ |
| [tc-m3x10-button](https://logicframeco.com/go/tc-m3x10-button/github) | M3 x 10 button-head socket screw | 2 | M3 x 10 ISO 7380 button head (self-taps into the carriage bracket); +4 for the dock R2 option | dock (R2 option), future tools | Amazon (Fgruh M2-M5 kit) | generic | ~ |
| [tc-m3-button-master](https://logicframeco.com/go/tc-m3-button-master/github) | M3 screws for the master (button head unless noted) | 15 | 4x M3x6 button (hub to roll horn), 2x M3x40 socket head (cage to hub), 5x M3x8 button (3 ring to cage, 2 strap), 4x M3x5 button (carrier to gripper horn) | arm-side coupler | Amazon (Fgruh M2-M5 kit) | generic | ~ |
| [tc-heat-insert-m3-5.0x4](https://logicframeco.com/go/tc-heat-insert-m3-5.0x4/github) | M3 brass heat-set insert, 5.0 OD x 4 L (dock option R2 only) | 4 | M3 thread, 5.0 mm OD, 4 mm long | dock (R2 option) | Amazon (FFVRVSS 600-pc set) | generic | ~ |
| [tc-vhb-tape-disc](https://logicframeco.com/go/tc-vhb-tape-disc/github) | Double-sided / VHB tape disc D10 x 0.4 mm | 2 | 10 mm dia x 0.4 mm | dock | any | generic | ~ |
| [tc-vhb-tape-strip](https://logicframeco.com/go/tc-vhb-tape-strip/github) | Double-sided / VHB tape strip 23.6 x 14.8 x 0.4 mm | 2 | 23.6 x 14.8 x 0.4 mm (cut from tape) | dock | any | generic | ~ |
| [tc-ptfe-grease](https://logicframeco.com/go/tc-ptfe-grease/github) | PTFE grease, 0.75 oz tube | 1 | all-purpose PTFE grease for the locking surfaces | arm-side coupler, tool-side coupler | Amazon (Magnalube-G) | generic | 11.99 |
| [tc-ca-glue](https://logicframeco.com/go/tc-ca-glue/github) | Cyanoacrylate gel glue | 1 | clear CA gel | assembly (all printed parts) | any | generic | 4.73 |
| [tc-pla-black](https://logicframeco.com/go/tc-pla-black/github) | PLA filament 1.75 mm, black (2 x 1 kg) | 1 | 1.75 mm PLA | printed parts | Amazon (ELEGOO) | bought | 23.73 |
| [tc-pla-white](https://logicframeco.com/go/tc-pla-white/github) | PLA filament 1.75 mm, white | 1 | 1.75 mm PLA | printed parts | any | generic | ~ |
| [tc-tpu-95a](https://logicframeco.com/go/tc-tpu-95a/github) | TPU 95A filament 1.75 mm | 1 | 1.75 mm TPU 95A (fin-ray fingers, master pads) | printed parts (TPU pads, fingers) | Amazon (Polymaker) | generic | ~ |

## Changer core: printed parts (grams approximate)

| part_id | part | spec | used for |
|---|---|---|---|
| tc-master-master-backpad-pla-v4 | [MASTER master backpad PLA v4 (printed)](../stl/master_v4/master_backpad_PLA_v4.stl) | ~1.0 g PLA; STL in stl/ | arm-side coupler |
| tc-master-master-backpad-tpu-v4 | [MASTER master backpad TPU v4 (printed)](../stl/master_v4/master_backpad_TPU_v4.stl) | ~1.0 g TPU; STL in stl/ | arm-side coupler |
| tc-master-master-cage-v4 | [MASTER master cage v4 (printed)](../stl/master_v4/master_cage_v4.stl) | ~35.6 g PLA; STL in stl/ | arm-side coupler |
| tc-master-master-carrier-v4 | [MASTER master carrier v4 (printed)](../stl/master_v4/master_carrier_v4.stl) | ~1.8 g PLA; STL in stl/ | arm-side coupler |
| tc-master-master-farpad-pla-v4 | [MASTER master farpad PLA v4 (printed)](../stl/master_v4/master_farpad_PLA_v4.stl) | ~0.2 g PLA; STL in stl/ | arm-side coupler |
| tc-master-master-farpad-tpu-v4 | [MASTER master farpad TPU v4 (printed)](../stl/master_v4/master_farpad_TPU_v4.stl) | ~0.2 g TPU; STL in stl/ | arm-side coupler |
| tc-master-master-hub-v4 | [MASTER master hub v4 (printed)](../stl/master_v4/master_hub_v4.stl) | ~11.6 g PLA; STL in stl/ | arm-side coupler |
| tc-master-master-plunger0-v4b | [MASTER master plunger0 v4b (printed)](../stl/master_v4/master_plunger0_v4b.stl) | ~0.2 g PLA; STL in stl/ | arm-side coupler |
| tc-master-master-plunger1-v4b | [MASTER master plunger1 v4b (printed)](../stl/master_v4/master_plunger1_v4b.stl) | ~0.2 g PLA; STL in stl/ | arm-side coupler |
| tc-master-master-plunger2-v4b | [MASTER master plunger2 v4b (printed)](../stl/master_v4/master_plunger2_v4b.stl) | ~0.2 g PLA; STL in stl/ | arm-side coupler |
| tc-master-master-ring-v4q | [MASTER master ring v4q (printed)](../stl/master_v4/master_ring_v4q.stl) | ~45.5 g PLA; STL in stl/ | arm-side coupler |
| tc-master-master-skirt-v4 | [MASTER master skirt v4 (printed)](../stl/master_v4/master_skirt_v4.stl) | ~13.4 g PLA; STL in stl/ | arm-side coupler |
| tc-master-master-strap-v4 | [MASTER master strap v4 (printed)](../stl/master_v4/master_strap_v4.stl) | ~2.4 g PLA; STL in stl/ | arm-side coupler |
| tc-gt1-gt-body | [GT1 gt body (printed)](../stl/tool_gt1/gt_body.stl) | ~50.3 g PLA; STL in stl/ | tool-side coupler |
| tc-gt1-gt-pins | [GT1 gt pins (printed)](../stl/tool_gt1/gt_pins.stl) | ~0.3 g PLA; STL in stl/ | tool-side coupler |
| tc-gt1-gt-plate | [GT1 gt plate (printed)](../stl/tool_gt1/gt_plate.stl) | ~25.3 g PLA; STL in stl/ | tool-side coupler |
| tc-gt1-gt-rotor | [GT1 gt rotor (printed)](../stl/tool_gt1/gt_rotor.stl) | ~3.8 g PLA; STL in stl/ | tool-side coupler |
| tc-dock-db-bar-back | [DOCK db bar back (printed)](../stl/master_v4/db_bar_back.stl) | ~17.5 g PLA; STL in stl/ | dock |
| tc-dock-db-bar-front | [DOCK db bar front (printed)](../stl/master_v4/db_bar_front.stl) | ~25.1 g PLA; STL in stl/ | dock |
| tc-dock-db-frame | [DOCK db frame (printed)](../stl/master_v4/db_frame.stl) | ~86.9 g PLA; STL in stl/ | dock |
| tc-dock-dock-post-v4 | [DOCK dock post v4 (printed)](../stl/master_v4/dock_post_v4.stl) | ~326.3 g PLA; STL in stl/ | dock |
| tc-dock-v5-cradle-col-l | [DOCK-V5 cradle col L (printed)](../stl/dock_v5/cradle_col_L.stl) | ~23.7 g PLA; STL in stl/ | dock |
| tc-dock-v5-cradle-col-r | [DOCK-V5 cradle col R (printed)](../stl/dock_v5/cradle_col_R.stl) | ~23.7 g PLA; STL in stl/ | dock |
| tc-dock-v5-dock-post-asbuilt | [DOCK-V5 dock post asbuilt (printed)](../stl/dock_v5/dock_post_asbuilt.stl) | ~326.3 g PLA; STL in stl/ | dock |
| tc-dock-v5-head-r2 | [DOCK-V5 head R2 (printed)](../stl/dock_v5/head_R2.stl) | ~79.3 g PLA; STL in stl/ | dock |
| tc-dock-v5-stop-pad | [DOCK-V5 stop pad (printed)](../stl/dock_v5/stop_pad.stl) | ~0.2 g PLA; STL in stl/ | dock |
| tc-dock-v5-stop-pad-l | [DOCK-V5 stop pad L (printed)](../stl/dock_v5/stop_pad_L.stl) | ~0.2 g PLA; STL in stl/ | dock |
| tc-dock-v5-stop-pad-r | [DOCK-V5 stop pad R (printed)](../stl/dock_v5/stop_pad_R.stl) | ~0.2 g PLA; STL in stl/ | dock |
| tc-dock-v5-t-gauge | [DOCK-V5 t gauge (printed)](../stl/dock_v5/t_gauge.stl) | ~19.6 g PLA; STL in stl/ | dock |

## Future tools (concept, not the focus yet): hardware

| part_id | part | qty | spec | used for | vendor | source | price (USD) |
|---|---|---|---|---|---|---|---|
| [tc-m3x6-button](https://logicframeco.com/go/tc-m3x6-button/github) | M3 x 6 button-head socket screw | 16 | M3 x 6 ISO 7380 button head | future tools | Amazon (Fgruh M2-M5 kit) | generic | ~ |
| [tc-m3x8-button](https://logicframeco.com/go/tc-m3x8-button/github) | M3 x 8 button-head socket screw | 24 | M3 x 8 ISO 7380 button head (some self-tap into printed bosses) | future tools | Amazon (Fgruh M2-M5 kit) | generic | ~ |
| [tc-m3x25-button](https://logicframeco.com/go/tc-m3x25-button/github) | M3 x 25 socket screw | 2 | M3 x 25 button/socket head | future tools | Amazon | generic | ~ |
| [tc-m3-nut](https://logicframeco.com/go/tc-m3-nut/github) | M3 hex nut (pack of 200) | 2 | M3 x 0.5 hex nut | future tools | Amazon (Yinpecly) | generic | 5.99 |
| [tc-m3-washer](https://logicframeco.com/go/tc-m3-washer/github) | M3 flat washer 3 mm ID x 9 mm OD (pack of 100) | 4 | M3, about 9 mm OD, stainless | future tools | Amazon | bought | 4.99 |

## Future tools: printed parts

| part_id | part | spec | used for |
|---|---|---|---|
| tc-jg2-plate | [JG2 plate (printed)](../stl/heads_jg2/plate.stl) | ~19.4 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-jg2-rotor | [JG2 rotor (printed)](../stl/heads_jg2/rotor.stl) | ~2.9 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-jg2-shaft | [JG2 shaft (printed)](../stl/heads_jg2/shaft.stl) | ~1.2 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-jg2-housing | [JG2 housing (printed)](../stl/heads_jg2/housing.stl) | ~25.9 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-jg2-hub | [JG2 hub (printed)](../stl/heads_jg2/hub.stl) | ~1.4 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-jg2-nose | [JG2 nose (printed)](../stl/heads_jg2/nose.stl) | ~15.6 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-jg2-bevel-a | [JG2 bevel a (printed)](../stl/heads_jg2/bevel_a.stl) | ~0.5 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-jg2-pinion-unit | [JG2 pinion unit (printed)](../stl/heads_jg2/pinion_unit.stl) | ~1.6 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-jg2-carriage | [JG2 carriage (printed)](../stl/heads_jg2/carriage.stl) | ~2.6 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-jg2-rod | [JG2 rod (printed)](../stl/heads_jg2/rod.stl) | ~2.1 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-jg2-lid | [JG2 lid (printed)](../stl/heads_jg2/lid.stl) | ~2.6 g PLA; STL in stl/ | Jenga pusher tool (future) |
| tc-hi5f-plate | [HI5F plate (printed)](../stl/heads_hi5f/plate.stl) | ~19.4 g PLA; STL in stl/ | hand tool (future) |
| tc-hi5f-rotor | [HI5F rotor (printed)](../stl/heads_hi5f/rotor.stl) | ~2.9 g PLA; STL in stl/ | hand tool (future) |
| tc-hi5f-shaft | [HI5F shaft (printed)](../stl/heads_hi5f/shaft.stl) | ~1.2 g PLA; STL in stl/ | hand tool (future) |
| tc-hi5f-housing | [HI5F housing (printed)](../stl/heads_hi5f/housing.stl) | ~25.9 g PLA; STL in stl/ | hand tool (future) |
| tc-hi5f-hub | [HI5F hub (printed)](../stl/heads_hi5f/hub.stl) | ~4.8 g PLA; STL in stl/ | hand tool (future) |
| tc-hi5f-frame | [HI5F frame (printed)](../stl/heads_hi5f/frame.stl) | ~4.6 g PLA; STL in stl/ | hand tool (future) |
| tc-hi5f-hand | [HI5F hand (printed)](../stl/heads_hi5f/hand.stl) | ~10.1 g PLA; STL in stl/ | hand tool (future) |
| tc-hi5f-carriage | [HI5F carriage (printed)](../stl/heads_hi5f/carriage.stl) | ~4.7 g PLA; STL in stl/ | hand tool (future) |
| tc-hi5f-f-index | [HI5F f index (printed)](../stl/heads_hi5f/f_index.stl) | ~3.6 g TPU; STL in stl/ | hand tool (future) |
| tc-hi5f-f-middle | [HI5F f middle (printed)](../stl/heads_hi5f/f_middle.stl) | ~3.8 g TPU; STL in stl/ | hand tool (future) |
| tc-hi5f-f-ring | [HI5F f ring (printed)](../stl/heads_hi5f/f_ring.stl) | ~3.7 g TPU; STL in stl/ | hand tool (future) |
| tc-hi5f-f-pinky | [HI5F f pinky (printed)](../stl/heads_hi5f/f_pinky.stl) | ~3.1 g TPU; STL in stl/ | hand tool (future) |
| tc-hi5f-f-thumb | [HI5F f thumb (printed)](../stl/heads_hi5f/f_thumb.stl) | ~2.5 g TPU; STL in stl/ | hand tool (future) |
| tc-stamp2-plate | [STAMP2 plate (printed)](../stl/heads_stamp2/plate.stl) | ~19.4 g PLA; STL in stl/ | stamp tool (future) |
| tc-stamp2-rotor | [STAMP2 rotor (printed)](../stl/heads_stamp2/rotor.stl) | ~2.9 g PLA; STL in stl/ | stamp tool (future) |
| tc-stamp2-shaft | [STAMP2 shaft (printed)](../stl/heads_stamp2/shaft.stl) | ~1.2 g PLA; STL in stl/ | stamp tool (future) |
| tc-stamp2-housing | [STAMP2 housing (printed)](../stl/heads_stamp2/housing.stl) | ~25.9 g PLA; STL in stl/ | stamp tool (future) |
| tc-stamp2-hub | [STAMP2 hub (printed)](../stl/heads_stamp2/hub.stl) | ~5.6 g PLA; STL in stl/ | stamp tool (future) |
| tc-stamp2-frame | [STAMP2 frame (printed)](../stl/heads_stamp2/frame.stl) | ~7.7 g PLA; STL in stl/ | stamp tool (future) |
| tc-stamp2-carriage | [STAMP2 carriage (printed)](../stl/heads_stamp2/carriage.stl) | ~2.2 g PLA; STL in stl/ | stamp tool (future) |
| tc-stamp2-rack | [STAMP2 rack (printed)](../stl/heads_stamp2/rack.stl) | ~1.5 g PLA; STL in stl/ | stamp tool (future) |
| tc-stamp2-die-rotor | [STAMP2 die rotor (printed)](../stl/heads_stamp2/die_rotor.stl) | ~7.5 g PLA; STL in stl/ | stamp tool (future) |
| tc-stamp2-die-cap | [STAMP2 die cap (printed)](../stl/heads_stamp2/die_cap.stl) | ~1.6 g TPU; STL in stl/ | stamp tool (future) |
| tc-fr1-plate | [FR1 plate (printed)](../stl/tool_fr1/plate.stl) | ~25.3 g PLA; STL in stl/ | fin-ray gripper tool (future) |
| tc-fr1-pins | [FR1 pins (printed)](../stl/tool_fr1/pins.stl) | ~0.3 g PLA; STL in stl/ | fin-ray gripper tool (future) |
| tc-fr1-rotor | [FR1 rotor (printed)](../stl/tool_fr1/rotor.stl) | ~3.8 g PLA; STL in stl/ | fin-ray gripper tool (future) |
| tc-fr1-shaft | [FR1 shaft (printed)](../stl/tool_fr1/shaft.stl) | ~1.5 g PLA; STL in stl/ | fin-ray gripper tool (future) |
| tc-fr1-body | [FR1 body (printed)](../stl/tool_fr1/body.stl) | ~49.2 g PLA; STL in stl/ | fin-ray gripper tool (future) |
| tc-fr1-cover | [FR1 cover (printed)](../stl/tool_fr1/cover.stl) | ~9.8 g PLA; STL in stl/ | fin-ray gripper tool (future) |
| tc-fr1-pinion | [FR1 pinion (printed)](../stl/tool_fr1/pinion.stl) | ~2.0 g PLA; STL in stl/ | fin-ray gripper tool (future) |
| tc-fr1-jaw1 | [FR1 jaw1 (printed)](../stl/tool_fr1/jaw1.stl) | ~15.4 g PLA; STL in stl/ | fin-ray gripper tool (future) |
| tc-fr1-jaw2 | [FR1 jaw2 (printed)](../stl/tool_fr1/jaw2.stl) | ~15.4 g PLA; STL in stl/ | fin-ray gripper tool (future) |
| tc-fr1-finray1 | [FR1 finray1 (printed)](../stl/tool_fr1/finray1.stl) | ~11.1 g TPU; STL in stl/ | fin-ray gripper tool (future) |
| tc-fr1-finray2 | [FR1 finray2 (printed)](../stl/tool_fr1/finray2.stl) | ~11.1 g TPU; STL in stl/ | fin-ray gripper tool (future) |
| tc-gt1-gt-cover | [GT1 gt cover (printed)](../stl/tool_gt1/gt_cover.stl) | ~10.0 g PLA; STL in stl/ | gripper tool (future) |
| tc-gt1-gt-finger | [GT1 gt finger (printed)](../stl/tool_gt1/gt_finger.stl) | ~15.5 g PLA; STL in stl/ | gripper tool (future) |
| tc-gt1-gt-jaw1 | [GT1 gt jaw1 (printed)](../stl/tool_gt1/gt_jaw1.stl) | ~10.5 g PLA; STL in stl/ | gripper tool (future) |
| tc-gt1-gt-jaw2 | [GT1 gt jaw2 (printed)](../stl/tool_gt1/gt_jaw2.stl) | ~10.5 g PLA; STL in stl/ | gripper tool (future) |
| tc-gt1-gt-pinion | [GT1 gt pinion (printed)](../stl/tool_gt1/gt_pinion.stl) | ~2.0 g PLA; STL in stl/ | gripper tool (future) |
| tc-gt1-gt-shaft | [GT1 gt shaft (printed)](../stl/tool_gt1/gt_shaft.stl) | ~1.5 g PLA; STL in stl/ | gripper tool (future) |
