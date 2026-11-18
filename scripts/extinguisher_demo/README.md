# Fire-extinguisher inspection demo (Isaac Sim)

The official demo: the Stretch searches Isaac's hospital for a fire extinguisher with the
object scene graph, then a second, larger vision-language model judges whether the
extinguisher is in good or bad condition. Everything is recorded as one composed video:
Isaac's third-person view, the robot camera with detections, the scene graph (top-down and
as a hierarchy), and the VLM verdict.

```
Isaac Sim (hospital + extinguisher) ──/rgb /depth /tf──▶ osg_bridge ──socket──▶ run_robot.py
        ▲  control file:                                  │                  │ OSG: YOLOE, scene graph,
        │  condition + reset                              ▼                  │ frontier search, approach
 open_extinguisher.py                   Nav2 ◀── navigate_to_pose ───────────┘
                                                                             │ found it (STOP)
                                                                             ▼
                                        osg/inspection: best views ──▶ Nemotron 3.5 Super VL
                                                                             │ (build.nvidia.com)
 demo_hud.py ◀── /osg/detections /osg/costmap /osg/scene_graph_json /osg/mission ◀─┘
```

## What each piece is

| File | What |
|---|---|
| `isaacsim/assets/fire_extinguisher_inspect.usda` | The extinguisher on its floor stand, with a `condition` variant. `good` is the model as published. `bad` is corroded, has its hose torn off and lying on the floor, and has dry-chemical powder spilled beside it. One defect each for the damage gallery: `corroded`, `hose_detached`, `discharged`, `fallen`, `blocked`. `blocked` references Isaac's warehouse cardboard boxes |
| `isaacsim/assets/fetch_fire_extinguisher.sh` | Downloads Poly Haven's CC0 `korean_fire_extinguisher_01` into `isaacsim/assets/fire_extinguisher/` (gitignored) and runs `make_fire_extinguisher.py` |
| `isaacsim/assets/make_fire_extinguisher.py` | Splits that one-mesh model into parts the variants can remove or move (stand, cylinder, head, levers, hose, tag). Writes a corroded copy of its paint and the powder decal. Runs with Isaac's bundled Python |
| `isaacsim/assets/stretch3_og_hospital_extinguisher.usda` | The hospital stage plus the extinguisher against a corridor wall at world (−7.5, 5.12), and a light over it |
| `open_extinguisher.py` | Isaac `--exec` script: opens that stage, sets the condition, presses Play, then follows a control file (switch condition, reset the scene) |
| `start_stack.sh` | Starts every process in tmux session `extinguisher`, skipping what is already up |
| `run_extinguisher_demo.sh` | The recorder: one take per condition argument |
| `inspect_gallery.py` | The damage gallery: with the robot parked at the extinguisher, switches through defect variants live and runs the inspection on each |
| `compile_demo.py` | Cuts the takes (and the gallery) into one video with title, per-take and result cards, the driving fast-forwarded |
| `demo_hud.py` | The 1920×1080 screen the video is made of; Isaac's viewport is composited into it |
| `move_extinguisher.sh` | Moves extinguisher #1 or #2 while Isaac plays |
| `far_placements.py` | Seeded wall spots far from #1's place to move it to (floor plan: `hospital_occupancy.png`) |
| OSG `configs/experiment/isaac_extinguisher_inspect.yaml` | The run: `isaac_hospital_map` + target `fire extinguisher` + `inspection=nim`, with an extinguisher checklist |
| OSG `src/osg/inspection/` | View selection, the VLM call, the verdict card |

The search is OSG's, unchanged. The inspection runs once the episode has ended at the
target. It is given the best views the approach collected plus fresh frames from where the
robot stopped. A whole extinguisher beats one cut off at the frame edge, and close beats far.
The VLM returns `{condition, confidence, issues, summary}`. A failed call or a non-answer is
reported as `unknown` with the reason, never as `good`.

The VLM is asked to check the extinguisher against this list, following NFPA 10's monthly
visual checks:
- **Body:** rust, corrosion or dents.
- **Hose and nozzle:** missing, detached or cracked.
- **Operating parts:** handle and lever, safety pin, gauge.
- **Placement:** upright in its stand.
- **Discharge:** powder or residue around it.
- **Access:** not blocked.

The gauge and the pin are too small to read from where the robot stops: under 20 px in its
camera at 1.5 m.

## One-time setup (sim container)

On top of the scene-graph setup in [../scene_graph_demo/README.md](../scene_graph_demo/README.md)
(conda envs, weights, ollama, ffmpeg, xdotool):

```bash
./isaacsim/assets/fetch_fire_extinguisher.sh      # ~110 MB; needs Isaac Sim at ~/isaacsim
# An NVIDIA API key from build.nvidia.com, kept out of the repo:
umask 077; echo 'export NVIDIA_API_KEY=nvapi-...' > ~/.nvidia_api.env
```

Don't re-run the fetch script while Isaac has the stage open. It rewrites `parts.usdc`,
which Isaac holds memory-mapped, and Isaac crashes.

The inspection model is `nvidia/nemotron-3.5-super-vl-120b-a12b`, a hosted reasoning VLM
(text, image and video input). With reasoning on, one inspection usually takes 7–15 s. A clean
object can set it second-guessing until it runs out of `inspection.max_tokens` (2048). It is
then asked once more with thinking off, which takes about 2 s; the report and the HUD mark
that answer as `fallback`. Set `inspection.thinking=false` to always answer directly.

## Bring the stack up

```bash
./scripts/extinguisher_demo/start_stack.sh       # ~1 min until Isaac logs "[open_extinguisher] PLAYING"
```

Defaults: Isaac on `:5`, the HUD on `:6` (both Xvfb, 1920×1080), `ROS_DOMAIN_ID=0`, bridge on
`127.0.0.1:18765`. Override them with environment variables, for example
`ROS_DOMAIN_ID=42 BRIDGE_ADDR=127.0.0.1:18766` to run beside another stack. Every process
shares `~/sim_nav_ws/extinguisher/env.sh`, so CycloneDDS and the domain always match. Pass the
same `BRIDGE_ADDR` to the recorder.

## Record

The demo is two kinds of run, which is how the scene graph is meant to be used.

```bash
# 1. Map run: explore, build the scene graph, find the extinguisher, inspect it, save the map.
MODE=map ./scripts/extinguisher_demo/run_extinguisher_demo.sh good
#    keep the map it wrote, so later map runs don't overwrite it:
cd src/semantic_nav/object_scene_graph/outputs/robot_maps
cp isaac_hospital_extinguisher.json isaac_hospital_extinguisher_day1.json
cp isaac_hospital_extinguisher.npz  isaac_hospital_extinguisher_day1.npz
cd -

# 2. Search runs: start from that scene graph, go straight to the extinguisher, inspect.
MODE=search MAP_TAG=isaac_hospital_extinguisher_day1 START_INDEX=2 \
    ./scripts/extinguisher_demo/run_extinguisher_demo.sh bad good
```

Each take resets Isaac through the control file, so the robot and the extinguisher go back to
their authored poses with the chosen condition. It then restarts the bridge, clears the
costmaps, checks the robot is at spawn, records, runs, and holds the verdict for `HOLD_S`
seconds.

Outputs, all gitignored:
- **`videos/extinguisher_demo/`:** `take<N>_<mode>_<condition>_<stamp>.mp4`, the `.log`
  beside it, and `demo_log.txt` with only the `[demo]`/`[robot]` lines.
- **The OSG run directory**, named in the take's last `[robot]` line: `inspection.json`,
  `inspection_view*.jpg` (exactly what the VLM saw), `inspection_card.jpg` and `stream.jsonl`.

### Damage gallery (optional)

One recorded take per kind of damage would be slow. The gallery shows the inspection alone
across six one-defect extinguishers. For each condition, the script switches the extinguisher
live (the control file without a token, so nothing resets) and takes fresh frames from the
robot's camera. It then runs the end of a mission on them: YOLOE with the mission's
vocabulary, `TargetViewBuffer` to choose the views, and `ConditionInspector` with the preset's
prompt and checklist.

```bash
source ~/.nvidia_api.env
cd src/semantic_nav/object_scene_graph
PYTHONPATH=src HF_HOME=$PWD/data/weights/hf /opt/conda/envs/habitat/bin/python \
    ../../../scripts/extinguisher_demo/inspect_gallery.py \
    corroded hose_detached discharged fallen blocked \
    --bridge "$BRIDGE_ADDR" --target-xy=-7.5,5.12 --out ../../../videos/extinguisher_demo/gallery
cd -
```

First it drives the camera to `--stand-off` (1.5 m) from the extinguisher, along the line it
is already on, and turns to face it. Every condition is shot from that one pose. The head
camera is 1.3 m up and looks level, so the floor is in frame only from 1.3 m out; what the
variants put on the floor lies beside or behind the stand. From 1.9 m the extinguisher was
too small for the VLM, which called a corroded and a fallen extinguisher "good" from there.

For each condition it writes, into `gallery/<condition>/`:
- `inspection.json`
- `inspection_view*.jpg`: what the VLM saw
- `robot_frame.jpg`: the frame with YOLOE's boxes
- `viewport.jpg`: Isaac's view

`--no-vlm` checks framing and detection without calling the model. If the GPU is full,
`--set detector.device=cpu --set detector.half=false` runs YOLOE on the CPU.

Measured 2026-11-10 with the camera 1.6 m off, all five called **BAD**, each naming the
defect staged:

| Condition | VLM issues |
|---|---|
| `corroded` | rust or corrosion visible on cylinder and stand |
| `hose_detached` | hose detached from extinguisher and lying on floor |
| `discharged` | white powder residue on floor around base |
| `fallen` | extinguisher leaning on floor instead of mounted upright in bracket |
| `blocked` | access blocked by cardboard boxes |

Tried and dropped, because the VLM called them `good` from 1.5 m:
- **The hose missing outright.** Its absence is only a few pixels on a 0.2 m body.
- **The levers missing.** Same problem.
- **A soot-blackened, scorched body.** In another run it said "reddish stains" instead.

From 1.9 m it also missed the corrosion and the fallen extinguisher. Two settings helped:
- **Crops:** the preset's `crop_min_aspect` and `crop_upscale_px` widen and enlarge them.
- **Distance:** shooting from 1.5 m.

The demo's takes, also 2026-11-10:
- **Map run (good):** explored 83 steps and answered GOOD.
- **Bad search:** BAD 0.95, naming the powder and the dropped hose but not the rust.
- **Good search:** GOOD 0.90.

### Cut the video

```bash
python3 scripts/extinguisher_demo/compile_demo.py videos/extinguisher_demo/take{1,2,3}_*.mp4 \
    --gallery videos/extinguisher_demo/gallery -o videos/extinguisher_demo/fire_extinguisher_demo.mp4
```

The cut is built in this order:
1. **Title card.**
2. **A card before each take** stating the extinguisher's real condition, so a viewer can check
   the VLM.
3. **The takes, trimmed.** Each loses its model-loading pre-roll. All of a take's driving plays
   at `--fast` speed (4×, with a badge), both the exploring in a map run and the drive to the
   extinguisher in a search run. It returns to real time 1.5 s before the VLM is asked, so the
   inspection and the verdict play as they happened. That point is found in the video: the
   HUD's VLM panel turns orange.
4. **The damage gallery**, after the first take of a damaged extinguisher: 4 s per defect, the
   image the VLM saw beside its answer and the defect staged.
5. **An end card** with every verdict.

It writes a 1080p master and a `_720p.mp4` copy for sharing.

| Variable | Default | |
|---|---|---|
| `MODE` / `MAP_TAG` | `map` / `isaac_hospital_extinguisher` | map or search run, and which map |
| `TARGET` | the preset's | `"fire extinguisher #1"`: one of several (see "Two extinguishers") |
| `SCENE_TITLE` / `SCENE` / `TRUTH_TEXT` | | the cut's card before this take: its headline, what is set up, the ground-truth line. Saved beside the take as `take*.json` |
| `TRUTHS` | | each instance's real condition, `1=bad,2=good`: the cut then takes the real condition from the extinguisher the run inspected, and its end card shows mission, inspected and verdict |
| `MAX_STEPS` | `300` | step budget per take |
| `START_INDEX` | `1` | the first take number |
| `HOLD_S` | `8` | seconds the verdict stays on screen |
| `VIEWPORT_CROP` | `1735x976+116,66` | Isaac's render inside its window (`EXTINGUISHER_CLEAN_UI=1`, 1920×1080) |
| `NVIDIA_KEY_FILE` | `~/.nvidia_api.env` | sourced for `NVIDIA_API_KEY` |

## Two extinguishers: "#1", "#2", and moving one

With more than one extinguisher, the scene graph tells them apart: each gets an index (`#1`,
`#2`) in the order the robot found them, a mission can name one, and when one is moved its index
follows it instead of passing to the other (OSG's `src/osg/objects/instances.py`, switched on by
`scene_graph.instance_labels` in `isaac_extinguisher_inspect`). The HUD and the log name them:

```
[robot] fire extinguisher #1 at [-7.5, 5.29]
[robot] fire extinguisher #2 at [-9.41, 5.35]
[robot] fire extinguisher #1 moved: [-7.5, 5.29] -> [-7.48, 1.72] (3.57 m; track 29 -> 74)
[robot] inspecting the fire extinguisher #1 at [-7.6, 1.87]: ...
```

How a move is followed, in one paragraph: the robot goes where #1 was; finds it gone (the
presence filter disbelieves its track, and #2 in the same frame does not count as #1, because a
sighting must be within `instance_match_m` of #1's track and no nearer another instance); keeps
exploring with #2 struck off the candidate list; and the first extinguisher it sees that no
instance explains is claimed by the missing #1. #2 is never re-bound while it is still where it
was. The inspection then judges only views of the extinguisher it stopped at.

```bash
# The stage with a second extinguisher (good, 2.1 m west of the first on the same wall).
# TWO=1 also sets EXTINGUISHER_FOLLOW_CAMERA=close: the third-person camera 1.7 m behind the
# robot with a wider lens, since at the chairs the stage's camera (2.33 m back) ends up on the
# far side of the corridor wall and records a blank yellow wall.
TWO=1 ./scripts/extinguisher_demo/start_stack.sh

# Day 1: map both. The map run stops at the first extinguisher it reaches; from the lobby it
# reaches #1 with #2 in view behind it, and both go into the map.
MODE=map MAP_TAG=isaac_hospital_two_ext ./scripts/extinguisher_demo/run_extinguisher_demo.sh corroded

# Move #1 to the waiting chairs (any time, while Isaac plays; it stays across resets):
./scripts/extinguisher_demo/move_extinguisher.sh 1 -7.6 1.85 -80

# Day 2: ask for #1 (or #2). TARGET="fire extinguisher" alone means any of them.
MODE=search MAP_TAG=isaac_hospital_two_ext TARGET="fire extinguisher #1" \
    ./scripts/extinguisher_demo/run_extinguisher_demo.sh corroded

./scripts/extinguisher_demo/move_extinguisher.sh 1 home        # put it back

# Farther: a seeded spot against a wall, 8-30 m from #1's place, reachable from the lobby.
python3 scripts/extinguisher_demo/far_placements.py --seed 3 --min 8 --max 30 --png /tmp/seed3.png
#   seed 3: 1 -23.65 1.85 100   (16.5 m from #1's place, 25.4 m walk from spawn; ...)
./scripts/extinguisher_demo/move_extinguisher.sh 1 -23.65 1.85 100
```

`corroded` is #1's condition, so the verdict also says which one was inspected: #1 is rusty,
#2 is not.

The HUD shows the mission ("MISSION · fire extinguisher #1") and, under the scene graph, the
decisions as they are made: where each index is, when one is not at its place, and when its
index passes to an extinguisher found elsewhere (the map crosses out the empty place and draws
the move). A recorded set of these takes is cut like the others; give each its card with
`SCENE_TITLE`/`SCENE`/`TRUTH_TEXT` and `TRUTHS=1=bad,2=good`, and the end card lists which
extinguisher each mission asked for and which one it judged. Measured in Isaac on 2026-11-13 (#1 corroded, #2 good, the map above):

| Run | What happened | Verdict |
|---|---|---|
| Before the change: #1 moved to the chairs, sent for "an extinguisher" | stopped at #1's empty place, inspected #2 | GOOD (wrong one) |
| #1 moved to the chairs, sent for #1 | #1's place empty, #2 skipped 23 times, #1 found at the chairs and re-indexed | BAD 0.95 |
| Both where they belong, sent for #1 | straight to #1, #2 2.1 m beyond it left alone | BAD 0.85 |
| #1 moved to the chairs, sent for #2 | straight to #2 | GOOD 0.85 |
| #1 moved beside #2 (0.65 m apart), sent for #1 | stopped short of #1's empty place, looked, found #1 beside #2 and re-indexed it; #2 kept its own track | BAD 0.90 |
| Same as row 2, after an Isaac restart, moved with `move_extinguisher.sh` | the detector never named #1 by the chairs on the way; the robot followed the region proposer's track there, overshot to 0.46 m, backed up 0.94 m, the detector named it, and #1 was claimed at the stop | BAD 0.95 |
| One extinguisher (#2 hidden), the original Day 1 map, sent for "a fire extinguisher" | as before: straight to it, indexed #1 | BAD 0.95 |

Recorded that afternoon as `videos/extinguisher_demo/two_extinguishers/two_extinguishers_demo.mp4`
(the takes beside it; map `isaac_hospital_two_ext_final` from take 1, final code, `TWO=1` camera):

| Take | What happened | Verdict |
|---|---|---|
| 1 map run | both indexed (#1 at [-7.46, 5.03], #2 at [-9.61, 5.26]), stopped at #1 | BAD 0.85 |
| 2 nothing moved, sent for #1 | straight to #1 | BAD 0.85 |
| 3 #1 at the chairs, sent for #1 | #1's place empty, #2's track (pulled 0.40 m off its place) re-anchored, #1 found at the chairs | BAD 0.80 |
| 4 #1 at the chairs, sent for #2 | straight to #2 | GOOD 0.90 |
| 5 #1 beside #2, sent for #1 | #1's place empty, #1 found 0.65 m from #2; two more runs alike | BAD 0.95 (×3) |

How the side-by-side case got there, on the earlier map (`isaac_hospital_two_ext_rec`):

| Code | Runs | Result |
|---|---|---|
| before the drifted rule | 1 | #2's track pulled toward #1, #1 claimed the fresh track at #2's place: #2 inspected as #1, GOOD (wrong one) |
| drifted rule (0.2 m place radius) | 3 | 2 right (one indexed the left-behind track "#3"); 1 never found #1: claimed from afar on a track 0.35 m off it, which the close look then disbelieved (300 steps) |
| + the left-behind track retired | 3 | all three found #1 beside #2 (BAD 0.95, 0.80, 0.90) |

The place radius then went from 0.2 m to `instance_fragment_m` after a run with #1 at the
chairs indexed #2's own detections "#3" (#2's track had drifted 0.46 m, its detections sat
0.24 m from its place). Earlier takes are in `two_extinguishers_earlier/`, `_trials*/` and
`_failed/`. One earlier take 4 got BAD 0.90 for the intact #2 from a single view ("safety pin
not visible"). Two earlier takes 3 inspected #1 right but showed it as "?" on the HUD (claimed
at the stop with no display update; a stop on a proposer track beside #1's); both fixed above.

### Far placements

`far_placements.py --seed N --min 8 --max 30` puts #1 against a wall 8-30 m from its place;
the search then has to find it in a part of the hospital the 87-step map run never saw. Seeds
1-5 (map `isaac_hospital_two_ext_final`, sent for #1):

| Seed | #1 moved to | from its place | walk from spawn |
|---|---|---|---|
| 1 | (2.95, 7.25), east of the lobby | 10.7 m | 5.5 m |
| 2 | (-17.55, 1.65), dark west corridor | 10.6 m | 18.3 m |
| 3 | (-23.65, 1.85), west rooms | 16.5 m | 25.4 m |
| 4 | (6.35, 2.85), east | 14.0 m | 7.3 m |
| 5 | (-33.55, 3.25), far west | 26.1 m | 35.1 m |

| Round (code) | Found | What happened |
|---|---|---|
| 1 (two-extinguisher code, 300 steps) | 0/5 | seeds 1, 4, 5 stopped at the red FIRE EXTINGUISHER wall sign at (-23.2, 5.3), 1.4 m up (seed 5 claimed it as #1 moved), no usable view; seeds 2, 3 drove into a supply cart / height scale Nav2's costmap misses and sat ~200 steps |
| 2 (+ height gate, wedge back-out; 300 steps) | 0/5 | no sign stops, no 200-step wedges; #1 never in the camera's view in seeds 1, 3, 4, 5 (the search goes out from #1's old place, nearest frontier first: west, so the two spots east of the lobby are never visited); seed 2 passed it 5-8 m off in the unlit corridor, not detected; seed 3 stopped at a shelf of red-labelled binders only the region proposer called an extinguisher (no usable view); seed 4 clipped a wheelchair at step 69 and fell over |
| 3 (+ stop check, fall check; 600 steps) | 2/5 reached #1, 0/5 right | seeds 3 and 4 found the moved #1 (16 m and 14 m away, steps 285 and 274) but stopped on the neighbouring track they had committed to before the registry claimed #1 on another, so #1 was inspected as "?", and from unlit crops the VLM called the corroded #1 GOOD both times; seed 1 committed to the track at #2's place two steps before #2 was re-anchored onto it, and inspected #2 for the #1 mission (GOOD; the wrong one); seed 2 not found in 600 steps: it came within 1.9 m of #1, by a supply rack, and the detector never named it; seed 5 fell over in the lobby at step 7 (the fall check ended the run) |
| 4 (+ approach follows the registry, no verdict for another instance, dark images lifted; Isaac restarted) | 0/4 (seeds 1, 3, 4, 5) | #1 never in the camera's view, so none of round 3's fixes was exercised, and no wrong verdict: each search drove to #1's old place, found it empty, and took the nearest frontier from there, which is west. Seeds 1 and 4 stand 5-7 m east of the spawn, in space the 87-step map run never saw, and the robot did not turn back east within 600 steps (round 3's seed 4 got there only by circling the island). Seed 3 drove into a wheelchair at (-12.8, 5.2) and fell over at step 89, the second fall at the wheelchairs west of #2 (round 2's seed 4 at (-11.0, 3.1)) |
| 5 (round 4's code, plus an operator hint `ros2.waypoint_xy=[4.0,4.5]`, the east side of the lobby; 400 steps) | 1/2 found and judged right | seed 4: after #1's old place was seen empty the robot drove to the hint, saw #1, re-indexed it as moved 13.9 m (track 44 -> 124), inspected it where it really stood (6.3, 2.85) and judged it BAD 0.80, right; #2 left alone; 253 steps, 6.7 min. Seed 1 fell over at step 27 on the way to #1's old place, on open floor just past an over-bed table's low base, before the hint was used |

Round 2's fixes and round 3's: a stop where the detector names no extinguisher is taken back
and the search goes on with that place struck off (`eval.attempts: 4`, OSG `run_robot.py`
`_stop_check`; the HUD logs "looking at it" and "no fire extinguisher … after all"), and a
robot that falls over ends the run at once ("the robot fell over at step N").

Without a hint, a far move is an exploration problem. Once #1's old place is seen empty, the
search takes the nearest frontier from there, which in this hospital is west, so #1 was
reached in 2 of 19 unhinted far runs (2 of the 9 given 600 steps), both after 270+ steps. For a live demo, either move #1
within sight of its old place, or give an operator hint that says where to look first:
`OSG_ARGS="ros2.waypoint_xy=[4.0,4.5]" ./run_extinguisher_demo.sh corroded` (any free spot a
few metres from the new place). The robot still has to see #1, tell it from #2 and judge it by
itself; say so when the run is shown. The Stretch also tips over on low furniture bases that
Nav2's laser does not see (wheelchairs, an over-bed table): 4 of 21 far runs, the run then ends
with "the robot fell over".

## Things to know

- **The extinguisher placement is set for the robot's view.** It stands against the corridor
  wall, turned so a robot coming from the lobby sees its gauge and hose side. The gallery's
  layouts (the fallen extinguisher, the powder, the boxes) are laid out toward that side, and
  `fallen` leans on that wall. Moving it means re-checking those.
- **A far sighting can't be placed.** Depth past 5 m reads 0. An extinguisher first seen from
  8 m away had almost no depth under its mask, apart from the edge of the reception desk in
  front of it. The first map run put the extinguisher on the desk corner, 7.6 m short, and
  stopped there. The preset sets OSG's `scene_graph.min_valid_depth_frac: 0.5`, so a
  detection is mapped only when most of its mask is in depth range.
- **YOLOE doesn't recognise an extinguisher lying flat.** It scored one 0.03 against the 0.2
  floor, so `fallen` stands it beside its stand, tipped 25° onto the wall, where YOLOE names
  it at 0.96.
- **Autonomous map runs do not always find the target.** The frontier ranker's choices vary
  from run to run. Retake a map run until the log shows `inspecting the fire extinguisher`. A
  search run starts with the extinguisher already in its graph, so it doesn't depend on the
  explorer.
- **What can still go wrong with two extinguishers (seen while testing, not fixed here):**
  the close look at an empty place can pick a viewpoint on the far side of a wall when the
  costmap has a gap there, and the drive round can stall; and YOLOE does not always name the
  rusty extinguisher by the chairs from a distance, so finding a moved one can take a full
  exploration round. Neither confuses #1 with #2 -- the worst outcome seen after the fixes is
  "inspection FAILED: no usable view", never the wrong extinguisher's verdict.
- **Two extinguishers side by side are the hard case.** 0.65 m apart and seen from across the
  lobby, one's detection can join the other's track; the drifted rule puts the index back on
  the track at its place (above). Seen once, before the left-behind track was retired, and
  not in the six runs since: #1 claimed from afar on a track placed 0.35 m off it, disbelieved
  by the next close look, and #1 not found within the step budget (no wrong verdict). Six
  runs is not many; for a live demo, a move to another part of the room (the chairs) is the
  better-tested one.
- **An index needs the robot to have looked.** A new extinguisher seen in a search run stays
  `?` until the robot has checked where the known ones are: until then it could be one of them,
  moved. A move is only concluded once the missing one's place has been seen empty.
- **Set the follow camera at startup, not while the scene plays.** Changed live (through a
  script inside Kit), it held for one take; the next take's robot fell over 20 s after the
  reset in the open lobby and Isaac stopped publishing `/clock`. The cause was not pinned
  down; `EXTINGUISHER_FOLLOW_CAMERA` applies it before Play.
- **Don't edit a stage file while Isaac has it open.** Isaac shows a "Base USD files have been
  changed" popup inside the recorded viewport. Restart Isaac (kill its tmux window, then run
  `start_stack.sh` again), and restart the bridge after it.
- **Room labels.** The rooms in the graph are numbered, not named: the Places365 room
  classifier needs `data/priors/place365_room_map.json`, which this checkout doesn't have.
- `[robot] POSE JUMP: ~1.1 m` lines at the sim's driving speed are steps that were long, not
  localisation errors. Isaac's TF is ground truth.
