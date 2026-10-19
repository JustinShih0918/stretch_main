# Recording scene-graph demo videos

This guide records Object_scene_graph (OSG) object-search runs in the Isaac hospital as
`.mp4` files: one video per target object, with the robot starting from the same spawn pose
each time.

It assumes the stack from [README.md](README.md) is already up: Isaac playing the hospital,
Nav2, the static camera TF, ollama, and the bridge in tmux window `osg_sim:bridge`. Everything
below runs **inside the sim container**.

## Quick start

```bash
cd ~/stretch_main
./scripts/scene_graph_demo/run_sg_demo.sh "trash can" wheelchair table
```

This produces one video per target in `videos/scene_graph_demo/`. Leave the Isaac window
alone while it runs, because the recorder captures the whole screen and clicks Isaac's
toolbar.

## What one take does

`run_sg_demo.sh` loops over its arguments. For each target:

| # | Step | How | Log line |
|---|---|---|---|
| 1 | Reset the scene | clicks Isaac's **Stop** (restores robot and objects), waits 4 s, clicks **Play**, then parks the pointer in the bottom-right corner | `[demo] resetting Isaac scene (Stop, Play)` |
| 2 | Wait for the sim | blocks until `/clock` publishes again, then waits 10 s for TF, costmaps and camera to settle | |
| 3 | Restart the bridge | `tmux respawn-window -k -t osg_sim:bridge`, then waits for `bridge listening` in the bridge log | |
| 4 | Clear costmaps | `clear_entirely_{global,local}_costmap` | `[demo] bridge restarted, costmaps cleared` |
| 5 | Return to spawn | `reset_pose.py`: a Nav2 goal to world (0, 0, 0°), up to 3 attempts, must end within 0.3 m and 20° | `[reset] OK at spawn` |
| 6 | Clear costmaps again | the drive back leaves marks | `[demo] bridge restarted, costmaps cleared` |
| 7 | Start recording | `ffmpeg -f x11grab` of the whole Isaac display, 30 fps, `h264_nvenc` (preset p5, CQ 23), then waits 2 s | `[demo] === run N: target='…' -> …mp4` |
| 8 | Run the search | `run_robot.py +experiment=isaac_hospital_map ros2.target=<target>` with `agent.max_steps=$MAX_STEPS`, `exploration.giveup_after_steps=15`, `ros2.nav_timeout_s=15` | `[robot] …` |
| 9 | Stop recording | waits 4 s to hold the last frame, then sends SIGINT to ffmpeg so it finalises the mp4 | `[demo] run N done` |

If step 5 fails, the script stops with `[demo] reset failed, stopping` instead of recording
from the wrong start pose.

## Output

Everything goes in `videos/scene_graph_demo/`, which is gitignored:

| File | Content |
|---|---|
| `demoN_<target>_<YYYYmmdd_HHMMSS>.mp4` | screen recording, 1920×1200 |
| `demoN_<target>_<stamp>.log` | the complete `run_robot.py` output (model loading, warnings, `[robot]` lines) |
| `demo_log.txt` | appended across all runs: only the `[demo]`, `[reset]` and `[robot]` lines; read this first |

OSG also writes its own outputs to `src/semantic_nav/object_scene_graph/outputs/`, which is
also gitignored:
- the scene-graph map at `robot_maps/isaac_hospital_<target>.json` (checkpointed every 20 steps)
- per-step artefacts in `<stamp>/` (the path is printed in the run's last `[robot]` line)

`N` counts up from `START_INDEX` (default 1), so retakes can continue the existing numbering
(see below).

## Options

All options are environment variables:

| Variable | Default | Use |
|---|---|---|
| `MAX_STEPS` | `200` | step budget per target; the table run needed `250` |
| `START_INDEX` | `1` | the first `demoN` number |
| `OUT` | `videos/scene_graph_demo` | output directory |
| `ISAAC_DISPLAY` | `:4` | the X display Isaac runs on, which is also the one recorded |
| `VIDEO_SIZE` | `1920x1200` | capture size; must match that display |
| `STOP_XY` / `PLAY_XY` | `472 398` / `472 360` | screen coordinates of Isaac's Stop and Play buttons |
| `PARK_XY` | `1915 1195` | where to leave the mouse pointer during the take |
| `BRIDGE_WINDOW` / `BRIDGE_LOG` | `osg_sim:bridge` / `/home/user/sim_nav_ws/bridge.log` | the bridge's tmux window and log |
| `OSG_REPO` / `OSG_PYTHON` | vendored OSG / `/opt/conda/envs/habitat/bin/python` | which OSG and interpreter to run |

## Before you record

1. **Isaac's layout is fixed.** Isaac must fill `:4` at 1920×1200 with its toolbar where it was
   when the coordinates were measured. If you move or resize anything, measure again: hover over
   each button, run `DISPLAY=:4 xdotool getmouselocation`, and set `STOP_XY` / `PLAY_XY`.
2. **Check the stack is up.** Run `ros2 topic echo --once /clock`, and check that
   `ros2 action list` shows `/navigate_to_pose` and that the bridge log contains
   `bridge listening`. If the bridge window isn't named `osg_sim:bridge`, the recorder waits
   forever at step 3.
3. **Do a short dry run first.** `MAX_STEPS=20 OUT=/tmp/sg_try ./scripts/scene_graph_demo/run_sg_demo.sh chair`
   checks the reset, the clicks and the capture without polluting the real output folder.

## Checking a take

Read `demo_log.txt` after each run:

- **Keep it if** you see `[reset] OK at spawn`, no `POSE JUMP`, and a final
  `[robot] N steps, T s -> outputs/…`.
- **Retake it if** you see `POSE JUMP: x m in one step`. The localisation jumped, not the
  robot, so everything mapped before that step is misplaced and the scene graph in the video
  is wrong.
- **Look at the video if** goals were `refused` (`k goals posted to Nav2, r refused`). A few
  refusals are normal, since some frontiers are unreachable. Many refusals usually mean the
  costmap is stale.
- **Expect** `/map published by: nobody (no map yet?)`. The sim has no `map` frame, so this
  line is harmless.

To retake a single target and keep the numbering:

```bash
START_INDEX=3 MAX_STEPS=250 ./scripts/scene_graph_demo/run_sg_demo.sh table
```

Each take gets a new timestamp, so old takes are never overwritten. Delete or move the bad
ones yourself.

## The 2026-09-23 recordings

These were recorded in two batches: `"trash can" wheelchair` with `MAX_STEPS=200`, then
`table` with `START_INDEX=3 MAX_STEPS=250`.

| Video | Target | Steps | Time | Nav2 goals | Notes |
|---|---|---|---|---|---|
| `demo1_trash_can_20260923_201028.mp4` | trash can | 30 | 72 s | 1 | clean |
| `demo2_wheelchair_20260923_201221.mp4` | wheelchair | 34 | 84 s | 2 | clean |
| `demo3_table_20260923_203930.mp4` | table | 188 | 322 s | 11 (2 refused) | two `POSE JUMP`s around step 170 |

Five earlier takes were thrown away: `bed` once, and `vending machine` four times. They hit
`POSE JUMP`s (up to 2.4 m) or were stopped part-way.

## Sharing the videos

The raw files are large, about 200–780 MB each. They are gitignored on purpose, and GitHub
rejects files over 100 MB anyway. To make a smaller copy for slides or chat:

```bash
ffmpeg -i demo1_trash_can_*.mp4 -vf scale=1280:-2 -c:v libx264 -crf 28 -preset slow -an demo1_small.mp4
```

Share the originals through a drive or a GitHub Release, not the repo.

`OSG_ASCENT_scene_graph.pptx` in the same folder is the slide deck that goes with these
videos. It was generated with `pptxgenjs` from a `build.js` script in a temporary session
directory, and that source was not kept in this repo.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Stuck right after `resetting Isaac scene` | `/clock` never came back, so Play wasn't clicked. Check `STOP_XY`/`PLAY_XY` and that Isaac is on `ISAAC_DISPLAY` |
| Stuck before `bridge restarted` | wrong `BRIDGE_WINDOW`, or the bridge window doesn't tee to `BRIDGE_LOG` |
| `[reset] no navigate_to_pose server` | Nav2 isn't running, or it's on a different RMW. Every process needs `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| `[reset] FAILED to reach spawn` | robot stuck after a bad take. Move it in Isaac or restart the stage, then run again |
| ffmpeg: `Cannot get the image data … Continuing without shared memory` | harmless: x11grab falls back to slower capture, and the video is still written |
| mp4 won't play | ffmpeg was killed hard (Ctrl-C on the script mid-take) and never wrote the file's index. Re-record, or try `ffmpeg -i broken.mp4 -c copy fixed.mp4` |
| Robot drives but the scene graph is off | look for `POSE JUMP` in the `.log` and retake |
