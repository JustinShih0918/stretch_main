# Vendored: Object_scene_graph

A snapshot of https://github.com/JustinShih0918/Object_scene_graph, copied here so it can be
modified for stretch_main without touching the original repo. **This copy is the one to edit.**
It does not track upstream automatically.

| | |
|---|---|
| Source | `JustinShih0918/Object_scene_graph`, branch `feat/ros2` |
| Commit | `44c1ec6a4ff0240f145d64b122b6c6ab4de9e1f8` |
| Copied | 2026-10-19, via `git archive` (no history) |

## Local changes vs. upstream

- Added `configs/experiment/isaac_hospital_map.yaml` and `configs/ros2/isaac_hospital.yaml`
  (Isaac hospital preset, recovered from the sim container used for the 2026-09-23 demo).
- Added `COLCON_IGNORE`: this is a plain Python project, not a ROS package (the bridge runs
  via `scripts/ros2/bridge.sh`), and its ASCENT third_party tree must not be picked up by colcon.
- Dropped `.claude/settings.local.json` (machine-local permissions); `.claude/skills/` kept.
- `relative_work/ascent` stays a git submodule (Zeying-Gong/ascent, pinned at `8f7bbf9`),
  registered in stretch_main's `.gitmodules`.
- Pinned `isaac_hospital_map` in `tests/unit/golden/experiment_fingerprints.json` (regenerated;
  that one line is the only change).
- **Removed what stretch_main doesn't use** (all still upstream if needed):
  - `docker/`: OSG runs in stretch_main's sim container instead. `docker/ascent-requirements.txt`
    moved to `requirements/ascent-requirements.txt` (read by `install_osg_envs.sh`).
  - `tests/unit/test_docker_build_context.py`: it only tested the removed Dockerfiles.
  - `scripts/legacy/`: one-off probes, which upstream's own rule says to delete.
  - `scripts/video/` and `docs/VIDEO.md`: the narrated paper video. This is not our demo
    recorder, which is `scripts/scene_graph_demo/`.
  - `docs/DEMO_FLOW.md`: the Jetson Thor two-floor demo checklist, which depends on the removed
    Thor compose files.
- Docs: a "vendored copy" note at the top of `README.md`, `docs/SETUP.md` and `docs/USAGE.md`
  (their docker instructions describe upstream and don't apply here). `CLAUDE.md`'s
  Environment section now describes the sim container. Links to removed files are dropped.

Unit tests after these changes: 1493 passed. One known error remains: `test_room_classifier`
needs the gitignored `data/priors/place365_room_map.json`.

Add an entry above whenever you change something here, so a later re-sync from upstream
knows what to keep.

## Pulling upstream changes later

```bash
git -C ~/Warehouse/Object_scene_graph fetch origin
git -C ~/Warehouse/Object_scene_graph diff 44c1ec6 origin/feat/ros2 > /tmp/osg.patch
git apply --directory=src/semantic_nav/object_scene_graph /tmp/osg.patch   # resolve conflicts, update the commit above
```

Running it with Isaac Sim: see [scripts/scene_graph_demo/README.md](../../../scripts/scene_graph_demo/README.md).
