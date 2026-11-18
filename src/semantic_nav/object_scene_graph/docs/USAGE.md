> **stretch_main vendored copy.** This repo's `docker/` was removed here: OSG runs inside
> stretch_main's sim container (conda envs from `scripts/scene_graph_demo/install_osg_envs.sh`).
> Setup: [scripts/scene_graph_demo/README.md](../../../../scripts/scene_graph_demo/README.md).
> Local changes: [VENDORED.md](../VENDORED.md).

# How to run OSG's ObjectNav pipeline

The default configuration is OSG's own agent (`nav_agent`) on ASCENT's served
perception models, sensor-only, on the frozen PointNav mover, with the S71 group
settings. This copy is **single-floor**: the ASCENT transcription that S71's
63.0% SR / 0.36 SPL on `scenes20_ep0to4` was measured with
(`docs/history/AB_RESULTS.md`), and all multi-floor support, were removed
(`VENDORED.md`), so that number is not this default's. Everything below runs
inside the sim container.

## 1. One-time setup

**See [docs/SETUP.md](SETUP.md)** — the submodule (`--recursive` is mandatory),
the GroundingDINO CUDA extension that must be compiled, ~5 GB of model weights,
and the datasets. In short:

```bash
git submodule update --init --recursive     # ASCENT + its nested submodules (also needed to build)
bash scripts/fetch_ascent_weights.sh
python scripts/download_weights.py --pointnav
docker exec docker-ollama-1 ollama pull qwen2.5:7b
```

The five ASCENT models run in the `ascent` conda env, which `docker/Dockerfile`
builds alongside `habitat` (`docs/SETUP.md` §5) — BLIP-2's `lavis` and
habitat-sim cannot share an interpreter.

## 2. Start the model servers

```bash
bash scripts/serve_perception.sh          # tmux session `osg_models`, five windows
bash scripts/serve_perception.sh --stop
```

| window | port | role in the pipeline |
|---|---|---|
| `blip2itm` | 13182 | value-map cosine every step; the commit gate latches at ≥ 0.15 |
| `sam` | 13183 | one MobileSAM mask per D-FINE box |
| `gdino` | 13184 | GroundingDINO, for targets D-FINE cannot name (`detector.gdino_target_url`) |
| `ram` | 13185 | RAM++ scene tags for the LLM prompt, once per step |
| `dfine` | 13186 | D-FINE COCO detector, target detections at conf ≥ 0.8 |

Logs land in `relative_work/ascent/debug/vlm_logs/<window>.log`. The GPU
budget with all five plus habitat is ~12 GB.

Only run **one evaluation at a time** against these servers. They hold
per-image state on the model object and are not reentrant — the submodule is
pinned to upstream, without the lock a local patch once added — so concurrent
requests race and the loser gets HTTP 500. The client retries once, which
covers a transient hit, not a second eval.

The run **refuses to start** if any server it needs is down
(`PerceptionUnavailable`, raised by `probe_served_models` before Habitat
loads), and a server that stops answering mid-run raises the same error
instead of returning a neutral value. That is deliberate: a silent 0 from a
served model changes what the agent does, and a 0% run would look like a bad
algorithm.

## 3. Run an evaluation

```bash
python scripts/run_eval.py                                   # the default: 100 episodes, ~3.5 h
python scripts/run_eval.py output_dir=outputs/my_run         # name the run
python scripts/run_eval.py eval.behaviour_log=true           # + per-step trace for the comparison script
python scripts/run_eval.py eval.num_episodes=3               # smoke
bash scripts/run_full_split.sh                               # a long split, supervised: restarts itself until done
```

A full split takes ~60 h and has twice been killed by things unrelated to the
agent (a model server returning HTTP 500 under a concurrent eval; a session
teardown). `scripts/run_full_split.sh` re-derives the unscored episodes from
`episodes.jsonl` before every attempt and relaunches, so a restart can neither
re-run a scored episode nor skip an unscored one; it adopts a run already in
flight, holds a lock so two supervisors cannot race, brings the model servers
back if they died, and gives up after three attempts that score nothing rather
than looping on a real fault. `--status` reports what is left. It is a process
in this container, so it does **not** survive a container restart or a session
teardown — run it again afterwards and it picks up where the record left off.

Outputs, under `output_dir`:

- `episodes.jsonl` — one record per episode: `success`, `spl`, `steps`,
  `distance_to_goal`, `agent_stats` (LLM calls, gate latches, give-ups…), `state_log`, `giveup_log`, `approach_stop_reason`, and with
  `eval.behaviour_log=true` the full `step_trace`.
- `summary.json` — the config used, the algorithm field inventory, SR / SPL /
  mean steps.
- `timing.csv` — per-episode wall time.

## 4. Compare two runs

```bash
python scripts/compare_runs.py outputs/<baseline> outputs/<treatment>
```

pairs two runs episode by episode. At n = 100, SR moves ±3 points on noise, so a
change should move one of the paired trace metrics and not just SR.

## 5. Variants worth running

| command | what it tests |
|---|---|
| `verification.enabled=true` | add OSG's forced-choice VLM verifier on top (needs `NVIDIA_API_KEY`) |
| `+experiment=ycb_dynamic_best` | the authored dynamic benchmark, two passes (`docs/DYNAMIC_EXPERIMENT_PROTOCOL.md`) |
| `eval=hm3d_val_single_floor` | HM3D val restricted to single-floor scenes, which is all this copy can reach |

Pre-S71 presets (`matched_*`, `ycb_*`, `ascent_sensor*`, `full_v1_navmesh`)
pull `configs/legacy_defaults.yaml` first, which restores the old base (YOLOE-11s,
NIM, the VLM verifier, the text-LLM frontier scorer).

## 6. Tests

```bash
pytest tests/unit -q                       # ~800 tests, ~20 s, no GPU or data
python tests/unit/test_config_snapshot.py  # regenerate the config golden after a deliberate default change
```

## 7. Where things are

| | |
|---|---|
| the agent | `src/osg/agent/nav_agent.py` and the handlers beside it (`docs/ARCHITECTURE.md`) |
| the map | `src/osg/mapping/map_layer.py`, `costmap.py`; the scene graph in `src/osg/graph/` |
| served-model clients | `src/osg/perception/ascent_models.py`, `detector.py` (`DFineDetector`), `image_text.py` (`Blip2ItmScorer`) |
| the mover | `src/osg/planning/pointnav_driver.py` (weights bit-identical to ASCENT's) |
| config | `configs/config.yaml` → `agent/s71`, `exploration/s71`, `verification/s71`, `detector/dfine`, `llm/qwen_local`, `scene_graph/place365`, `eval/scenes20_ep0to4` |
| setup | `docs/SETUP.md` — submodule, CUDA extension, weights, datasets |
| results log | `docs/history/AB_RESULTS.md` |
| methodology | `docs/METHODOLOGY.md` — protocol, evidence rules, differences from native ASCENT |
