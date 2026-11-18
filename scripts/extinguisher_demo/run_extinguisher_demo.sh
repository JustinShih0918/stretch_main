#!/usr/bin/env bash
# Record the fire-extinguisher inspection demo: one take per extinguisher condition.
#
#   ./scripts/extinguisher_demo/start_stack.sh                    # once; see README.md
#   ./scripts/extinguisher_demo/run_extinguisher_demo.sh bad good      # a take of each
#
# Each take: reset Isaac with the extinguisher in that condition (open_extinguisher.py's
# control file) -> restart the bridge -> clear costmaps -> check the robot is at
# spawn -> record -> run_robot.py +experiment=isaac_extinguisher_inspect (scene-graph
# search for the extinguisher, then the VLM inspection) -> hold the verdict -> stop.
#
# The video is the HUD's display ($HUD_DISPLAY, demo_hud.py) with Isaac's
# viewport cropped out of $ISAAC_DISPLAY and composed into it live by one ffmpeg.
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
OSG="$WS/src/semantic_nav/object_scene_graph"

[ $# -gt 0 ] || { echo "usage: $0 <good|bad> [<good|bad> ...]" >&2; exit 1; }

SESSION=${SESSION:-extinguisher}
LOGS=${LOGS:-$HOME/sim_nav_ws/extinguisher}
ENV_FILE=${ENV_FILE:-$LOGS/env.sh}
EXTINGUISHER_CONTROL=${EXTINGUISHER_CONTROL:-$LOGS/condition}
BRIDGE_ADDR=${BRIDGE_ADDR:-127.0.0.1:18765}
NVIDIA_KEY_FILE=${NVIDIA_KEY_FILE:-$HOME/.nvidia_api.env}   # exports NVIDIA_API_KEY
OUT=${OUT:-$WS/videos/extinguisher_demo}
MAX_STEPS=${MAX_STEPS:-300}
# map: explore, build the scene graph, find and inspect the extinguisher, keep the map.
# search: start from that saved map (MAP_TAG) and go straight to the extinguisher.
MODE=${MODE:-map}
MAP_TAG=${MAP_TAG:-isaac_hospital_extinguisher}
# Which extinguisher, when there is more than one: "fire extinguisher #1". Empty
# = the preset's target, any extinguisher.
TARGET=${TARGET:-}
# Extra Hydra overrides for run_robot.py, space-separated, e.g.
# OSG_ARGS="ros2.waypoint_xy=[4.0,4.5]" (README "Far placements").
OSG_ARGS=${OSG_ARGS:-}
# For compile_demo.py's card before the take, saved beside it as take*.json:
# SCENE_TITLE and SCENE (what is set up and what to watch for), TRUTHS (each
# instance's real condition, "1=bad,2=good", so the cut can say whether the one
# inspected was judged right) and TRUTH_TEXT (the card's ground-truth line).
SCENE_TITLE=${SCENE_TITLE:-}
SCENE=${SCENE:-}
TRUTHS=${TRUTHS:-}
TRUTH_TEXT=${TRUTH_TEXT:-}
START_INDEX=${START_INDEX:-1}
ISAAC_DISPLAY=${ISAAC_DISPLAY:-:5}
HUD_DISPLAY=${HUD_DISPLAY:-:6}
# Isaac's 16:9 render inside its viewport, below the viewport's toolbar, with
# EXTINGUISHER_CLEAN_UI=1 and a 1920x1080 window. Re-measure if the layout differs.
VIEWPORT_CROP=${VIEWPORT_CROP:-1735x976+116,66}
# Where demo_hud.py leaves the hole for it (VIEWPORT, under its header).
VIEWPORT_SLOT=${VIEWPORT_SLOT:-1152x648+16+146}
HOLD_S=${HOLD_S:-8}

# shellcheck disable=SC1090
source "$ENV_FILE"
[ -f "$NVIDIA_KEY_FILE" ] && source "$NVIDIA_KEY_FILE"
[ -n "$NVIDIA_API_KEY" ] || { echo "NVIDIA_API_KEY is not set (looked in $NVIDIA_KEY_FILE)" >&2; exit 1; }
mkdir -p "$OUT"
LOG=$OUT/demo_log.txt
BRIDGE_LOG=$LOGS/bridge.log

clear_costmaps() {
  timeout 10 ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap >/dev/null 2>&1
  timeout 10 ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap >/dev/null 2>&1
}

free_ollama() {
  # The ranker's LLM stays resident in ollama for minutes after a run ends, and
  # with Isaac and the model servers that leaves the next run no room to load
  # YOLOE (CUDA OOM at warmup, 2026-11-10). Ollama reloads it on first use.
  local m
  for m in $(curl -s -m 3 http://127.0.0.1:11434/api/ps | python3 -c \
      'import json,sys; print(" ".join(x["name"] for x in json.load(sys.stdin).get("models", [])))' 2>/dev/null); do
    curl -s -m 10 http://127.0.0.1:11434/api/generate -d "{\"model\": \"$m\", \"keep_alive\": 0}" >/dev/null
  done
}

reset_scene() {  # condition token
  echo "[demo] resetting Isaac: extinguisher $1" | tee -a "$LOG"
  free_ollama
  echo "$1 $2" > "$EXTINGUISHER_CONTROL"
  for _ in $(seq 1 60); do [ "$(cat "$EXTINGUISHER_CONTROL.ack" 2>/dev/null)" = "$2" ] && break; sleep 1; done
  [ "$(cat "$EXTINGUISHER_CONTROL.ack" 2>/dev/null)" = "$2" ] || { echo "[demo] Isaac never acknowledged the reset" | tee -a "$LOG"; return 1; }
  until timeout 3 ros2 topic echo --once /clock >/dev/null 2>&1; do :; done
  sleep 10                                                             # TF, costmaps, camera settle
  # The bridge's TF buffer runs on wall time and keeps the previous session's
  # (larger-stamped) transforms across Isaac's clock reset: restart it clean.
  : > "$BRIDGE_LOG"
  tmux respawn-window -k -t "$SESSION:bridge"
  # A fresh HUD, so the take does not open on the previous one's verdict.
  tmux respawn-window -k -t "$SESSION:hud" 2>/dev/null
  until grep -q "bridge listening" "$BRIDGE_LOG" 2>/dev/null; do sleep 1; done
  sleep 3
  clear_costmaps
  echo "[demo] bridge restarted, costmaps cleared" | tee -a "$LOG"
  python3 "$WS/scripts/scene_graph_demo/reset_pose.py" 2>&1 | tee -a "$LOG"
  return "${PIPESTATUS[0]}"
}

# A take interrupted part-way (Ctrl-C, a failed reset) must still finalise its
# mp4: ffmpeg runs in the background, where the terminal's SIGINT never reaches
# it, and an mp4 whose recorder is killed hard has no index and will not play.
FF=""
stop_recording() { [ -n "$FF" ] && kill -INT "$FF" 2>/dev/null && wait "$FF"; FF=""; }
trap 'stop_recording; exit 130' INT TERM
trap stop_recording EXIT

crop_w=${VIEWPORT_CROP%%x*}; rest=${VIEWPORT_CROP#*x}; crop_h=${rest%%+*}; crop_xy=${rest#*+}
slot_w=${VIEWPORT_SLOT%%x*}; rest=${VIEWPORT_SLOT#*x}; slot_h=${rest%%+*}; rest=${rest#*+}
slot_x=${rest%%+*}; slot_y=${rest#*+}

i=$((START_INDEX - 1))
for condition in "$@"; do
  i=$((i + 1)); stamp=$(date +%Y%m%d_%H%M%S)
  reset_scene "$condition" "take${i}_$stamp" || { echo "[demo] reset failed, stopping" | tee -a "$LOG"; exit 1; }
  sleep 2
  clear_costmaps                                                       # the check at spawn leaves marks
  V=$OUT/take${i}_${MODE}_${condition}_${stamp}.mp4
  echo "[demo] === take $i: $MODE run, extinguisher $condition -> $V" | tee -a "$LOG"
  MODE="$MODE" CONDITION="$condition" TARGET="$TARGET" MAP_TAG="$MAP_TAG" SCENE_TITLE="$SCENE_TITLE" \
    SCENE="$SCENE" TRUTHS="$TRUTHS" TRUTH_TEXT="$TRUTH_TEXT" python3 -c 'import json, os, sys
keys = ("MODE", "CONDITION", "TARGET", "MAP_TAG", "SCENE_TITLE", "SCENE", "TRUTHS", "TRUTH_TEXT")
json.dump({k.lower(): os.environ[k] for k in keys if os.environ.get(k)}, open(sys.argv[1], "w"), indent=1)' \
    "${V%.mp4}.json"
  ffmpeg -hide_banner -loglevel error \
    -f x11grab -draw_mouse 0 -framerate 30 -video_size 1920x1080 -i "$HUD_DISPLAY" \
    -f x11grab -draw_mouse 0 -framerate 30 -video_size "${crop_w}x${crop_h}" -i "$ISAAC_DISPLAY+$crop_xy" \
    -filter_complex "[1:v]scale=${slot_w}:${slot_h}:flags=lanczos[vp];[0:v][vp]overlay=${slot_x}:${slot_y}" \
    -c:v h264_nvenc -preset p5 -cq 21 -pix_fmt yuv420p -y "$V" &
  FF=$!
  sleep 2
  (cd "$OSG" && PYTHONPATH=$OSG/src HF_HOME=$OSG/data/weights/hf MAGNUM_LOG=quiet HABITAT_SIM_LOG=quiet \
    /opt/conda/envs/habitat/bin/python scripts/run_robot.py +experiment=isaac_extinguisher_inspect \
    "ros2.bridge_addr=$BRIDGE_ADDR" "agent.max_steps=$MAX_STEPS" "ros2.map_mode=$MODE" \
    "ros2.map_tag=$MAP_TAG" exploration.giveup_after_steps=15 ros2.nav_timeout_s=15 \
    ${TARGET:+"ros2.target='$TARGET'"} $OSG_ARGS) 2>&1 \
    | tee "${V%.mp4}.log" | grep --line-buffered '^\[robot\]' | tee -a "$LOG"
  sleep "$HOLD_S"                                                      # hold the verdict on screen
  stop_recording
  echo "[demo] take $i done" | tee -a "$LOG"
done
echo "[demo] ALL DONE" | tee -a "$LOG"
