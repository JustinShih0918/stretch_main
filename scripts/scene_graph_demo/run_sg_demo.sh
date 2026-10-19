#!/usr/bin/env bash
# Record Object_scene_graph (OSG) object-search demos in the Isaac hospital,
# one video + log per target. See README.md in this directory for the setup
# this expects to already be running (Isaac, Nav2, the osg_bridge tmux window).
#
#   ./scripts/scene_graph_demo/run_sg_demo.sh "trash can" wheelchair table
#   START_INDEX=3 MAX_STEPS=250 ./scripts/scene_graph_demo/run_sg_demo.sh table
#
# Each run: reset Isaac (Stop/Play) -> restart the bridge -> clear costmaps ->
# drive back to spawn -> start ffmpeg screen capture -> run_robot.py -> stop.
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"

[ $# -gt 0 ] || { echo "usage: $0 <target> [<target> ...]" >&2; exit 1; }

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
source /opt/ros/humble/setup.bash

OSG_REPO=${OSG_REPO:-$WS/src/semantic_nav/object_scene_graph}  # vendored Object_scene_graph
OSG_PYTHON=${OSG_PYTHON:-/opt/conda/envs/habitat/bin/python}
OUT=${OUT:-$WS/videos/scene_graph_demo}
LOG=$OUT/demo_log.txt
MAX_STEPS=${MAX_STEPS:-200}
START_INDEX=${START_INDEX:-1}                              # first demoN number
BRIDGE_WINDOW=${BRIDGE_WINDOW:-osg_sim:bridge}             # tmux window running bridge.sh
BRIDGE_LOG=${BRIDGE_LOG:-/home/user/sim_nav_ws/bridge.log} # that window's log (tee target)

# Isaac runs on its own X display; Stop/Play are clicked with xdotool. The
# coordinates are the toolbar buttons for a 1920x1200 Isaac window -- re-measure
# (`DISPLAY=:4 xdotool getmouselocation`) if your layout differs.
ISAAC_DISPLAY=${ISAAC_DISPLAY:-:4}
STOP_XY=${STOP_XY:-"472 398"}
PLAY_XY=${PLAY_XY:-"472 360"}
PARK_XY=${PARK_XY:-"1915 1195"}                            # pointer out of the shot
VIDEO_SIZE=${VIDEO_SIZE:-1920x1200}

mkdir -p "$OUT"
cd "$OSG_REPO"
export HF_HOME=${HF_HOME:-/workspace/data/weights/hf} MAGNUM_LOG=quiet HABITAT_SIM_LOG=quiet

clear_costmaps() {
  timeout 10 ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap >/dev/null 2>&1
  timeout 10 ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap >/dev/null 2>&1
}

reset_scene() {
  echo "[demo] resetting Isaac scene (Stop, Play)" | tee -a "$LOG"
  DISPLAY=$ISAAC_DISPLAY xdotool mousemove $STOP_XY click 1; sleep 4   # Stop restores robot + objects
  DISPLAY=$ISAAC_DISPLAY xdotool mousemove $PLAY_XY click 1
  DISPLAY=$ISAAC_DISPLAY xdotool mousemove $PARK_XY
  until timeout 3 ros2 topic echo --once /clock >/dev/null 2>&1; do :; done
  sleep 10                                                             # let TF/costmaps/camera settle
  # The bridge's TF buffer runs on wall time and keeps the previous session's
  # (larger-stamped) transforms across Isaac's clock reset: restart it clean.
  : > "$BRIDGE_LOG"
  tmux respawn-window -k -t "$BRIDGE_WINDOW"
  until grep -q "bridge listening" "$BRIDGE_LOG" 2>/dev/null; do sleep 1; done
  sleep 3
  clear_costmaps
  echo "[demo] bridge restarted, costmaps cleared" | tee -a "$LOG"
  python3 "$HERE/reset_pose.py" 2>&1 | tee -a "$LOG"
  return ${PIPESTATUS[0]}
}

i=$((START_INDEX - 1))
for target in "$@"; do
  i=$((i + 1)); slug=${target// /_}; stamp=$(date +%Y%m%d_%H%M%S)
  reset_scene || { echo "[demo] reset failed, stopping" | tee -a "$LOG"; exit 1; }
  sleep 3
  clear_costmaps                                                       # the drive to spawn leaves marks
  V=$OUT/demo${i}_${slug}_${stamp}.mp4
  echo "[demo] === run $i: target='$target' -> $V" | tee -a "$LOG"
  ffmpeg -hide_banner -loglevel error -f x11grab -framerate 30 -video_size "$VIDEO_SIZE" -i "$ISAAC_DISPLAY" \
    -c:v h264_nvenc -preset p5 -cq 23 -pix_fmt yuv420p -y "$V" &
  FF=$!
  sleep 2
  PYTHONPATH=$OSG_REPO/src "$OSG_PYTHON" scripts/run_robot.py +experiment=isaac_hospital_map \
    "ros2.target=$target" "ros2.map_tag=isaac_hospital_$slug" "agent.max_steps=$MAX_STEPS" \
    exploration.giveup_after_steps=15 ros2.nav_timeout_s=15 2>&1 \
    | tee "$OUT/demo${i}_${slug}_${stamp}.log" | grep --line-buffered '^\[robot\]' | tee -a "$LOG"
  sleep 4                                                              # hold the final frame
  kill -INT $FF; wait $FF
  echo "[demo] run $i done" | tee -a "$LOG"
done
echo "[demo] ALL DONE" | tee -a "$LOG"
