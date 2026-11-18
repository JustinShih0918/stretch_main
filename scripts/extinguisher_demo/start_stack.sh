#!/usr/bin/env bash
# Bring up everything the extinguisher demo needs, each piece in its own window of one
# tmux session, skipping any piece that is already up:
#
#   xvfb      Isaac's X display ($ISAAC_DISPLAY, 1920x1080)
#   hud_x     a second display for the demo HUD ($HUD_DISPLAY)
#   ollama    the local LLM the frontier ranker uses
#   blip2itm  ASCENT's BLIP-2 value map server (port 13182)
#   ram       ASCENT's RAM++ tagger (port 13185)
#   isaac     Isaac Sim playing the hospital + extinguisher stage (open_extinguisher.py)
#   nav2      stretch3_navigation
#   tf        camera_link -> camera_color_optical_frame
#   bridge    OSG's ROS bridge (scripts/ros2/bridge.sh)
#   hud       demo_hud.py on $HUD_DISPLAY, the screen the video is made of
#
#   ./scripts/extinguisher_demo/start_stack.sh            # then wait for Isaac (~1 min)
#   tmux kill-session -t extinguisher                      # tear it all down
#
# Every process shares $ENV_FILE (CycloneDDS, $ROS_DOMAIN_ID), because actions
# and services do not cross DDS vendors or domains.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
OSG="$WS/src/semantic_nav/object_scene_graph"

SESSION=${SESSION:-extinguisher}
LOGS=${LOGS:-$HOME/sim_nav_ws/extinguisher}
ISAAC_DIR=${ISAAC_DIR:-$HOME/isaacsim}
ISAAC_EXEC=${ISAAC_EXEC:-$HERE/open_extinguisher.py}
ISAAC_DISPLAY=${ISAAC_DISPLAY:-:5}
HUD_DISPLAY=${HUD_DISPLAY:-:6}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
BRIDGE_ADDR=${BRIDGE_ADDR:-127.0.0.1:18765}
EXTINGUISHER_CONDITION=${EXTINGUISHER_CONDITION:-bad}
EXTINGUISHER_CONTROL=${EXTINGUISHER_CONTROL:-$LOGS/condition}
EXTINGUISHER_CLEAN_UI=${EXTINGUISHER_CLEAN_UI:-1}      # viewport only, as run_extinguisher_demo.sh crops it
# The stage Isaac opens. TWO=1 picks the one with a second extinguisher (README,
# "Two extinguishers"); EXTINGUISHER_STAGE names any other.
ASSETS="$(cd "$HERE/../../isaacsim/assets" && pwd)"
if [ "${TWO:-0}" = 1 ]; then
  EXTINGUISHER_STAGE=${EXTINGUISHER_STAGE:-$ASSETS/stretch3_og_hospital_two_extinguishers.usda}
  # #1 is inspected by the waiting chairs there, with a wall 2 m behind the robot.
  EXTINGUISHER_FOLLOW_CAMERA=${EXTINGUISHER_FOLLOW_CAMERA:-close}
fi
EXTINGUISHER_STAGE=${EXTINGUISHER_STAGE:-$ASSETS/stretch3_og_hospital_extinguisher.usda}
ENV_FILE=$LOGS/env.sh

mkdir -p "$LOGS"
cat > "$ENV_FILE" <<EOF
# Written by start_stack.sh: the environment every process of the demo shares.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=$ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
EOF

tmux has-session -t "$SESSION" 2>/dev/null || tmux new-session -d -s "$SESSION" -n main
has_window() { tmux list-windows -t "$SESSION" -F '#W' | grep -qx "$1"; }
window() {  # name command -- start it unless a window of that name exists
    if has_window "$1"; then echo "[stack] $1: already running"; return; fi
    tmux new-window -d -t "$SESSION" -n "$1" "bash -c '$2 2>&1 | tee $LOGS/$1.log'"
    echo "[stack] $1: started"
}
x_up() { [ -S "/tmp/.X11-unix/X${1#:}" ]; }
http_up() { curl -s -o /dev/null -m 2 "$1"; }

x_up "$ISAAC_DISPLAY" || window xvfb "Xvfb $ISAAC_DISPLAY -screen 0 1920x1080x24"
x_up "$HUD_DISPLAY" || window hud_x "Xvfb $HUD_DISPLAY -screen 0 1920x1080x24"
http_up http://127.0.0.1:11434 || window ollama "ollama serve"
for spec in "blip2itm model_api.blip2itm_out 13182" "ram model_api.ram_out 13185"; do
    set -- $spec
    http_up "http://127.0.0.1:$3" || window "$1" "cd $OSG/relative_work/ascent && \
        PYTHONPATH=$OSG/relative_work/ascent HF_HOME=$OSG/data/weights/hf CUDA_VISIBLE_DEVICES=0 \
        PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True /opt/conda/envs/ascent/bin/python -m $2 --port $3"
done
window isaac "source $ENV_FILE; export DISPLAY=$ISAAC_DISPLAY EXTINGUISHER_CONDITION=$EXTINGUISHER_CONDITION \
    EXTINGUISHER_CONTROL=$EXTINGUISHER_CONTROL EXTINGUISHER_CLEAN_UI=$EXTINGUISHER_CLEAN_UI \
    EXTINGUISHER_STAGE=$EXTINGUISHER_STAGE EXTINGUISHER_FOLLOW_CAMERA=${EXTINGUISHER_FOLLOW_CAMERA:-}; cd $ISAAC_DIR && ./isaac-sim.sh \
    --/app/window/width=1920 --/app/window/height=1080 --exec $ISAAC_EXEC"
window nav2 "source $ENV_FILE; ros2 launch stretch3_navigation navigation.launch.py"
window tf "source $ENV_FILE; ros2 run tf2_ros static_transform_publisher --frame-id camera_link \
    --child-frame-id camera_color_optical_frame --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5"
window bridge "source $ENV_FILE; cd $OSG && bash scripts/ros2/bridge.sh \
    +experiment=isaac_extinguisher_inspect ros2.bridge_addr=$BRIDGE_ADDR"
window hud "source $ENV_FILE; DISPLAY=$HUD_DISPLAY python3 $HERE/demo_hud.py"

echo "[stack] logs in $LOGS; Isaac is ready when its log says '[open_extinguisher] PLAYING'"
