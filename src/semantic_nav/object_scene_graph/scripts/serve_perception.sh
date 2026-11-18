#!/usr/bin/env bash
# ASCENT's perception models, as HTTP services, for the OSG side to call.
#
#   bash scripts/serve_perception.sh          # start
#   bash scripts/serve_perception.sh --stop
#
# Process-per-model over HTTP is ASCENT's own architecture (`model_api/`), not a
# workaround: lavis (BLIP-2) and habitat cannot share one environment -- lavis
# needs numpy 1.x builds and transformers pins habitat's env does not have -- so
# each model runs in the `ascent` env and answers on a port.
#
#   blip2itm :13182  value map + commit gate cosine (`value_model: blip2itm`)
#   sam      :13183  MobileSAM, one mask per detection box
#   gdino    :13184  GroundingDINO, the detector half of the stair fusion
#   ram      :13185  RAM++, per-step scene tags for the LLM prompt
#   dfine    :13186  D-FINE, ASCENT's closed-set COCO target detector
set -eo pipefail
cd "$(dirname "$0")/.."

ASCENT_DIR="${ASCENT_DIR:-relative_work/ascent}"
# The merged image (docker/Dockerfile) builds the ascent env at
# /opt/conda/envs/ascent. Containers built before that merge have a
# workspace-local clone instead; fall back to it so both work.
ASCENT_PYTHON="${ASCENT_PYTHON:-/opt/conda/envs/ascent/bin/python}"
[ -x "$ASCENT_PYTHON" ] || ASCENT_PYTHON=/workspace/.conda-envs/ascent/bin/python
SESSION="${OSG_MODEL_SESSION:-osg_models}"

PORTS="13182 13183 13184 13185 13186"

# tmux daemonizes into its own session, so the model servers are NOT in the
# process group of whoever ran this script: killing the session (or the
# control panel stopping its `perception` unit) left all five running and
# holding their ports, and the next start then died one line into Flask with
# "Address already in use" while the panel still reported it up. Reap by what
# they are, not by who started them.
reap() {
    tmux kill-session -t "$SESSION" 2>/dev/null && echo "stopped $SESSION" || echo "no session $SESSION"
    pkill -f 'model_api\.[a-z_]*_out' 2>/dev/null && echo "killed stray model servers" || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        pgrep -f 'model_api\.[a-z_]*_out' >/dev/null 2>&1 || return 0
        sleep 1
    done
    pkill -9 -f 'model_api\.[a-z_]*_out' 2>/dev/null || true
    sleep 1
}

# Who, if anyone, still answers on a model port after the reap. A port held by
# something this script cannot kill (another user's leftovers, another
# container sharing the network) must stop the start rather than produce five
# servers that die on bind.
taken() {
    local busy=""
    for port in $PORTS; do
        if curl -s -o /dev/null -m 1 "http://127.0.0.1:$port/" 2>/dev/null; then busy="$busy $port"; fi
    done
    echo "$busy"
}

if [ "${1:-}" = "--stop" ]; then
    reap
    exit 0
fi

[ -x "$ASCENT_PYTHON" ] || { echo "no ascent env at $ASCENT_PYTHON" >&2; exit 1; }
mkdir -p "$ASCENT_DIR/debug/vlm_logs"
reap
busy="$(taken)"
if [ -n "$busy" ]; then
    echo "ports already in use by something this script did not start:$busy" >&2
    echo "find it with 'ps -eo pid,user,args | grep model_api' and stop it, then start again" >&2
    exit 1
fi
tmux new-session -d -s "$SESSION"

launch() {  # name module port
    tmux new-window -t "$SESSION" -n "$1" \
        "cd $PWD/$ASCENT_DIR && PYTHONPATH=$PWD/$ASCENT_DIR \
         HF_HOME=/workspace/data/weights/hf CUDA_VISIBLE_DEVICES=0 \
         PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
         $ASCENT_PYTHON -m $2 --port $3 2>&1 | tee debug/vlm_logs/$1.log"
}
launch blip2itm model_api.blip2itm_out       13182
launch sam      model_api.sam_out            13183
launch gdino    model_api.grounding_dino_out 13184
launch ram      model_api.ram_out            13185
launch dfine    model_api.dfine_out          13186

cat <<EOF
Started '$SESSION'. Weights load for up to ~90s. Check with:
    curl -s -o /dev/null -w '%{http_code}\\n' http://localhost:13182/blip2itm
Stop with:
    bash scripts/serve_perception.sh --stop
EOF
