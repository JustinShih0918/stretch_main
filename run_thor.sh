#!/usr/bin/env bash
# Thor side: start every container the deployment needs (docker/thor/compose.yaml):
#   osg-ollama, osg-thor, osg-bridge, stretch-panel (web panel on :$PANEL_PORT).
# rviz is behind a profile and is not started here.
#
#   ./run_thor.sh          # start (default)
#   ./run_thor.sh down     # stop and remove
#   ./run_thor.sh status   # list containers
#   ./run_thor.sh logs     # follow logs (Ctrl-C to quit)
set -e

ROOT="$(cd "$(dirname "$0")" && pwd)"
COMPOSE=(docker compose -f "$ROOT/docker/thor/compose.yaml" --env-file "$ROOT/docker/thor/.env")
export UID GID="$(id -g)"

case "${1:-up}" in
  up)
    "${COMPOSE[@]}" up -d
    "${COMPOSE[@]}" ps --format 'table {{.Name}}\t{{.Status}}'
    PORT="$(grep -E '^PANEL_PORT=' "$ROOT/docker/thor/.env" | cut -d= -f2)"
    IP="$(ip -4 -o addr show wlP1p1s0 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
    echo "Panel: http://${IP:-<thor-ip>}:${PORT:-8080}"
    ;;
  down)   "${COMPOSE[@]}" down ;;
  status) "${COMPOSE[@]}" ps ;;
  logs)   "${COMPOSE[@]}" logs -f --tail 50 ;;
  *) echo "usage: $0 [up|down|status|logs]" >&2; exit 1 ;;
esac
