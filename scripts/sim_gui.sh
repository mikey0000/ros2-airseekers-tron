#!/usr/bin/env bash
# Headless mower_sim + the MowgliNext web GUI on a dev host (docs/simulation_gui.md).
#
#   ./scripts/sim_gui.sh up [world] [launch_arg:=value ...]   # default world: garden
#   ./scripts/sim_gui.sh down
#   ./scripts/sim_gui.sh status
#   ./scripts/sim_gui.sh ros2 <args...>     # ros2 CLI on the sim's domain (topic echo, service call)
#   ./scripts/sim_gui.sh build-gui           # rebuild OUR GUI (third_party/mowglinext/gui) -> mower-gui:amd64
#
# Then open http://localhost:4006. Build first:
#   DEV_ROS_DOMAIN_ID=90 ./scripts/dev_build.sh build --packages-up-to mower_sim mower_bringup
#
# Isolation: every ROS process runs with ROS_LOCALHOST_ONLY=1 and SIM_ROS_DOMAIN_ID (default 90),
# never domain 0, so nothing joins the robot's DDS graph over the LAN (2026-10-07: a dev-host
# test feeder latched a lethal boundary stop on the mower). Ports 4006/8765/8766 bind on this
# host only; nothing here talks to the mower.
#
# The GUI reads its config (datum) from a scratch dir, never config/gui: the seed yaml is copied
# into SIM_GUI_DIR/maps and gui_bridge rewrites its datum to the world's.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"
IMG="${SIM_IMAGE:-mower:jazzy-dev-amd64}"
# mower-gui:amd64 = built from THIS tree by ./gui/build.sh (ARCH=amd64). mower-gui:latest on a dev
# box can be a stale older build (2026-10-09: no drawn-path tools), so it is not the default.
GUI_IMG="${GUI_IMAGE:-mower-gui:amd64}"
DOMAIN="${SIM_ROS_DOMAIN_ID:-90}"
DIR="${SIM_GUI_DIR:-/tmp/mower_sim_gui.$DOMAIN}"
SIM=mower_sim_gui_sim
GUI=mower_sim_gui_gui

[[ "$DOMAIN" =~ ^[0-9]+$ ]] && (( DOMAIN > 0 && DOMAIN < 233 )) \
  || { echo "SIM_ROS_DOMAIN_ID must be 1..232 (not 0, the robot's domain)" >&2; exit 2; }

port_busy() { ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]$1\$"; }

# Warn when the vendored GUI source changed after the image was built (the GUI then lacks it).
gui_stale_warning() {
  local built newest
  built=$(date -d "$(docker image inspect "$GUI_IMG" --format '{{.Created}}')" +%s 2>/dev/null) || return 0
  newest=$(find "$STACK_ROOT/third_party/mowglinext/gui" \( -name node_modules -o -name dist \) -prune \
           -o -type f -printf '%T@\n' 2>/dev/null | sort -n | tail -1 | cut -d. -f1)
  [[ -n "$newest" && "$newest" -gt "$built" ]] && echo "WARNING: $GUI_IMG is older than the GUI source" \
    "in third_party/mowglinext/gui: run ./scripts/sim_gui.sh build-gui" >&2
  return 0
}

up() {
  local world="${1:-garden}"; shift || true
  [[ -f "$STACK_ROOT/install/setup.bash" ]] || { echo "no install/: build first (see header)" >&2; exit 1; }
  docker image inspect "$GUI_IMG" >/dev/null 2>&1 \
    || { echo "no $GUI_IMG: run ./scripts/sim_gui.sh build-gui (or set GUI_IMAGE)" >&2; exit 1; }
  gui_stale_warning
  for p in 4006 8765 8766; do
    port_busy "$p" && { echo "port $p already in use (another sim/GUI?): ./scripts/sim_gui.sh down" >&2; exit 1; }
  done
  # fresh scratch dir (files are root-owned from the containers: clean it inside one), unless
  # keep_maps:=true: then the GUI-drawn areas / paths in $DIR/maps survive (sim.launch.py keep_maps)
  mkdir -p "$DIR"
  if [[ " $* " == *" keep_maps:=true "* && -f "$DIR/maps/areas.dat" ]]; then
    echo "keep_maps:=true: reusing $DIR/maps/areas.dat"
    docker run --rm -v "$DIR":/d "$IMG" bash -c 'rm -rf /d/db; rm -f /d/maps/coverage_resume.txt'
  else
    docker run --rm -v "$DIR":/d "$IMG" find /d -mindepth 1 -delete
  fi
  mkdir -p "$DIR/maps" "$DIR/db"
  [[ -f "$DIR/maps/mowgli_robot.yaml" ]] || cp "$STACK_ROOT/config/gui/mowgli_robot.yaml.seed" "$DIR/maps/mowgli_robot.yaml"

  docker run -d --rm --name "$SIM" --network host \
    -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="$DOMAIN" \
    -v "$STACK_ROOT":/work -v "$DIR":"$DIR" -w /work "$IMG" bash -c \
    "source install/setup.bash; exec ros2 launch mower_bringup sim.launch.py world:=$world \
     maps_dir:=$DIR/maps foxglove:=true teleop_relay:=true $*" >/dev/null

  # wait for foxglove_bridge, so the GUI connects on its first try
  for _ in $(seq 60); do port_busy 8765 && break; sleep 1; done
  port_busy 8765 || { echo "foxglove_bridge did not come up: docker logs $SIM" >&2; exit 1; }

  docker run -d --rm --name "$GUI" --network host \
    -e FOXGLOVE_URL=ws://localhost:8765 \
    -e MOWER_YAML_CONFIG_FILE=/mowgli_config/mowgli_robot.yaml \
    -e DB_PATH=/db -e ONBOARDING_COMPLETED=true -e ROBOT_PROFILE=AirseekersTron \
    -e MQTT_ENABLED=false -e HOMEKIT_ENABLED=false -e GPS_CONTAINER_NAME= \
    -v "$DIR/maps":/mowgli_config -v "$DIR/maps":/ros2_ws/maps -v "$DIR/db":/db \
    "$GUI_IMG" >/dev/null
  echo "sim ($world, ROS_DOMAIN_ID=$DOMAIN, localhost only) + GUI up: http://localhost:4006"
  echo "logs: docker logs -f $SIM | docker logs -f $GUI    scratch: $DIR"
}

down() { docker stop "$GUI" "$SIM" 2>/dev/null || true; }

status() { docker ps --filter "name=mower_sim_gui_" --format '{{.Names}}\t{{.Status}}'; }

roscli() {
  docker exec -i $([[ -t 0 ]] && echo -t) "$SIM" bash -c "source /work/install/setup.bash; ros2 $(printf '%q ' "$@")"
}

case "${1:-}" in
  up) shift; up "$@" ;;
  build-gui) ARCH=amd64 "$STACK_ROOT/gui/build.sh" ;;
  down) down ;;
  status) status ;;
  ros2) shift; roscli "$@" ;;
  *) sed -n '2,9p' "$0" >&2; exit 2 ;;
esac
