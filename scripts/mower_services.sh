#!/usr/bin/env bash
# Stop / start / inspect the Airseekers mower software stack.
#
#   mower_services.sh stop            stop the ROS robot stack incl. the MQTT cloud link (keeps network, rosmaster, cameras' 3A)
#   mower_services.sh stop --all      also stop rosmaster, foxglove, LTE/network switching, vmstat
#   mower_services.sh start [--all]   start them again (reverse order)
#   mower_services.sh disable [--all] stop AND prevent start at boot   (OTA install.sh would re-enable)
#   mower_services.sh enable  [--all] re-enable at boot and start
#   mower_services.sh status          show state of every mower-related unit and ROS node
#
# Order matters: logic (brain + mower_mqtt) first so nothing re-commands the base, base last.
# mower_mqtt_node is launched by mower-logic.service (mower_logic.launch), not by its own unit.

set -uo pipefail

STACK=(
  mower-logic          # behaviour tree, mower_mqtt_node (cloud), mower_light_sound, ekf map
  mower-controller     # navigation + mower_charge (docking)
  mower-planning
  mower-perception     # stereo_ros, seg/det, light_ai, explore, edge mapping
  mower-vslam          # as_vio_node, as_vmap
  mower-localization   # fusion_localization_node
  mower-cam-keeper     # starts base_cameras on demand
  mower-webcam         # web_video_server :8181
  mower-rtsp           # mediamtx :8554
  mower-base           # chassis/cutter MCU driver, BLE, GPS, cameras
)
INFRA=(
  foxglove
  mower-vmstat
  mower-network-auto
  mower-lte
  rosmaster
)
NEVER=(mower-webshell)   # masked on purpose; listed for status only

ALL=0; [[ "${2:-}" == "--all" ]] && ALL=1
units() { local u=("${STACK[@]}"); (( ALL )) && u+=("${INFRA[@]}"); printf '%s\n' "${u[@]}"; }
reverse() { tac; }

kill_stray_nodes() {
  # ROS nodes started outside systemd (e.g. by hand) that live in the vendor workspace
  local pids
  pids=$(pgrep -f '/workspace/devel/|/workspace/src/mower/bin/|roslaunch --wait +mower' || true)
  if [[ -n "$pids" ]]; then
    echo "[stop] killing stray workspace processes: $pids"
    kill $pids 2>/dev/null; sleep 2
    pids=$(pgrep -f '/workspace/devel/|/workspace/src/mower/bin/' || true)
    [[ -n "$pids" ]] && kill -9 $pids 2>/dev/null
  fi
}

show_status() {
  echo "== systemd units"
  for u in "${STACK[@]}" "${INFRA[@]}" "${NEVER[@]}" rkaiq_3A; do
    printf '  %-22s %-9s %s\n' "$u" "$(systemctl is-active "$u.service" 2>/dev/null)" "$(systemctl is-enabled "$u.service" 2>/dev/null)"
  done
  echo "== ROS nodes"
  if source /opt/ros/noetic/setup.bash 2>/dev/null; then
    export ROS_MASTER_URI=http://127.0.0.1:11311
    timeout 5 rosnode list 2>/dev/null | sed 's/^/  /' || echo "  (rosmaster not reachable)"
  fi
  echo "== MQTT link"
  if pgrep -f mower_mqtt_node >/dev/null; then echo "  mower_mqtt_node RUNNING"; else echo "  mower_mqtt_node not running"; fi
  ss -tnp 2>/dev/null | grep -E ':1883|:8883' | sed 's/^/  /' || true
  echo "== serial ports held"
  for p in /proc/[0-9]*; do for fd in "$p"/fd/*; do t=$(readlink "$fd" 2>/dev/null)
    case "$t" in /dev/ttyS1|/dev/ttyS3|/dev/ttyS4|/dev/ttyS9) echo "  $t <- $(tr '\0' ' ' <"$p"/cmdline | cut -c1-70)";; esac; done; done 2>/dev/null | sort -u
}

case "${1:-}" in
  stop)
    for u in $(units); do echo "[stop] $u"; systemctl stop "$u.service" 2>/dev/null; done
    kill_stray_nodes
    # purge stale registrations so `rosnode list` reflects reality
    if (( ! ALL )) && source /opt/ros/noetic/setup.bash 2>/dev/null; then
      export ROS_MASTER_URI=http://127.0.0.1:11311; yes | timeout 20 rosnode cleanup >/dev/null 2>&1 || true
    fi
    show_status ;;
  start)
    for u in $(units | reverse); do echo "[start] $u"; systemctl start "$u.service" 2>/dev/null; done
    show_status ;;
  disable)
    for u in $(units); do echo "[disable] $u"; systemctl disable --now "$u.service" 2>/dev/null; done
    kill_stray_nodes
    echo "NOTE: an OTA run of /workspace/install.sh re-enables everything."
    show_status ;;
  enable)
    for u in $(units | reverse); do echo "[enable] $u"; systemctl enable --now "$u.service" 2>/dev/null; done
    show_status ;;
  status) show_status ;;
  *) sed -n '2,13p' "$0"; exit 1 ;;
esac
