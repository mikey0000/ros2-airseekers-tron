#!/usr/bin/env bash
# Switch the mower host between the vendor (stock ROS 1 Noetic) software and this ROS 2 stack,
# without reflashing anything. Run ON THE MOWER HOST as root from the stack checkout:
#
#   sudo /userdata/ros2_stack/scripts/switch_stack.sh status          # what runs, what is enabled
#   sudo /userdata/ros2_stack/scripts/switch_stack.sh stock [--yes]   # back to the vendor software
#   sudo /userdata/ros2_stack/scripts/switch_stack.sh ros2  [--yes]   # back to this stack
#
# Without --yes it only prints the plan (dry run). docs/handover.md explains when to use which.
# A step that fails rolls back: the reverse switch runs (every step tried, best effort) and the
# script exits with the failed step's status. "ROLLBACK INCOMPLETE" means check `status`.
#
# stock:
#   1. refuses unless the mower is docked or idle (/behavior_tree_node/high_level_status);
#   2. stops mower_jazzy + mower_gui and sets their restart policy to "no" (images, data and
#      /userdata/ros2_stack stay, so `ros2` can bring them back);
#   3. install_host_services.sh uninstall (vendor udev rules, stereo IMU back in netlink mode,
#      vendor socket buffers, our units removed);
#   4. enables + starts the vendor units that were enabled in the factory snapshot
#      (ros2_port_handoff/07_system_config/systemd/unit_enablement_state.txt), via
#      mower_services.sh enable --all.
#   Never touched: docker itself, the mowgli-remote Tailscale container (owner's tailnet),
#   the vendor tailscaled (stays disabled: it joins the vendor's tailnet), mower-webshell (stays
#   masked), mower-bluetooth (factory: disabled), /userdata data. A reboot afterwards is advised.
#
# ros2: the reverse. Stops + disables the vendor robot units and foxglove (keeps rosmaster,
#   network-auto (base_net_node), vmstat, cam-firewall; mower-lte is left as found, the v1.3.33
#   install disables it and base_net repairs LTE), install_host_services.sh, then restart
#   policy back to unless-stopped and starts mower_jazzy + mower_gui. Nothing here commands motion.
#   rosmaster stays: from v1.3.33 mower-network-auto runs the vendor's base_net_node with
#   `roslaunch --wait`, which never starts a master itself; without it nothing sets usb0's
#   default route (udhcp-resolve.sh no longer does) and LTE fallback is gone.
set -euo pipefail
cd "$(dirname "$0")/.."

CONTAINERS=(mower_jazzy mower_gui)
REMOTE=mowgli-remote
# Vendor units our stack keeps stopped (mower_services.sh STACK) plus the infra unit it does not
# need (foxglove: the vendor install disables it too). rosmaster, LTE, network-auto, vmstat and
# cam-firewall stay as they are in both modes (rosmaster hosts base_net_node, see above).
VENDOR_ROBOT_UNITS=(mower-logic mower-controller mower-planning mower-perception mower-vslam
                    mower-localization mower-cam-keeper mower-webcam mower-rtsp mower-base)
VENDOR_INFRA_OFF=(foxglove)

mode=${1:-}; confirm=${2:-}
rollback_failed=0
run() {
    [[ $confirm == --yes ]] || { echo "would run: $*"; return 0; }
    echo "+ $*"
    "$@" || { local rc=$?; rollback_failed=1; return "$rc"; }
}

mission_state() {
    docker exec mower_jazzy bash -c 'source /opt/ros/jazzy/setup.bash; source /work/install/setup.bash;
        timeout 10 ros2 topic echo --once /behavior_tree_node/high_level_status 2>/dev/null' \
        2>/dev/null | sed -n 's/^state_name: //p' | head -1
}

require_parked() {
    docker ps -q -f name='^mower_jazzy$' -f status=running | grep -q . || return 0
    local s; s=$(mission_state)
    case $s in
        IDLE|IDLE_DOCKED|CHARGING) echo "mission state $s: ok to switch";;
        *) echo "REFUSING: mission state '${s:-unknown}': dock the mower (or STOP it) first" >&2; exit 1;;
    esac
}

status() {
    echo "== containers"
    docker ps -a --format '  {{.Names}}  {{.Status}}' | grep -E "$(IFS='|'; echo "${CONTAINERS[*]}")|$REMOTE" || true
    for c in "${CONTAINERS[@]}" "$REMOTE"; do
        docker inspect -f "  $c restart={{.HostConfig.RestartPolicy.Name}}" "$c" 2>/dev/null || true
    done
    echo "== vendor units"
    for u in "${VENDOR_ROBOT_UNITS[@]}" "${VENDOR_INFRA_OFF[@]}" mower-lte mower-network-auto \
             mower-vmstat mower-cam-firewall mower-bluetooth mower-webshell tailscaled; do
        printf '  %-22s %-9s %s\n' "$u" "$(systemctl is-active "$u.service" 2>/dev/null)" \
            "$(systemctl is-enabled "$u.service" 2>/dev/null)"
    done
    echo "== our host pieces"
    bash scripts/install_host_services.sh status 2>/dev/null | sed 's/^/  /' || true
}

to_stock() {
    for c in "${CONTAINERS[@]}"; do
        run docker update --restart=no "$c"
        run docker stop "$c"
    done
    run bash scripts/install_host_services.sh uninstall
    run bash scripts/mower_services.sh enable --all
}

to_ros2() {
    run bash scripts/mower_services.sh disable
    for u in "${VENDOR_INFRA_OFF[@]}"; do run systemctl disable --now "$u.service"; done
    run bash scripts/install_host_services.sh
    for c in "${CONTAINERS[@]}"; do
        run docker update --restart=unless-stopped "$c"
        run docker start "$c"
    done
}

# EXIT trap while switching: a failed step (set -e) switches back to the mode we came from.
roll_back() {
    local rc=$? back=to_ros2
    [[ $mode == ros2 ]] && back=to_stock
    echo "FAILED (exit $rc) switching to $mode: rolling back ($back)" >&2
    set +e
    rollback_failed=0
    $back
    ((rollback_failed)) && echo "ROLLBACK INCOMPLETE: check '$0 status' and fix by hand" >&2
    exit "$rc"
}

[[ $mode == status ]] && { status; exit 0; }
[[ $mode == stock || $mode == ros2 ]] || { sed -n 2,9p "$0"; exit 2; }
[[ $EUID -eq 0 ]] || { echo "run as root (sudo)" >&2; exit 1; }
[[ $confirm == --yes ]] || echo "DRY RUN (add --yes to apply)"

[[ $mode == stock ]] && require_parked
trap roll_back EXIT
if [[ $mode == stock ]]; then
    to_stock
    trap - EXIT
    echo "Stock software enabled; $REMOTE (Tailscale) left running. Reboot advised: sudo reboot"
else
    to_ros2
    trap - EXIT
    echo "ROS 2 stack enabled. Reboot advised (stereo IMU driver mode): sudo reboot"
fi
