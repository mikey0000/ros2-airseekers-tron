#!/usr/bin/env bash
# Sim gate: one full mow in mower_sim, recorded and judged (docs/simulation.md "Sim gate").
#
#   ./scripts/sim_gate.sh                     # real-map world, ideal drive
#   ./scripts/sim_gate.sh -d                  # same, with the ground disturbance
#   ./scripts/sim_gate.sh -w path/world.yaml -n myrun -t 3000 -- extra:=launch args
#
# START from the dock with the committed parameters: undock -> transit -> mow every
# sub-path -> return -> dock. A bag is recorded; scripts/sim_gate/evaluate.py prints the
# PASS/FAIL table (criteria a-g) and the exit code is 0 only on PASS.
#
# Needs a built workspace (./scripts/dev_build.sh build) and the mower:jazzy-dev-amd64
# image. Isolation: private DDS domain (MOWER_SIM_ROS_DOMAIN_ID, default 175, never 0),
# localhost-only discovery, containers named sim<domain>-gate-<run>; ros2 commands run
# inside the sim container (Fast DDS shared memory). Never set MOWER_SIM_ALLOW_LAN /
# MOWER_SIM_ALLOW_ON_ROBOT for this.
#
# Options:
#   -w WORLD   world YAML (default src/mower_sim/worlds/real_run5.yaml)
#   -n NAME    run name (default gate-<date>); output in $SIM_GATE_OUT/NAME
#              (SIM_GATE_OUT default: <stack>/log/sim_gate)
#   -d         ground disturbance: wheel bias +0.04, slip std 0.05, pivot efficiency 0.85
#              (mower_sim kinematics DriveParams.disturb_*; override with -- disturb_*:=...)
#   -s SEED    disturbance seed (default 1)
#   -t SECS    give up after this long after START (default 2700)
#   -k         keep the sim container running at the end (debugging)
#   -- ARGS    extra sim.launch.py arguments (e.g. mission_params_file:=/abs/path.yaml)
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK="$(dirname "$SCRIPT_DIR")"
IMG=mower:jazzy-dev-amd64
DOMAIN="${MOWER_SIM_ROS_DOMAIN_ID:-175}"
WORLD="$STACK/src/mower_sim/worlds/real_run5.yaml"
NAME="gate-$(date +%Y%m%d-%H%M%S)"
DISTURB=0; SEED=1; TIMEOUT=2700; KEEP=0
while getopts "w:n:ds:t:k" o; do
  case "$o" in
    w) WORLD="$(cd "$(dirname "$OPTARG")" && pwd)/$(basename "$OPTARG")" ;;
    n) NAME="$OPTARG" ;;
    d) DISTURB=1 ;;
    s) SEED="$OPTARG" ;;
    t) TIMEOUT="$OPTARG" ;;
    k) KEEP=1 ;;
    *) sed -n '2,30p' "$0"; exit 2 ;;
  esac
done
shift $((OPTIND - 1))
[ "${1:-}" = "--" ] && shift
EXTRA="$*"

if ! [[ "$DOMAIN" =~ ^[0-9]+$ ]] || [ "$DOMAIN" -le 0 ] || [ "$DOMAIN" -gt 232 ]; then
  echo "sim_gate: MOWER_SIM_ROS_DOMAIN_ID must be 1..232 (never 0, the robot's domain)" >&2
  exit 2
fi
if [ -n "${MOWER_SIM_ALLOW_LAN:-}" ] || [ -n "${MOWER_SIM_ALLOW_ON_ROBOT:-}" ]; then
  echo "sim_gate: refusing to run with MOWER_SIM_ALLOW_LAN / MOWER_SIM_ALLOW_ON_ROBOT set" >&2
  exit 2
fi
[ -f "$WORLD" ] || { echo "sim_gate: world $WORLD not found" >&2; exit 2; }
[ -f "$STACK/install/setup.bash" ] || {
  echo "sim_gate: no install/ - run ./scripts/dev_build.sh build first" >&2; exit 2; }
case "$WORLD" in "$STACK"/*) ;; *) echo "sim_gate: the world must be inside $STACK" >&2; exit 2 ;; esac

OUT="${SIM_GATE_OUT:-$STACK/log/sim_gate}/$NAME"
mkdir -p "$OUT/maps"
C="sim${DOMAIN}-gate-$(echo "$NAME" | tr -c 'A-Za-z0-9_.\n-' '-')"
ENV=(-e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY=1 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
MOUNT=(-v "$STACK:/work" -w /work)
case "$OUT" in "$STACK"/*) ;; *) MOUNT+=(-v "$OUT:$OUT") ;; esac
CWORLD="/work/${WORLD#"$STACK"/}"
COUT="$OUT"; case "$OUT" in "$STACK"/*) COUT="/work/${OUT#"$STACK"/}" ;; esac
DARGS=""
if [ "$DISTURB" = 1 ]; then
  DARGS="disturb_wheel_bias:=0.04 disturb_slip_std:=0.05 disturb_pivot_eff:=0.85 disturb_seed:=$SEED"
fi
ROS="source /opt/ros/jazzy/setup.bash; source install/setup.bash"
cx() { docker exec "$C" bash -c "$ROS; $*" 2>&1 | grep -v "localhost_only\|ROS_LOCALHOST_ONLY" || true; }
state() { cx "timeout 8 ros2 topic echo --once --field state_name /behavior_tree_node/high_level_status" \
  | grep -v '^---' | head -1; }
cleanup() {
  if [ "$KEEP" != 1 ]; then docker stop -t 5 "$C" >/dev/null 2>&1 || true; fi
}
trap cleanup EXIT

{
  echo "world=$WORLD"; echo "domain=$DOMAIN"; echo "disturb=$DISTURB seed=$SEED args=$DARGS"
  echo "extra=$EXTRA"; echo "commit=$(git -C "$STACK" rev-parse --short HEAD 2>/dev/null)"
  echo "dirty=$(git -C "$STACK" status --porcelain 2>/dev/null | grep -c . || true)"
} > "$OUT/run.txt"
echo "sim_gate: run $NAME -> $OUT (container $C, domain $DOMAIN, disturbance $DISTURB)"
docker rm -f "$C" >/dev/null 2>&1 || true
docker run -d --rm --name "$C" --network host "${ENV[@]}" "${MOUNT[@]}" "$IMG" bash -c \
  "$ROS; ros2 launch mower_bringup sim.launch.py world:=$CWORLD localization:=sim \
   maps_dir:=$COUT/maps $DARGS $EXTRA > $COUT/launch.log 2>&1" >/dev/null

# ready: the mission is up and idle on the dock
ready=""; idle=""
for _ in $(seq 1 30); do
  sleep 5
  s="$(state)"
  case "$s" in CHARGING|IDLE_DOCKED) ready="$s"; break ;; IDLE) idle="$s" ;; esac
done
[ -n "$ready" ] || ready="$idle"
[ -n "$ready" ] || { echo "sim_gate: the mission did not come up (see $OUT/launch.log)" >&2; exit 3; }
sleep 10
docker exec -d "$C" bash -c "$ROS; cd $COUT && exec ros2 bag record -s sqlite3 -o bag \
  --include-hidden-topics --max-cache-size 0 \
  /sim/ground_truth /sim/status /behavior_tree_node/high_level_status \
  /behavior_tree_node/mow_plan /behavior_tree_node/mow_progress /cmd_vel /cmd_vel_nav \
  /behavior_tree_node/conditioned_route \
  /cmd_vel_emergency /cmd_vel_docking /cmd_vel_slew/shape_status /map_server_node/outside_m \
  /odometry/filtered_map /mission/incident /plan /rosout /navigate_to_pose/_action/status \
  /follow_path/_action/status /spin/_action/status /mower_docking/dock/_action/status \
  > $COUT/rec.log 2>&1"
sleep 5
date +%s.%N > "$OUT/start_epoch"
cx "ros2 service call /behavior_tree_node/high_level_control mowgli_interfaces/srv/HighLevelControl '{command: 1}'" | tail -1
echo "sim_gate: START sent ($ready)"

end=$(( $(date +%s) + TIMEOUT )); last=""; active=0; verdict_state=""
while [ "$(date +%s)" -lt "$end" ]; do
  sleep 15
  s="$(state)"
  [ -z "$s" ] && continue
  if [ "$s" != "$last" ]; then echo "  $(date +%T) $s"; last="$s"; fi
  case "$s" in
    STUCK_NEEDS_HELP|EMERGENCY|NAV_TO_DOCK_FAILED|UNDOCK_FAILED|MOWING_INCOMPLETE)
      verdict_state="$s"; sleep 5; break ;;
    IDLE_DOCKED|CHARGING|IDLE)
      if [ "$active" = 1 ]; then verdict_state="$s"; sleep 5; break; fi ;;
    *) active=1 ;;
  esac
done
[ -n "$verdict_state" ] || echo "sim_gate: timeout after ${TIMEOUT}s (last state $last)"
cx "pkill -INT -f 'ros2 bag record'" >/dev/null; sleep 4
grep "behavior_tree_node\]" "$OUT/launch.log" | sed 's/\x1b\[[0-9;]*m//g' > "$OUT/mission.log" || true
docker stop -t 5 "$C" >/dev/null 2>&1 || true
# evaluate on a copy of the bag
mkdir -p "$OUT/snap"; rm -f "$OUT"/snap/*.db3; cp "$OUT"/bag/*.db3 "$OUT/snap/"
set +e
docker run --rm --name "${C}-eval" --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
  "${MOUNT[@]}" "$IMG" bash -c \
  "$ROS; python3 scripts/sim_gate/extract.py $COUT/snap $COUT/data.json >/dev/null && \
   python3 scripts/sim_gate/evaluate.py $COUT $CWORLD" | tee "$OUT/verdict.txt"
rc=${PIPESTATUS[0]}
set -e
echo "sim_gate: $NAME exit $rc (output $OUT)"
exit "$rc"
