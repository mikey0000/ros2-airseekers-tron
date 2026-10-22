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
#              AND field-like localisation drift (mower_sim/loc_drift.py: drifting odom and
#              map -> odom matched to field run 6, seed SEED; -L turns the drift off)
#   -l         localisation drift also without -d (an "ideal drive, real localisation" run)
#   -L         no localisation drift (perfect static map -> odom, the pre-2026-10-16 gate)
#   -s SEED    disturbance and drift seed (default 1)
#   -t SECS    give up after this many SIM seconds after START (default 2700)
#   -x RATE    sim speed: sim time / real time (default $SIM_GATE_RATE or 1; 3 = three times
#              faster). The sim owns /clock, every node runs on sim time and the bag records
#              sim time (docs/simulation.md "Faster than real time"). A run whose controller
#              loop fell below 18 Hz sim is INVALID (exit 4), not FAIL: the host was too slow.
#   -k         keep the sim container running at the end (debugging)
#   -r INDEX   resume scenario: a priming START plans area 0 and is stopped, its resume
#              cursor is rewritten to continue at absolute pose INDEX of area 0 (sub-path 0:
#              the local index; field run 6 resumed at 279 = the pivot vertex (-0.50, 5.32)
#              0.14 m from the boundary, and at 402), the sim restarts docked and START
#              resumes there (scripts/sim_gate/resume_cursor.py; the mission's
#              coverage_resume_path is pointed into the run's maps/)
#   -- ARGS    extra sim.launch.py arguments (e.g. mission_params_file:=/abs/path.yaml)
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK="$(dirname "$SCRIPT_DIR")"
IMG=mower:jazzy-dev-amd64
DOMAIN="${MOWER_SIM_ROS_DOMAIN_ID:-175}"
WORLD="$STACK/src/mower_sim/worlds/real_run5.yaml"
NAME="gate-$(date +%Y%m%d-%H%M%S)"
DISTURB=0; SEED=1; TIMEOUT=2700; KEEP=0; DRIFT=""; RESUME=""; RATE="${SIM_GATE_RATE:-1}"
while getopts "w:n:ds:t:klLr:x:" o; do
  case "$o" in
    w) WORLD="$(cd "$(dirname "$OPTARG")" && pwd)/$(basename "$OPTARG")" ;;
    n) NAME="$OPTARG" ;;
    d) DISTURB=1 ;;
    s) SEED="$OPTARG" ;;
    t) TIMEOUT="$OPTARG" ;;
    k) KEEP=1 ;;
    l) DRIFT=1 ;;
    L) DRIFT=0 ;;
    r) RESUME="$OPTARG" ;;
    x) RATE="$OPTARG" ;;
    *) sed -n '2,34p' "$0"; exit 2 ;;
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
[[ "$RATE" =~ ^[0-9]+(\.[0-9]+)?$ ]] && awk "BEGIN{exit !($RATE >= 0.1 && $RATE <= 10)}" \
  || { echo "sim_gate: -x RATE must be a number in 0.1..10" >&2; exit 2; }
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
[ -n "$DRIFT" ] || DRIFT="$DISTURB"
if [ "$DRIFT" = 1 ]; then
  DARGS="$DARGS loc_drift:=true loc_drift_seed:=$SEED"
fi
if [ -n "$RESUME" ]; then
  [[ "$RESUME" =~ ^[0-9]+$ ]] || { echo "sim_gate: -r needs a pose index" >&2; exit 2; }
  mkdir -p "$OUT/params"
  sed "s|^\( *coverage_resume_path:\).*|\1 $COUT/maps/coverage_resume.txt|" \
    "$STACK/src/mower_mission/config/mission.yaml" > "$OUT/params/mission_resume.yaml"
  grep -q "coverage_resume_path: $COUT/maps/coverage_resume.txt" "$OUT/params/mission_resume.yaml" \
    || { echo "sim_gate: could not set coverage_resume_path" >&2; exit 2; }
  EXTRA="$EXTRA mission_params_file:=$COUT/params/mission_resume.yaml"
fi
ROS="source /opt/ros/jazzy/setup.bash; source install/setup.bash"
cx() { docker exec "$C" bash -c "$ROS; $*" 2>&1 | grep -v "localhost_only\|ROS_LOCALHOST_ONLY" || true; }
state() { cx "timeout 8 ros2 topic echo --once --field state_name /behavior_tree_node/high_level_status" \
  | grep -v '^---' | head -1; }
# sim time (s since epoch, integer) from /clock
simsec() { cx "timeout 8 ros2 topic echo --once --field clock /clock" | awk '/^sec:/{s=$2} /^nanosec:/{n=$2} END{if (s) printf "%d.%09d\n", s, n}'; }
cleanup() {
  if [ "$KEEP" != 1 ]; then docker stop -t 5 "$C" >/dev/null 2>&1 || true; fi
}
trap cleanup EXIT

{
  echo "world=$WORLD"; echo "domain=$DOMAIN"; echo "disturb=$DISTURB drift=$DRIFT seed=$SEED args=$DARGS"
  echo "extra=$EXTRA"; echo "rate=$RATE"; echo "commit=$(git -C "$STACK" rev-parse --short HEAD 2>/dev/null)"
  echo "dirty=$(git -C "$STACK" status --porcelain 2>/dev/null | grep -c . || true)"
} > "$OUT/run.txt"
echo "sim_gate: run $NAME -> $OUT (container $C, domain $DOMAIN, rate x$RATE, disturbance $DISTURB, drift $DRIFT${RESUME:+, resume at pose $RESUME})"
launch_sim() {
  docker rm -f "$C" >/dev/null 2>&1 || true
  docker run -d --rm --name "$C" --network host "${ENV[@]}" "${MOUNT[@]}" "$IMG" bash -c \
    "$ROS; ros2 launch mower_bringup sim.launch.py world:=$CWORLD localization:=sim \
     maps_dir:=$COUT/maps rate_factor:=$RATE $DARGS $EXTRA > $COUT/$1 2>&1" >/dev/null
  # ready: the mission is up and idle on the dock
  ready=""; idle=""
  for _ in $(seq 1 30); do
    sleep 5
    s="$(state)" || s=""
    case "$s" in CHARGING|IDLE_DOCKED) ready="$s"; break ;; IDLE) idle="$s" ;; esac
  done
  [ -n "$ready" ] || ready="$idle"
  [ -n "$ready" ] || { echo "sim_gate: the mission did not come up (see $OUT/$1)" >&2; exit 3; }
  sleep 10
}
if [ -n "$RESUME" ]; then
  # priming START: plan area 0 (the mission persists the plan's cursor), then stop the sim
  rm -f "$OUT/maps/coverage_resume.txt"
  launch_sim launch_prime.log
  cx "ros2 service call /behavior_tree_node/high_level_control mowgli_interfaces/srv/HighLevelControl '{command: 1}'" >/dev/null
  primed=""
  for _ in $(seq 1 60); do
    sleep 2
    if python3 "$SCRIPT_DIR/sim_gate/resume_cursor.py" "$OUT/maps/coverage_resume.txt" 0 \
        "$RESUME" > "$OUT/resume.txt" 2>&1; then primed=1; break; fi
    grep -q "outside the plan" "$OUT/resume.txt" && break
  done
  docker stop -t 5 "$C" >/dev/null 2>&1 || true
  sleep 3
  [ -n "$primed" ] || { echo "sim_gate: priming START wrote no plan cursor ($(cat "$OUT/resume.txt"))" >&2; exit 3; }
  # the stop may have rewritten the cursor (interrupt): set the resume point again
  python3 "$SCRIPT_DIR/sim_gate/resume_cursor.py" "$OUT/maps/coverage_resume.txt" 0 "$RESUME" \
    > "$OUT/resume.txt" 2>&1 || true
  cat "$OUT/resume.txt"
  echo "resume=$RESUME" >> "$OUT/run.txt"
fi
launch_sim launch.log
docker exec -d "$C" bash -c "$ROS; cd $COUT && exec ros2 bag record -s sqlite3 -o bag --use-sim-time \
  --include-hidden-topics --max-cache-size 0 \
  /sim/ground_truth /sim/status /behavior_tree_node/high_level_status \
  /behavior_tree_node/mow_plan /behavior_tree_node/mow_progress /cmd_vel /cmd_vel_nav \
  /cmd_vel_nav_raw \
  /behavior_tree_node/conditioned_route \
  /cmd_vel_emergency /cmd_vel_docking /cmd_vel_slew/shape_status /map_server_node/outside_m \
  /odometry/filtered_map /odometry/filtered /mission/incident /plan /rosout /navigate_to_pose/_action/status \
  /obstacle_tracker/nogo_zones /obstacle_tracker/obstacles \
  /follow_path/_action/status /spin/_action/status /mower_docking/dock/_action/status \
  > $COUT/rec.log 2>&1"
sleep 5
# START time on the sim clock (the bag and evaluate.py are in sim time)
t_start="$(simsec)"
[ -n "$t_start" ] || { echo "sim_gate: no /clock from the sim" >&2; exit 3; }
echo "$t_start" > "$OUT/start_epoch"
cx "ros2 service call /behavior_tree_node/high_level_control mowgli_interfaces/srv/HighLevelControl '{command: 1}'" | tail -1
echo "sim_gate: START sent ($ready)"

# -t is sim seconds; the wall cap only guards against a stalled sim clock
end_sim=$(awk "BEGIN{printf \"%d\", ${t_start%.*} + $TIMEOUT}")
end_wall=$(( $(date +%s) + $(awk "BEGIN{printf \"%d\", 2 * $TIMEOUT / $RATE + 120}") ))
poll=$(awk "BEGIN{p = 15 / $RATE; printf \"%d\", (p < 5 ? 5 : p)}")
last=""; active=0; verdict_state=""; now_sim="${t_start%.*}"
while [ "$(date +%s)" -lt "$end_wall" ] && [ "$now_sim" -lt "$end_sim" ]; do
  sleep "$poll"
  t="$(simsec)"; [ -n "$t" ] && now_sim="${t%.*}"
  s="$(state)" || s=""          # a busy / restarting mission node: try again
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
[ -n "$verdict_state" ] || echo "sim_gate: timeout after $(( now_sim - ${t_start%.*} )) s sim (limit ${TIMEOUT}s, last state $last)"
cx "pkill -INT -f 'ros2 bag record'" >/dev/null; sleep 4
grep "behavior_tree_node\]" "$OUT/launch.log" | sed 's/\x1b\[[0-9;]*m//g' > "$OUT/mission.log" || true
docker stop -t 5 "$C" >/dev/null 2>&1 || true
# evaluate the bag in place: the recorder and its container are stopped above, so the db3
# is final (the copy extract.py asks for only matters for a LIVE bag; it doubled the disk use)
set +e
docker run --rm --name "${C}-eval" --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e SIM_GATE_MEASURE="${SIM_GATE_MEASURE:-body}" \
  "${MOUNT[@]}" "$IMG" bash -c \
  "$ROS; python3 scripts/sim_gate/extract.py $COUT/bag $COUT/data.json >/dev/null && \
   python3 scripts/sim_gate/evaluate.py $COUT $CWORLD" | tee "$OUT/verdict.txt"
rc=${PIPESTATUS[0]}
set -e
echo "sim_gate: $NAME exit $rc (output $OUT)"
exit "$rc"
