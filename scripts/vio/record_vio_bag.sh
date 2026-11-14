#!/usr/bin/env bash
# Record the bag for the offline VIO evaluation (docs/vio.md section 8, 2026-11-14).
# Run ON THE MOWER HOST as root, with the stack up (mower_jazzy running). Changes nothing on
# the robot: no node is started or reconfigured, VIO stays off; it only adds one recorder.
#
#   sudo /userdata/ros2_stack/scripts/vio/record_vio_bag.sh check            # preflight only
#   sudo /userdata/ros2_stack/scripts/vio/record_vio_bag.sh mow   [--minutes 20]
#   sudo /userdata/ros2_stack/scripts/vio/record_vio_bag.sh drive [--minutes 8]
#
# mow:   start it, then start a normal mow from the GUI (slope, pivots, reverse legs, RTK truth).
# drive: a manual drive with the remote (docs/vio.md 8.2).
# Stops cleanly on Ctrl-C or after --minutes; prints where the bag is and how to copy it.
#
# Preflight (refuses to record if a required check fails, and says what to do):
#   host:      mower_jazzy running; net.core.rmem_max >= 8 MiB and the Metoak IMU in IIO mode
#              (both from scripts/setup_stereo_host.sh, runtime-only: once per boot); load;
#              free space on /userdata for --minutes plus bag_recorder's 3 GB floor + 1 GB;
#   container: scripts/vio/vio_preflight.py: exact stereo pairs >= 9 Hz, /stereo_imu/data
#              >= 190 Hz, /odom, /fix alive, RTK FIXED now.
#
# Storage choice (RK3588, 4 GB, no swap; load-190 incident: the stack comes first):
#   * ~6.4 MB/s, ~380 MB/min: two mono8 640x480 eyes at 10 Hz = 6.1 MB/s, the rest 0.25 MB/s.
#   * sqlite3, no compression. mcap is not in the mower image (bag_recorder.py) and the
#     evaluation tools (ekf_replay.py) read .db3 directly. zstd buys little on sensor-noise
#     grass images (~1.3x) for a sustained share of a core while the stack is mowing.
#   * 60 s splits (~380 MB each): a crash or power cut loses at most the open split.
#   * 16 MiB write cache instead of rosbag2's 100 MiB double buffer (200 MB of RAM).
#   * nice 10 + best-effort I/O class 7: under CPU or eMMC contention the recorder drops
#     messages (counted in the log), the stack does not starve.
#   * The bag goes to /userdata/ros2/vio_bags/ (not the bag_recorder ring, which would prune it).
set -uo pipefail
C=mower_jazzy
OUT_ROOT=/userdata/ros2/vio_bags
STACK_IN_C=/work
TOPICS=(
  /vio/left/image_raw /vio/right/image_raw /vio/left/camera_info /vio/right/camera_info
  /stereo_imu/data /imu/data /imu/data_aligned
  /odom /mcu/measured_speed /cmd_vel
  /fix /fix_status /gps/status /odometry/gps /odometry/filtered /odometry/filtered_map
  /tilt/status /heading_aligner/status /localization/status
  /behavior_tree_node/high_level_status /hardware_bridge/status
  /tf /tf_static
)

mode="${1:-}"; shift || true
case "$mode" in
  mow) minutes=20 ;;
  drive) minutes=8 ;;
  check) minutes=20 ;;
  *) sed -n 2,13p "$0"; exit 2 ;;
esac
while [ $# -gt 0 ]; do
  case "$1" in
    --minutes) minutes="$2"; shift 2 ;;
    *) echo "unknown argument $1" >&2; exit 2 ;;
  esac
done
[[ "$minutes" =~ ^[0-9]+$ ]] && [ "$minutes" -ge 1 ] || { echo "--minutes takes whole minutes" >&2; exit 2; }

in_c() { docker exec "$C" bash -c "source /opt/ros/jazzy/setup.bash; $*"; }
fail() { echo "  FAIL $1"; [ -n "${2:-}" ] && echo "       -> $2"; bad=1; }

echo "== VIO recording preflight ($mode, $minutes min)"
bad=0
if ! docker ps -q -f "name=^${C}\$" -f status=running | grep -q .; then
  echo "  FAIL $C is not running -> start the stack first (deploy_to_mower.sh up)"; exit 1
fi
rmem=$(sysctl -n net.core.rmem_max)
if [ "$rmem" -ge 8388608 ]; then echo "  ok   socket buffers  rmem_max $rmem"
else fail "socket buffers  rmem_max $rmem < 8388608" \
  "sudo /userdata/ros2_stack/scripts/setup_stereo_host.sh, then docker restart $C"; fi
if grep -qs icm40608-gyro /sys/bus/iio/devices/iio:device*/name; then
  echo "  ok   stereo IMU      IIO mode"
else fail "stereo IMU      not in IIO mode" \
  "sudo /userdata/ros2_stack/scripts/setup_stereo_host.sh, then docker restart $C"; fi
load=$(cut -d' ' -f1 /proc/loadavg)
if awk -v l="$load" 'BEGIN{exit !(l < 10)}'; then echo "  ok   load            $load"
else fail "load            $load >= 10" "wait for the load to settle (top), then retry"; fi
free_gb=$(df -P -k /userdata | awk 'NR==2 {printf "%.1f", $4/1048576}')
[ "$bad" = 0 ] || { echo "== preflight FAILED: nothing recorded"; exit 1; }
if ! in_c "python3 $STACK_IN_C/scripts/vio/vio_preflight.py --seconds 6 --minutes $minutes --free-gb $free_gb"; then
  echo "== preflight FAILED: nothing recorded"; exit 1
fi
[ "$mode" = check ] && { echo "== preflight passed"; exit 0; }

name="$(date +%Y%m%d-%H%M%S)_$mode"
bag="$OUT_ROOT/$name"
mkdir -p "$OUT_ROOT"
echo "== recording $bag for up to $minutes min (Ctrl-C stops it cleanly)"
docker exec -d "$C" bash -c "source /opt/ros/jazzy/setup.bash; echo \$\$ > $bag.pid; \
  exec nice -n 10 ionice -c2 -n7 ros2 bag record -o $bag --storage sqlite3 \
    --max-bag-duration 60 --max-cache-size 16777216 --compression-mode none \
    --topics ${TOPICS[*]} > $bag.log 2>&1"
sleep 3
pid=$(cat "$bag.pid" 2>/dev/null || true)
alive() { [ -n "$pid" ] && docker exec "$C" kill -0 "$pid" 2>/dev/null; }
alive || { echo "recorder did not start:"; tail -20 "$bag.log"; exit 1; }
case "$mode" in
  mow)   echo "   now start the mow from the GUI; keep this terminal open." ;;
  drive) echo "   now drive with the remote (docs/vio.md 8.2); keep this terminal open." ;;
esac

stop=0
trap 'stop=1' INT TERM
start=$SECONDS deadline=$((SECONDS + minutes * 60)) next=$((SECONDS + 30))
while [ "$stop" = 0 ] && [ $SECONDS -lt $deadline ]; do
  sleep 2
  if [ $SECONDS -ge $next ]; then
    alive || { echo "recorder exited early:"; tail -20 "$bag.log"; break; }
    printf '   %3d s  %s\n' $((SECONDS - start)) "$(du -sh "$bag" 2>/dev/null | cut -f1)"
    next=$((SECONDS + 30))
  fi
done
trap - INT TERM
if alive; then
  echo "== stopping the recorder"
  docker exec "$C" kill -INT "$pid"
  for _ in $(seq 30); do alive || break; sleep 1; done
  alive && { echo "   still running after 30 s: SIGKILL (the open split may be lost)";
             docker exec "$C" kill -KILL "$pid"; }
fi
rm -f "$bag.pid"
echo "== bag: $bag ($(du -sh "$bag" | cut -f1), $((SECONDS - start)) s)"
in_c "ros2 bag info $bag 2>/dev/null | sed -n '1,8p'"
grep -iE "drop|lost|error" "$bag.log" | tail -5
cat <<EOF
== copy it to the dev box (from the stack checkout there; same login as deploy_to_mower.sh):
   rsync -a --info=progress2 root@<mower>:$bag ../bags/vio/
   then: scripts/vio/eval_vio_bag.sh ../bags/vio/$name   (docs/vio.md section 8.4)
   delete it on the mower once copied: sudo rm -r $bag $bag.log
EOF
