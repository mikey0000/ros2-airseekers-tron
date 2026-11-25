#!/usr/bin/env bash
# Offline VIO evaluation of one recorded bag (docs/vio.md section 8.4, 2026-11-14). Dev box only,
# in mower:jazzy-dev-amd64 with --network none (no DDS contact with anything else).
#
#   scripts/vio/eval_vio_bag.sh BAG_DIR [OUT_DIR]       # OUT_DIR default BAG_DIR.eval
#
# Needs scripts/vio/build_openvins_local.sh first (ros2_serial_msckf). Steps, outputs in OUT_DIR:
#   1. ov/       OpenVINS over the bag, serially (ros2_serial_msckf, config/vio_metoak): every
#                stereo pair processed, deterministic, faster than real time. odomimu.csv,
#                updates.csv, run.log.
#   2. vio.csv   vio_odom_bridge's conversion + vio_gate on the recorded RTK (eval_vio_bag.py
#                bridge); gate_recorded.json.
#   3. replay/   ekf_replay.py, ekf_map with simulated RTK dropouts (wh_len s every wh_every s,
#                PHASES staggered phases as in the 2026-11-13 drift study): variants v<k> wheel
#                only (vy fused), o<k> the same + ekf_dual_vio.yaml's gated VIO; the gate runs
#                per variant with the dropouts as its RTK loss. --live-inputs when the bag has
#                /odom and /imu/data_aligned (record_vio_bag.sh records both).
#   4. drift.md  drift_analyse.py: error vs distance, distance to p90 0.10 / 0.20 / 0.30 m
#                for wheel only against wheel + VIO, flat / slope / side slope / pivot /
#                reverse / slip classes.
#   5. report.md eval_vio_bag.py report: init, rates, CPU, features, velocity vs RTK and wheels,
#                gate statistics.
#
# Env: SPEED (ekf replay x real time, default 1: 12 filters + 200 Hz VIO in one Python
#      publisher), PHASES (6), WH_LEN (100), WH_EVERY (120), MIN_N (budget table, 15),
#      VIO_HZ (thin the gated VIO, default 0 = every message as live), START_T (bag stamp
#      where OpenVINS starts; default from eval_vio_bag.py start: skips a docked idle part),
#      CONFIG (OpenVINS
#      config dir, default config/vio_metoak), VIO_BUILD (.vio_build),
#      ORACLE=1: skip OpenVINS and make odomimu.csv from RTK + noise (plumbing test on a bag
#      without images; NOT a VIO result).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
BAG="$(cd "${1:?usage: $0 BAG_DIR [OUT_DIR]}" && pwd)"
OUT="${2:-$BAG.eval}"
mkdir -p "$OUT/ov" "$OUT/replay"
OUT="$(cd "$OUT" && pwd)"
VIO_BUILD="${VIO_BUILD:-$STACK_ROOT/.vio_build}"
IMG="${IMG:-mower:jazzy-dev-amd64}"
CONFIG="${CONFIG:-config/vio_metoak}"
SPEED="${SPEED:-1}" PHASES="${PHASES:-6}" WH_LEN="${WH_LEN:-100}" WH_EVERY="${WH_EVERY:-120}"
MIN_N="${MIN_N:-15}" VIO_HZ="${VIO_HZ:-0}" ORACLE="${ORACLE:-0}"

in_image() {
  docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e PYTHONDONTWRITEBYTECODE=1 -v "$STACK_ROOT":/work:ro -v "$VIO_BUILD":/vio:ro \
    -v "$BAG":/bag:ro -v "$OUT":/out -w /out "$IMG" bash -c \
    "source /opt/ros/jazzy/setup.bash; [ -f /vio/install/setup.bash ] && source /vio/install/setup.bash; $*"
}

if [ "$ORACLE" = 1 ]; then
  echo "== 1. ORACLE odomimu (RTK + noise, not OpenVINS)"
  in_image "python3 /work/scripts/vio/eval_vio_bag.py oracle /out --bag /bag"
  report_flags=--oracle
else
  [ -x "$VIO_BUILD/install/ov_msckf/lib/ov_msckf/ros2_serial_msckf" ] || {
    echo "no ros2_serial_msckf in $VIO_BUILD: run scripts/vio/build_openvins_local.sh" >&2; exit 1; }
  START_T="${START_T:-$(in_image "python3 /work/scripts/vio/eval_vio_bag.py start --bag /bag")}"
  echo "== 1. OpenVINS (serial) over $BAG from stamp $START_T"
  in_image "ros2 run ov_msckf ros2_serial_msckf --ros-args \
    -p config_path:=/work/$CONFIG/estimator_config.yaml -p bag:=/bag -p out_dir:=/out/ov \
    -p verbosity:=WARNING -p start_t:=$START_T" 2>&1 | tee "$OUT/ov/run.log" | tail -4
  report_flags=
fi

echo "== 2. bridge + gate on the recorded RTK"
in_image "python3 /work/scripts/vio/eval_vio_bag.py bridge /out --bag /bag"

echo "== 3. ekf_map replay with simulated RTK dropouts (speed x$SPEED)"
live=
if grep -q '/imu/data_aligned' "$BAG/metadata.yaml" && grep -q 'name: /odom$' "$BAG/metadata.yaml"; then
  live=--live-inputs
fi
V=""
for k in $(seq 0 $((PHASES - 1))); do
  ph=$((30 + WH_EVERY * k / PHASES))
  wh="wh_len=$WH_LEN,wh_every=$WH_EVERY,wh_phase=$ph"
  V="$V --variant v$k:vy_var=0.05,fuse_vy=1,$wh --variant o$k:vy_var=0.05,fuse_vy=1,vio=1,vio_hz=$VIO_HZ,$wh"
done
in_image "cd / && python3 -P /work/scripts/replay/ekf_replay.py /bag /out/replay --t0 0 --t1 4e9 \
  --speed $SPEED $live --vio /out/vio.csv $V" > "$OUT/replay.log" 2>&1 || {
  tail -20 "$OUT/replay.log"; exit 1; }

echo "== 4. drift tables"
in_image "python3 /work/scripts/replay/drift_analyse.py /out/replay --min-n $MIN_N \
  --json /out/drift.json" > "$OUT/drift.md"
echo "== 5. VIO health report"
in_image "python3 /work/scripts/vio/eval_vio_bag.py report /out $report_flags" > "$OUT/report.md"
echo "== done: $OUT/report.md, $OUT/drift.md"
sed -n '/^## Budget: distance/,/^$/p' "$OUT/drift.md"
