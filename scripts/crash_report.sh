#!/usr/bin/env bash
# Bundle the newest crash evidence into one tar for upload (docs/crash_recovery.md).
# Run on the mower HOST (docker access: adds the container log) or inside mower_humble.
#
#   scripts/crash_report.sh [CRASH_DIR_ENTRY] [-o OUT.tar.gz]
#
# Contents: the newest black-box dump dir (or the one given: bag/, reason.json,
# launch_log_tail.txt, copied crash records), events.jsonl + process_exits.jsonl, Python
# crash records + faulthandler logs of the last 2 days, core file *names* (cores are big:
# copy one explicitly if needed), the latest ROS log dir (launch.log + per-node rosout
# logs, files > 20 MB truncated to their tail), versions (DEPLOYED_REV / git HEAD, image id),
# and `docker logs --tail 500 mower_humble` when docker is reachable.
set -u
CRASH_DIR=${MOWER_CRASH_DIR:-/userdata/ros2/crashes}
LOG_DIR=${ROS_LOG_DIR:-/userdata/ros2/log}
STACK_DIR=${MOWER_STACK_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
CONTAINER=${CONTAINER:-mower_humble}
OUT=""; PICK=""
while [ $# -gt 0 ]; do
  case "$1" in
    -o) OUT=$2; shift 2;;
    -h|--help) sed -n 2,14p "$0"; exit 0;;
    *) PICK=$1; shift;;
  esac
done
ts=$(date +%Y%m%d-%H%M%S)
OUT=${OUT:-$CRASH_DIR/crash_report_$ts.tar.gz}
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
dst=$work/crash_report_$ts; mkdir -p "$dst"

if [ -n "$PICK" ]; then dump=$CRASH_DIR/${PICK#"$CRASH_DIR"/}
else dump=$(ls -1dt "$CRASH_DIR"/2*_*/ 2>/dev/null | head -1); dump=${dump%/}; fi
if [ -n "$dump" ] && [ -d "$dump" ]; then cp -a "$dump" "$dst/"; echo "dump: $dump"
else echo "no black-box dump dir found in $CRASH_DIR"; fi

for f in events.jsonl process_exits.jsonl; do
  [ -f "$CRASH_DIR/$f" ] && tail -n 2000 "$CRASH_DIR/$f" > "$dst/$f"
done
mkdir -p "$dst/records"
find "$CRASH_DIR" -maxdepth 1 \( -name '*.txt' -o -name '*.faulthandler.log' \) -mtime -2 \
     -exec cp -a {} "$dst/records/" \; 2>/dev/null
ls -l "$CRASH_DIR"/core.* > "$dst/core_files.txt" 2>/dev/null || echo "none" > "$dst/core_files.txt"

latest=$(readlink -f "$LOG_DIR/latest" 2>/dev/null || true)
if [ -n "$latest" ] && [ -d "$latest" ]; then
  mkdir -p "$dst/ros_log"
  for f in "$latest"/*; do
    [ -f "$f" ] || continue
    if [ "$(stat -c %s "$f")" -gt 20971520 ]; then tail -c 20971520 "$f" > "$dst/ros_log/${f##*/}"
    else cp -a "$f" "$dst/ros_log/"; fi
  done
fi

{
  echo "report: $ts"; echo "host: $(hostname)"; echo "kernel: $(uname -r)"
  echo "core_pattern: $(cat /proc/sys/kernel/core_pattern)"
  if git -C "$STACK_DIR" rev-parse HEAD >/dev/null 2>&1; then
    echo "git: $(git -C "$STACK_DIR" rev-parse HEAD) $(git -C "$STACK_DIR" status --porcelain | wc -l) dirty"
  fi
  [ -f "$STACK_DIR/DEPLOYED_REV" ] && echo "deployed: $(cat "$STACK_DIR/DEPLOYED_REV")"
  if command -v docker >/dev/null && docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "image: $(docker inspect -f '{{.Image}}' "$CONTAINER")"
    echo "container_started: $(docker inspect -f '{{.State.StartedAt}}' "$CONTAINER")"
    echo "restart_count: $(docker inspect -f '{{.RestartCount}}' "$CONTAINER")"
  fi
} > "$dst/versions.txt"
if command -v docker >/dev/null && docker inspect "$CONTAINER" >/dev/null 2>&1; then
  docker logs --tail 500 "$CONTAINER" > "$dst/container_log_tail.txt" 2>&1
fi

mkdir -p "$(dirname "$OUT")"
tar -czf "$OUT" -C "$work" "crash_report_$ts"
echo "wrote $OUT ($(du -h "$OUT" | cut -f1))"
