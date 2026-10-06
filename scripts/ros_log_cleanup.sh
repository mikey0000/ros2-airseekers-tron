#!/usr/bin/env bash
# Retention for persistent ROS logs and crash artefacts (docs/crash_recovery.md).
#   ROS_LOG_DIR (/userdata/ros2/log): one dir per launch; keep 14 days and <= 500 MB total
#   MOWER_CRASH_DIR (/userdata/ros2/crashes): keep 30 days and <= 2 GB; core files 14 days
#   and the newest 5 only (one core of det_ros_cpp is ~100s of MB).
# Oldest first; the newest ROS log dir (the running launch) is never removed.
# Usage: ros_log_cleanup.sh [--loop SECONDS]   (env: LOG_DAYS LOG_MAX_MB CRASH_DAYS CRASH_MAX_MB)
set -u
LOG_DIR=${ROS_LOG_DIR:-/userdata/ros2/log}
CRASH_DIR=${MOWER_CRASH_DIR:-/userdata/ros2/crashes}
LOG_DAYS=${LOG_DAYS:-14}; LOG_MAX_MB=${LOG_MAX_MB:-500}
CRASH_DAYS=${CRASH_DAYS:-30}; CRASH_MAX_MB=${CRASH_MAX_MB:-2048}
CORE_DAYS=${CORE_DAYS:-14}; CORE_KEEP=${CORE_KEEP:-5}

size_mb() { du -sm "$1" 2>/dev/null | cut -f1; }

# delete oldest entries (by mtime) of $1 matching find args until size <= $2 MB; skip $3
cap_dir() {
  local dir=$1 max=$2 keep=${3:-} f
  while [ "$(size_mb "$dir")" -gt "$max" ]; do
    f=$(find "$dir" -mindepth 1 -maxdepth 1 ! -name latest -printf '%T@ %p\n' | sort -n \
        | awk '{print $2}' | grep -vxF -- "$keep" | head -1)
    [ -n "$f" ] || break
    rm -rf -- "$f"
  done
}

clean_once() {
  if [ -d "$LOG_DIR" ]; then
    local newest
    newest=$(readlink -f "$LOG_DIR/latest" 2>/dev/null || true)
    find "$LOG_DIR" -mindepth 1 -maxdepth 1 ! -name latest -mtime +"$LOG_DAYS" \
         ! -path "$newest" -exec rm -rf -- {} + 2>/dev/null
    cap_dir "$LOG_DIR" "$LOG_MAX_MB" "$newest"
  fi
  if [ -d "$CRASH_DIR" ]; then
    find "$CRASH_DIR" -maxdepth 1 -name 'core.*' -mtime +"$CORE_DAYS" -delete 2>/dev/null
    ls -1t "$CRASH_DIR"/core.* 2>/dev/null | tail -n +$((CORE_KEEP + 1)) | xargs -r rm -f --
    find "$CRASH_DIR" -mindepth 1 -maxdepth 1 -mtime +"$CRASH_DAYS" \
         ! -name '*.jsonl' ! -name '*.faulthandler.log' -exec rm -rf -- {} + 2>/dev/null
    # append-only logs: keep the last 5000 lines
    for f in "$CRASH_DIR"/*.jsonl "$CRASH_DIR"/*.faulthandler.log; do
      [ -f "$f" ] || continue
      [ "$(wc -l < "$f")" -gt 5000 ] && { tail -n 5000 "$f" > "$f.tmp" && mv -f "$f.tmp" "$f"; }
    done
    cap_dir "$CRASH_DIR" "$CRASH_MAX_MB"
  fi
  echo "ros_log_cleanup: log $(size_mb "$LOG_DIR" 2>/dev/null || echo 0) MB," \
       "crashes $(size_mb "$CRASH_DIR" 2>/dev/null || echo 0) MB"
}

if [ "${1:-}" = "--loop" ]; then
  while sleep "${2:-21600}"; do clean_once; done
else
  clean_once
fi
