#!/usr/bin/env bash
# Remove Fast DDS shared-memory leftovers in /dev/shm that no live process maps.
# Fast DDS 2.6 (Humble) leaks fastrtps_* segments/port files when a participant is killed
# (docker restart, SIGKILL, crash). /dev/shm is the HOST tmpfs (bind-mounted /dev) and each
# run leaves ~145 MB (4 camera publishers x 32 MiB + ~30 x 512 KiB), so ~7 restarts fill 1 GB.
# Only base segment/port files are candidates (never *_el locks or sem.* of live ports).
# Safe: a file is removed only if (a) no /proc/*/maps (this PID namespace) references it and
# (b) it is older than MIN_AGE seconds (default 60, covers participants still creating it).
# Usage: shm_gc.sh [--loop SECONDS]     (env: MIN_AGE, SHM_DIR)
set -u
SHM_DIR=${SHM_DIR:-/dev/shm}
MIN_AGE=${MIN_AGE:-60}

gc_once() {
  local mapped now f m freed=0 n=0
  mapped=$(cat /proc/[0-9]*/maps 2>/dev/null | awk '/fastrtps/ {print $6}' | sort -u)
  now=$(date +%s)
  for f in "$SHM_DIR"/fastrtps_*; do
    case "$f" in *_el) continue;; esac   # lock files: removed with their base file
    [ -e "$f" ] || continue
    m=$(stat -c %Y "$f" 2>/dev/null) || continue
    [ $((now - m)) -ge "$MIN_AGE" ] || continue
    grep -qxF "$f" <<<"$mapped" && continue
    # also skip files held open (fd) by any process
    ls -l /proc/[0-9]*/fd 2>/dev/null | grep -qF -- "-> $f" && continue
    freed=$((freed + $(stat -c %s "$f"))); n=$((n + 1))
    rm -f -- "$f" "${f}_el" "$SHM_DIR/sem.${f##*/}_mutex"
  done
  echo "shm_gc: removed $n stale file(s), $((freed / 1048576)) MiB"
}

if [ "${1:-}" = "--loop" ]; then
  while true; do gc_once; sleep "${2:-300}"; done
else
  gc_once
fi
