#!/usr/bin/env bash
# Build the Fields2Cover v3 install-tree image ONCE per arch (dev host only).
#   ./scripts/build_f2c.sh              -> mower-f2c:v3-amd64
#   ARCH=arm64 ./scripts/build_f2c.sh   -> mower-f2c:v3-arm64 (needs QEMU binfmt
#                                          for aarch64 on the host, see below)
# NEVER run on the mower (3.9 GB RAM). Defaults: -j2 under nice 15; measured
# native amd64: ~2 min wall, ~0.7 GB peak build-container RSS.
# arm64 on an x86 host: RUN steps execute under qemu-user, so register it once
#   docker run --privileged --rm tonistiigi/binfmt --install arm64
# The base is pulled by per-arch digest so the shared ros:jazzy-ros-base-noble
# tag is never re-pointed.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"
ARCH="${ARCH:-amd64}"
MAKE_JOBS="${MAKE_JOBS:-2}"
TAG="${TAG:-mower-f2c:v3-${ARCH}}"
if [[ -z "${BASE:-}" ]]; then
  BASE="ros@$(docker manifest inspect ros:jazzy-ros-base-noble | python3 -c "
import json,sys; d=json.load(sys.stdin)
print(next(m['digest'] for m in d['manifests'] if m['platform'].get('os')=='linux' and m['platform']['architecture']=='${ARCH}'))")"
fi
free -g | sed -n 1,2p
echo ">> building $TAG from $BASE with -j${MAKE_JOBS}"
DOCKER_BUILDKIT=0 nice -n 15 docker build --build-arg BASE="$BASE" --build-arg MAKE_JOBS="$MAKE_JOBS" \
  -t "$TAG" -f "$STACK_ROOT/docker/fields2cover/Dockerfile" "$STACK_ROOT/docker/fields2cover"
