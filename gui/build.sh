#!/usr/bin/env bash
# Build the MowgliNext GUI image for the Tron port.
#
#   ./gui/build.sh              -> mower-gui:latest  (host arch)
#   ARCH=arm64 ./gui/build.sh   -> mower-gui:arm64   (linux/arm64, for the mower)
#   ARCH=amd64 ./gui/build.sh   -> mower-gui:amd64
#
# Every RUN step executes natively on the build host (Go cross-compiles with
# CGO_ENABLED=0; the final stage has no RUN), so an arm64 build needs no
# QEMU/binfmt. Works with BuildKit (buildx) or the legacy builder; set
# DOCKER_BUILDKIT=0 to force the legacy one. The legacy builder ignores
# gui/Dockerfile.dockerignore and uses the context's own .dockerignore.
#
# Ship to the mower:  docker save mower-gui:arm64 | gzip > mower-gui-arm64.tar.gz
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
CTX="$ROOT/third_party/mowglinext/gui"

UPSTREAM_SHA="$(tr -d '[:space:]' < "$ROOT/third_party/mowglinext/UPSTREAM_COMMIT")"
SHORT_SHA="${UPSTREAM_SHA:0:8}"
BUILD_VERSION="${BUILD_VERSION:-tron-${SHORT_SHA}}"
BUILD_TIME="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

host_arch() {
  case "$(uname -m)" in
    x86_64|amd64) echo amd64 ;;
    aarch64|arm64) echo arm64 ;;
    *) uname -m ;;
  esac
}

if [[ -n "${ARCH:-}" ]]; then
  PLATFORM="linux/${ARCH}"
  TAG="${TAG:-mower-gui:${ARCH}}"
else
  PLATFORM="linux/$(host_arch)"
  TAG="${TAG:-mower-gui:latest}"
fi

BUILD_ARCH="$(host_arch)"
TARGET_ARCH="${PLATFORM#linux/}"

# Use BuildKit only when the buildx plugin is present.
if [[ -z "${DOCKER_BUILDKIT:-}" ]]; then
  if docker buildx version >/dev/null 2>&1; then export DOCKER_BUILDKIT=1; else export DOCKER_BUILDKIT=0; fi
fi

echo ">> building $TAG for $PLATFORM (version $BUILD_VERSION, upstream $UPSTREAM_SHA)"
# Runtime base for the target arch under a per-arch local tag (the legacy
# builder stores one arch per tag; without this an arm64 build followed by an
# amd64 build silently reuses the arm64 base).
RUNTIME_BASE="mower-gui-base:${TARGET_ARCH}"
docker pull -q --platform "$PLATFORM" debian:bookworm-slim >/dev/null
docker tag debian:bookworm-slim "$RUNTIME_BASE"
ARCH_SEEN="$(docker image inspect "$RUNTIME_BASE" --format '{{.Architecture}}')"
[[ "$ARCH_SEEN" == "$TARGET_ARCH" ]] || { echo "runtime base is $ARCH_SEEN, wanted $TARGET_ARCH" >&2; exit 1; }

# BuildKit: --platform selects the target and auto-sets the platform ARGs.
# Legacy builder: --platform would force EVERY stage to the target arch (and
# rejects the build-platform stages), so pass the platforms as build args
# instead; the Dockerfile pins each stage with FROM --platform=...
PLATFORM_FLAG=()
if [[ "$DOCKER_BUILDKIT" == "1" ]]; then PLATFORM_FLAG=(--platform "$PLATFORM"); fi

docker build \
  "${PLATFORM_FLAG[@]}" \
  --build-arg TARGETPLATFORM="$PLATFORM" \
  --build-arg RUNTIME_BASE="$RUNTIME_BASE" \
  --build-arg BUILDPLATFORM="linux/${BUILD_ARCH}" \
  --build-arg TARGETOS=linux \
  --build-arg TARGETARCH="$TARGET_ARCH" \
  -f "$HERE/Dockerfile" \
  --build-arg BUILD_REVISION="$UPSTREAM_SHA" \
  --build-arg BUILD_VERSION="$BUILD_VERSION" \
  --build-arg BUILD_TIME="$BUILD_TIME" \
  -t "$TAG" \
  "$@" \
  "$CTX"
GOT="$(docker image inspect "$TAG" --format '{{.Os}}/{{.Architecture}}')"
[[ "$GOT" == "$PLATFORM" ]] || { echo "built $TAG is $GOT, expected $PLATFORM" >&2; exit 1; }
echo ">> built $TAG ($GOT)"
