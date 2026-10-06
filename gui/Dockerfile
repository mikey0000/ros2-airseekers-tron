#
# MowgliNext web GUI, slimmed for the Airseekers Tron ROS 2 Humble port.
#
# Build context MUST be third_party/mowglinext/gui (see gui/build.sh):
#   docker build -f gui/Dockerfile -t mower-gui:latest third_party/mowglinext/gui
#
# Differences from upstream third_party/mowglinext/gui/Dockerfile:
#   * no openocd / platformio / python / docker CLI (no firmware flashing,
#     no docker-driven tools on Tron)
#   * every RUN executes on the BUILD platform: Go cross-compiles with
#     CGO_ENABLED=0, the SPA is arch-independent, and ca-certificates/tzdata
#     are installed in a build-platform ubuntu:22.04 stage and copied in. The
#     target-arch final stage (debian:bookworm-slim) has no RUN, so `--platform linux/arm64` needs no
#     QEMU/binfmt.
#   * works with BuildKit and with the legacy builder (this host has no
#     buildx): no RUN --mount, and gui/build.sh passes BUILDPLATFORM /
#     TARGETOS / TARGETARCH explicitly because the legacy builder does not
#     populate them.
#
# Licence: GPLv3 (third_party/mowglinext/LICENSE). Distributing this image is
# distributing a derivative work -- ship/offer the corresponding source.

ARG BUILDPLATFORM=linux/amd64
ARG TARGETPLATFORM=linux/amd64
# Runtime base. gui/build.sh pulls debian:bookworm-slim for the target arch and
# passes a per-arch local tag here, because the legacy (non-buildx) builder keeps
# only one arch per local tag and would otherwise reuse the wrong one.
ARG RUNTIME_BASE=debian:bookworm-slim
ARG BUILD_REVISION=""
ARG BUILD_VERSION="tron-dev"
ARG BUILD_TIME=""

# ---------------------------------------------------------------- Go backend
FROM --platform=$BUILDPLATFORM golang:1.25 AS build-go
ARG TARGETOS
ARG TARGETARCH
ARG BUILD_REVISION
ARG BUILD_VERSION
ARG BUILD_TIME
WORKDIR /src
COPY go.mod go.sum ./
RUN go mod download
COPY . .
RUN CGO_ENABLED=0 GOOS=${TARGETOS:-linux} GOARCH=${TARGETARCH:-amd64} go build -buildvcs=false \
      -ldflags "-X github.com/mowglinext/mowglinext/pkg/buildinfo.Revision=${BUILD_REVISION} -X github.com/mowglinext/mowglinext/pkg/buildinfo.Version=${BUILD_VERSION} -X github.com/mowglinext/mowglinext/pkg/buildinfo.BuiltAt=${BUILD_TIME}" \
      -o /out/mowglinext

# ---------------------------------------------------------------- React SPA
FROM --platform=$BUILDPLATFORM node:22 AS build-web
ARG BUILD_REVISION
ARG BUILD_VERSION
ARG BUILD_TIME
# The Tron UI trim is runtime (ROBOT_PROFILE / mower_model, see web/src/constants/profileGates.ts).
# Build-time only: skip the first-run redirect to the onboarding wizard (read by AppShell.tsx).
ARG VITE_SKIP_ONBOARDING=1
WORKDIR /web
COPY web/package.json web/yarn.lock web/.npmrc ./
RUN yarn install --frozen-lockfile --network-timeout 600000
COPY web/ ./
# `yarn build` = tsc && vite build (no lint / tests)
RUN VITE_BUILD_REVISION="$BUILD_REVISION" VITE_BUILD_VERSION="$BUILD_VERSION" VITE_BUILD_TIME="$BUILD_TIME" \
    VITE_SKIP_ONBOARDING="$VITE_SKIP_ONBOARDING" \
    yarn build \
 && find dist/assets -type f \( -name '*.js' -o -name '*.css' -o -name '*.json' -o -name '*.svg' -o -name '*.wasm' \) \
      -size +1024c -exec gzip -9 -k {} +

# ------------------------------------------- ca-certificates + tzdata (native)
FROM --platform=$BUILDPLATFORM ubuntu:22.04 AS sysfiles
RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ca-certificates tzdata \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------- runtime
# The only target-arch stage: COPY only, no RUN -> nothing executes for the
# target arch (no QEMU, nothing heavy for a 4 GB arm64 box to do). Its base is a
# different image from the build-platform sysfiles stage (ubuntu:22.04) so
# the two arches never share a local tag.
FROM --platform=$TARGETPLATFORM ${RUNTIME_BASE}
ARG BUILD_VERSION
# Arch-independent data files only (PEM bundle + zoneinfo).
COPY --from=sysfiles /etc/ssl/certs /etc/ssl/certs
COPY --from=sysfiles /usr/share/ca-certificates /usr/share/ca-certificates
COPY --from=sysfiles /usr/share/zoneinfo /usr/share/zoneinfo
LABEL org.opencontainers.image.title="mower-gui" \
      org.opencontainers.image.version="${BUILD_VERSION}" \
      org.opencontainers.image.licenses="GPL-3.0-only" \
      org.opencontainers.image.description="MowgliNext GUI (GPLv3) packaged for the Airseekers Tron ROS 2 port"
COPY --from=build-go /out/mowglinext /app/mowglinext
COPY --from=build-web /web/dist /app/web
# getSchema() opens asserts/mower_config.schema.json relative to the CWD
COPY asserts /app/asserts
WORKDIR /app
ENV WEB_DIR=/app/web \
    API_ADDR=:4006 \
    DB_PATH=/db
EXPOSE 4006
ENTRYPOINT ["/app/mowglinext"]
