# Fields2Cover v3 install tree, built ONCE per arch and reused as an image layer.
#
#   ./scripts/build_f2c.sh            # amd64 (dev image)
#   ARCH=arm64 ./scripts/build_f2c.sh # arm64 (mower image, QEMU-emulated)
#
# Result: image `mower-f2c:v3-<arch>` (FROM scratch) containing only
# /opt/fields2cover (lib, include, lib/cmake/Fields2Cover) plus the CPack .deb at
# /opt/fields2cover-deb/. Dockerfile.humble / Dockerfile.dev-amd64 pull it with
# `COPY --from`, so stack image rebuilds never recompile F2C.
#
# NEVER build this on the mower (3.9 GB RAM). Memory: -j2 peaks ~1.5 GB
# (matplot++ / boost.geometry TUs); keep MAKE_JOBS at 2 on a loaded host.
#
# Upstream v3 is not tagged yet: pin the head of the `v3.0` branch
# (project VERSION 3.0.0). OR-Tools is a hard dependency of v3 (no switch to
# disable it); it comes prebuilt from ros-humble-ortools-vendor, so nothing of
# OR-Tools is compiled here. CGAL is not used by F2C at all. Python/SWIG,
# tutorials and tests are disabled.
ARG BASE=ros:humble-ros-base-jammy
FROM ${BASE} AS build
ARG F2C_REPO=https://github.com/Fields2Cover/Fields2Cover.git
ARG F2C_REF=884d895b59192882476e986ba44ea9143a06a6a9
ARG MAKE_JOBS=2
ENV DEBIAN_FRONTEND=noninteractive
RUN (dpkg -s ros-humble-fields2cover >/dev/null 2>&1 && apt-get remove -y ros-humble-fields2cover || true) \
    && apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git ca-certificates file dpkg-dev \
      libgdal-dev libgeos-dev libtbb-dev libtinyxml2-dev libeigen3-dev \
      ros-humble-ortools-vendor \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --filter=blob:none ${F2C_REPO} /src/f2c \
    && git -C /src/f2c checkout ${F2C_REF}
RUN . /opt/ros/humble/setup.sh \
    && cmake -S /src/f2c -B /src/build \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/opt/fields2cover \
         -DCMAKE_INSTALL_LIBDIR=lib \
         -DCMAKE_INSTALL_RPATH=/opt/fields2cover/lib \
         -DBUILD_TUTORIALS=OFF -DBUILD_PYTHON=OFF -DBUILD_TESTING=OFF -DBUILD_DOC=OFF \
         -DUSE_ORTOOLS_VENDOR=ON \
         -DMATPLOTPP_BUILD_EXAMPLES=OFF -DMATPLOTPP_BUILD_TESTS=OFF \
         -DMATPLOTPP_BUILD_INSTALLER=ON \
    && nice -n 15 cmake --build /src/build -j${MAKE_JOBS} \
    && cmake --install /src/build \
    && echo "${F2C_REF}" > /opt/fields2cover/F2C_REF \
    && cd /src/build && (cpack -G DEB -D CPACK_PACKAGING_INSTALL_PREFIX=/opt/fields2cover \
         && mkdir -p /opt/fields2cover-deb && cp *.deb /opt/fields2cover-deb/ \
         || echo "WARNING: cpack .deb failed; install tree is still complete")

FROM scratch
COPY --from=build /opt/fields2cover /opt/fields2cover
COPY --from=build /opt/fields2cover-deb /opt/fields2cover-deb
