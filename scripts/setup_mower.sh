#!/usr/bin/env bash
# setup_mower.sh — Idempotent mower-side setup
# Detects OS (expect Ubuntu 20.04 aarch64), checks free space,
# installs docker.io if missing, adds user to docker group,
# notes iptables/nft caveat, and verifies Docker is functional
# via 'docker info' (docker run hello-world does not work
# off-image on this host).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"

# --- OS detection ---
if [ -f /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  OS_NAME="${ID:-}"
  OS_VERSION="${VERSION_ID:-}"
  OS_ARCH="${MACHINE:-$(uname -m)}"
else
  echo "WARNING: /etc/os-release not found; skipping OS check."
  OS_NAME=""
  OS_VERSION=""
  OS_ARCH=""
fi

echo "OS detection: ID=$OS_NAME  VERSION=$OS_VERSION  ARCH=$OS_ARCH"

if [ "$OS_NAME" != "ubuntu" ] || [ "$OS_VERSION" != "20.04" ]; then
  echo "WARNING: This script expects Ubuntu 20.04 (focal). Detected: $OS_NAME $OS_VERSION"
fi

if [ "$OS_ARCH" != "aarch64" ] && [ "$OS_ARCH" != "arm64" ]; then
  echo "WARNING: This script expects aarch64/arm64. Detected: $OS_ARCH"
fi

# --- Free disk space check (root filesystem) ---
echo "Checking free disk space on '/' ..."
ROOT_FREE=$(df -B1 / | awk 'NR==2 {print $4}')
ROOT_TOTAL=$(df -B1 / | awk 'NR==2 {print $2}')
ROOT_USED=$((ROOT_TOTAL - ROOT_FREE))
ROOT_GB=$((ROOT_USED / 1000000000))

echo "  Used: ${ROOT_GB} GB, Free: $((ROOT_FREE / 1000000000)) GB"

if [ "$ROOT_FREE" -lt 10000000000 ]; then
  # Less than 10 GB free
  echo "WARNING: Less than 10 GB free on '/'. Current free: $((ROOT_FREE / 1000000000)) GB"
fi

# --- Install docker.io if missing ---
DOCKER_PKG="docker.io"
if dpkg -l | grep -q "^ii.*${DOCKER_PKG}" 2>/dev/null; then
  echo "docker.io is already installed."
else
  echo "Installing docker.io via apt ..."
  # sudo guard: this script should be run as root or under sudo
  if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Please run as root or with sudo to install packages."
    exit 1
  fi
  apt-get update
  apt-get install -y --no-install-recommends "$DOCKER_PKG"
fi

# --- Add current user to docker group ---
CURRENT_USER=$(getent passwd "$SUDO_USER" | cut -d: -f1 || getent passwd "$USER" | cut -d: -f1)
if [ -n "$CURRENT_USER" ] && [ "$CURRENT_USER" != "root" ]; then
  if getent group docker | grep -q "^docker:"; then
    echo "Adding user '$CURRENT_USER' to docker group ..."
    usermod -aG docker "$CURRENT_USER"
    echo "NOTE: New docker group membership requires a re-login or a fresh sudo session to take effect."
  else
    echo "WARNING: docker group does not exist; skipping usermod."
  fi
else
  echo "Skipping user group addition (current user is root or unknown)."
fi

# --- iptables / nft caveat ---
echo "CAVEAT: Docker configures iptables by default. If you use nftables or a custom"
echo "  iptables setup, you may need to adjust rules after Docker starts. Docker"
echo "  may clash with existing firewall rules. See: https://docs.docker.com/engine/networking/"

# --- Optionally pre-pull ros:humble-ros-base-jammy ---
# Set PRE_PULL=1 to force pull, or leave unset/empty for no pull.
if [ "${PRE_PULL:-}" = "1" ]; then
  echo "Pre-pulling ros:humble-ros-base-jammy (linux/arm64) ..."
  docker pull --platform linux/arm64 ros:humble-ros-base-jammy 2>&1 || {
    echo "WARN: Pre-pull failed; you may pull manually later: docker pull ros:humble-ros-base-jammy"
  }
else
  echo "Skipping pre-pull of ros:humble-ros-base-jammy. Set PRE_PULL=1 to force."
fi

echo "setup_mower.sh complete."
echo "  Run 'docker info' to verify Docker is functional (off-image verification)."
echo "  After re-login, test: docker run --rm hello-world"