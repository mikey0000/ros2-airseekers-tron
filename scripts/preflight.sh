#!/usr/bin/env bash
# preflight.sh — Mower preflight checks before bringing up the ROS 2 stack
# Checks: docker present, serial device nodes, vendor services holding ports,
# free disk on /userdata, and prints go/no-go for bringing up the ROS 2 stack.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=== Mower preflight checks ==="

# --- Docker present ---
if ! command -v docker &>/dev/null; then
  echo "FAIL: docker command not found. Install docker.io first (see setup_mower.sh)."
  DOCKER_OK="no"
else
  echo "PASS: docker is available."
  DOCKER_OK="yes"
fi

# --- Serial device node symlinks / nodes ---
echo "Checking serial device nodes ..."

# Expected: /dev/serial_mower -> ttyS9, /dev/serial_imu -> ttyS1, /dev/serial_rtk -> ttyS4
if [ -L /dev/serial_mower ]; then
  TARGET=$(readlink /dev/serial_mower)
  if [ "$TARGET" = "ttyS9" ]; then
    echo "PASS: /dev/serial_mower -> $TARGET"
  else
    echo "FAIL: /dev/serial_mower points to $TARGET (expected ttyS9)"
  fi
else
  echo "FAIL: /dev/serial_mower does not exist or is not a symlink"
fi

if [ -L /dev/serial_imu ]; then
  TARGET=$(readlink /dev/serial_imu)
  if [ "$TARGET" = "ttyS1" ]; then
    echo "PASS: /dev/serial_imu -> $TARGET"
  else
    echo "FAIL: /dev/serial_imu points to $TARGET (expected ttyS1)"
  fi
else
  echo "FAIL: /dev/serial_imu does not exist or is not a symlink"
fi

if [ -L /dev/serial_rtk ]; then
  TARGET=$(readlink /dev/serial_rtk)
  if [ "$TARGET" = "ttyS4" ]; then
    echo "PASS: /dev/serial_rtk -> $TARGET"
  else
    echo "FAIL: /dev/serial_rtk points to $TARGET (expected ttyS4)"
  fi
else
  echo "FAIL: /dev/serial_rtk does not exist or is not a symlink"
fi

# --- Vendor services holding the ports ---
echo "Checking vendor services (mower-base, mower-logic, mower-controller) ..."
SERVICES=("mower-base" "mower-logic" "mower-controller")
for svc in "${SERVICES[@]}"; do
  if systemctl is-active "$svc" >/dev/null 2>&1; then
    echo "WARN: $svc is active — it may hold serial ports needed by the container."
  else
    echo "OK: $svc is not active."
  fi
done

# --- Free disk on /userdata ---
echo "Checking free disk on /userdata ..."
if mountpoint -q /userdata; then
  USR_FREE=$(df -B1 /userdata | awk 'NR==2 {print $4}')
  USR_TOTAL=$(df -B1 /userdata | awk 'NR==2 {print $2}')
  USR_USED=$((USR_TOTAL - USR_FREE))
  USR_GB=$((USR_USED / 1000000000))
  echo "  Used: ${USR_GB} GB, Free: $((USR_FREE / 1000000000)) GB"

  if [ "$USR_FREE" -lt 10000000000 ]; then
    # Less than 10 GB free
    echo "WARNING: Less than 10 GB free on /userdata. Current free: $((USR_FREE / 1000000000)) GB"
  fi
else
  echo "NOTE: /userdata is not a mountpoint; skipping disk check."
fi

# --- Go / No-go summary ---
echo ""
echo "=== Preflight summary ==="

ALL_OK=true

if [ "${DOCKER_OK:-}" = "no" ]; then
  echo "Docker not available — fix before proceeding."
  ALL_OK=false
fi

# Check serial nodes
SERIAL_OK=true
for node in serial_mower serial_imu serial_rtk; do
  if [ -L "/dev/${node}" ]; then
    TARGET=$(readlink "/dev/${node}" 2>/dev/null || true)
    if [ "$TARGET" != "ttyS${!}" ] 2>/dev/null; then
      # More explicit check
      case "$node" in
        serial_mower) expected="ttyS9" ;;
        serial_imu)   expected="ttyS1" ;;
        serial_rtk)   expected="ttyS4" ;;
      esac
      if [ "$TARGET" != "$expected" ]; then
        echo "FAIL: /dev/serial_${node} -> $expected expected but got $TARGET"
        SERIAL_OK=false
        ALL_OK=false
      else
        echo "PASS: /dev/serial_${node} -> $expected"
      fi
    fi
  else
    echo "FAIL: /dev/serial_${node} missing"
    SERIAL_OK=false
    ALL_OK=false
  fi
done

if $SERIAL_OK; then
  echo "PASS: All serial device nodes are present and correctly linked."
fi

# Check /userdata space
if mountpoint -q /userdata && [ "${USR_FREE:-}" -lt 10000000000 ] 2>/dev/null; then
  echo "WARNING: /userdata has less than 10 GB free — may affect container runtime."
  ALL_OK=false
fi

if $ALL_OK; then
  echo "GO: All preflight checks passed. Safe to bring up the ROS 2 stack."
else
  echo "NO-GO: Some preflight checks failed. Resolve the issues above before starting."
fi