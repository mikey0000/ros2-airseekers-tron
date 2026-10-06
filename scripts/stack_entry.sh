#!/usr/bin/env bash
# Container entrypoint for mower_humble (docker/docker-compose.yml `command:`).
# Sets up crash recording (docs/crash_recovery.md), cleans Fast DDS shm leftovers, then
# execs the launch so ros2 launch is the container's main process (respawn lives there).
# no `set -u`: the ROS setup scripts reference unset variables
source /opt/ros/humble/setup.bash
source /work/install/setup.bash

export MOWER_CRASH_DIR=${MOWER_CRASH_DIR:-/userdata/ros2/crashes}
export ROS_LOG_DIR=${ROS_LOG_DIR:-/userdata/ros2/log}
export PYTHONFAULTHANDLER=${PYTHONFAULTHANDLER:-1}   # every Python process: fatal-signal stacks on stderr
mkdir -p "$MOWER_CRASH_DIR" "$ROS_LOG_DIR"
# Core dumps: size limit per process (inherited by every node). Where they are written is
# the HOST's kernel.core_pattern (docker/host/91-ros2-cores.conf).
ulimit -c unlimited || echo "stack_entry: cannot raise the core size limit"
echo "stack_entry: core_pattern=$(cat /proc/sys/kernel/core_pattern) ulimit -c=$(ulimit -c)" \
     "ROS_LOG_DIR=$ROS_LOG_DIR crashes=$MOWER_CRASH_DIR"

MIN_AGE=0 /work/scripts/shm_gc.sh
(/work/scripts/shm_gc.sh --loop 300 &)
/work/scripts/ros_log_cleanup.sh
(/work/scripts/ros_log_cleanup.sh --loop 21600 &)

exec ros2 launch mower_bringup mower.launch.py "$@"
