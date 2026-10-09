#!/usr/bin/env bash
# Runs the nav stack smoke test in the dev image (domain 81, localhost only).
#   test/smoke/run_smoke.sh            # positive checks 1-7
#   test/smoke/run_smoke.sh negative   # check 4 negative control (no bounds_keeper layer)
#   test/smoke/run_smoke.sh keeper_bumper_only     # positive control: bounds_keeper kept, bumper source only
#   test/smoke/run_smoke.sh negative_bumper_only   # ... and only the (empty) bumper source
# Must be run from the workspace root (it is mounted at /work). Needs install/ built:
#   DEV_ROS_DOMAIN_ID=81 ./scripts/dev_build.sh build --packages-select mower_navigation \
#       bumper_controller mower_interfaces mowgli_nav2_plugins
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
mkdir -p log
mode="${1:-positive}"
if [ "$mode" = negative ] || [ "$mode" = negative_bumper_only ] || [ "$mode" = keeper_bumper_only ]; then
  launch_args="params_file:=/work/log/nav2_params_no_bounds_keeper.yaml"
  prep="python3 - <<'PY'
import yaml
src = '/work/src/mower_navigation/config/nav2_params.yaml'
d = yaml.safe_load(open(src))
lp = d['local_costmap']['local_costmap']['ros__parameters']
if '$mode' != 'keeper_bumper_only':
    lp['plugins'] = ['obstacle_layer', 'inflation_layer']
if '$mode' != 'negative':
    lp['obstacle_layer']['observation_sources'] = 'bumper'   # no clearing source: bounds stay empty
yaml.safe_dump(d, open('/work/log/nav2_params_no_bounds_keeper.yaml', 'w'))
PY"
  smoke_args="--checks 4"; [ "$mode" = keeper_bumper_only ] || smoke_args="--negative"
else
  launch_args=""; prep=":"; smoke_args=""
fi
docker run --rm --network host -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=81 -v "$PWD":/work -w /work \
  mower:humble-dev-amd64 bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; $prep
  ros2 launch mower_navigation navigation.launch.py $launch_args > /work/log/smoke_launch.log 2>&1 &
  sleep 3
  timeout 480 python3 src/mower_navigation/test/nav_stack_smoke.py --log /work/log/smoke_launch.log $smoke_args
  rc=\$?; kill -INT %1; sleep 3; exit \$rc"
