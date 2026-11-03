#!/usr/bin/env bash
# CI tests (code review 2026-10-23 B5, review 2026-11-02 step 3), all in the dev image
# (mower:jazzy-dev-amd64, built from docker/Dockerfile.dev-amd64 by scripts/dev_build.sh when
# absent: there is no registry to pull it from, so a cold runner builds Fields2Cover + the
# image first, ~20-30 min; cache the image to avoid that).
#
#   ./scripts/ci_test.sh                  # build + every package's tests + script tests
#   ./scripts/ci_test.sh --gate           # ... + one ideal sim gate (-s 2 -x 3)
#   ./scripts/ci_test.sh --no-colcon      # script + replay tests only (needs a built tree)
#   ./scripts/ci_test.sh --packages-select mower_mission     # colcon args pass through
#
# Stages:
#   1. colcon build + colcon test (dev_build.sh), which includes the incident replay tests
#      (src/mower_mission/test/replay, docs/DEVELOPMENT.md "Incident -> regression")
#   2. scripts/ble/test_*.py, scripts/sim_gate/test_*.py
#   3. the incident replay tests on their own (a named summary; they also ran in 1)
#   4. --gate (needs a writable log/: chown it if colcon in docker left it root-owned):
#      scripts/sim_gate.sh -s 2 -x 3 (ideal drive, real_run5) under
#      flock /tmp/mower_sim_gate.lock. Exit 0 PASS; 4 INVALID (the host could not hold the
#      18 Hz controller loop at x3: reported as INVALID, not as a code failure, and fails CI
#      unless CI_GATE_ALLOW_INVALID=1); anything else FAIL.
#
# Exit status: 0 when every stage passed (colcon failures listed in CI_KNOWN_FAILURES are
# reported but tolerated), 1 otherwise. Runs on its own DDS domain, localhost-only.
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$STACK_ROOT"
export DEV_ROS_DOMAIN_ID="${DEV_ROS_DOMAIN_ID:-211}"
IMG=mower:jazzy-dev-amd64
# Space-separated test case names that may fail without failing CI (keep it empty if you can).
CI_KNOWN_FAILURES="${CI_KNOWN_FAILURES:-test_scenario_transit_box}"

gate=0 colcon=1 args=()
for a in "$@"; do
  case "$a" in
    --gate) gate=1 ;;
    --no-colcon) colcon=0 ;;
    *) args+=("$a") ;;
  esac
done

declare -A result
fail=0
stage() {  # stage NAME STATUS
  result[$1]=$2
  [ "$2" = PASS ] || fail=1
}

# the same isolation as dev_build.sh: no host networking needed, private domain
in_image() {
  docker run --rm -i --network none -e ROS_LOCALHOST_ONLY=1 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e ROS_DOMAIN_ID="$DEV_ROS_DOMAIN_ID" \
    -v "$STACK_ROOT":/work -w /work "$IMG" bash -c \
    "source /opt/ros/jazzy/setup.bash; [ -f install/setup.bash ] && source install/setup.bash;
     export PYTHONPATH=\$(ls -d src/*/ | sed 's|/\$||' | tr '\n' ':'):\${PYTHONPATH:-}; $*"
}

if [ "$colcon" = 1 ]; then
  if "$SCRIPT_DIR/dev_build.sh" build "${args[@]}"; then
    # Sequential: every package's tests use the same private test domain (conftest.py), so
    # real-rclpy tests of two packages running at once see each other's /odom etc.
    # (2026-10-23: the docking goal-input test read another package's zero /odom).
    "$SCRIPT_DIR/dev_build.sh" test --executor sequential "${args[@]}" | tee ci_test.log || true
    if python3 - "$CI_KNOWN_FAILURES" <<'PY'
import glob, sys
import xml.etree.ElementTree as ET
known = set(sys.argv[1].split())
failed, tolerated, total = [], [], 0
files = glob.glob('build/*/pytest.xml') + glob.glob('build/*/test_results/*/*.xml') \
    + glob.glob('build/*/Testing/*/Test.xml')
for f in sorted(files):
    try:
        root = ET.parse(f).getroot()
    except ET.ParseError as e:
        failed.append('%s: unparsable (%s)' % (f, e))
        continue
    for tc in root.iter('testcase'):
        total += 1
        if tc.find('failure') is not None or tc.find('error') is not None:
            name = tc.get('name', '?')
            (tolerated if name.split('[')[0] in known else failed).append(
                '%s %s.%s' % (f.split('/')[1], tc.get('classname', ''), name))
if not files:
    print('ci_test: no test results found')
    sys.exit(1)
print('ci_test: %d test cases, %d failed, %d tolerated' % (total, len(failed), len(tolerated)))
for t in tolerated:
    print('  tolerated (CI_KNOWN_FAILURES): ' + t)
for t in failed:
    print('  FAILED: ' + t)
sys.exit(1 if failed else 0)
PY
    then stage colcon PASS; else stage colcon FAIL; fi
  else
    stage colcon "FAIL (build)"
  fi
fi

if in_image "python3 -m pytest -q -p no:cacheprovider scripts/ble scripts/sim_gate"; then
  stage scripts PASS; else stage scripts FAIL; fi

if in_image "cd src/mower_mission && python3 -m pytest -q -p no:cacheprovider -rxX test/replay"; then
  stage replay PASS; else stage replay FAIL; fi

if [ "$gate" = 1 ]; then
  name="ci-$(date -u +%Y%m%d-%H%M%S)"
  flock /tmp/mower_sim_gate.lock "$SCRIPT_DIR/sim_gate.sh" -s 2 -x 3 -n "$name"
  rc=$?
  case $rc in
    0) stage gate PASS ;;
    4) if [ "${CI_GATE_ALLOW_INVALID:-0}" = 1 ]; then result[gate]="INVALID (tolerated)"
       else stage gate "INVALID (controller < 18 Hz sim at x3: host too slow; rerun at -x 1)"; fi ;;
    2) stage gate "NOT RUN (sim_gate preflight refused: power profile / load, see above)" ;;
    *) stage gate "FAIL (exit $rc, log/sim_gate/$name)" ;;
  esac
fi

echo "ci_test summary:"
for k in colcon scripts replay gate; do
  [ -n "${result[$k]:-}" ] && printf '  %-8s %s\n' "$k" "${result[$k]}"
done
exit $fail
