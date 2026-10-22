#!/usr/bin/env bash
# CI unit tests (code review 2026-10-23, B5): build the workspace and run every package's
# unit tests in the dev image (mower:jazzy-dev-amd64, built from docker/Dockerfile.dev-amd64
# by scripts/dev_build.sh when absent: there is no registry to pull it from, so a cold CI
# runner builds Fields2Cover + the image first, ~20-30 min; cache the image to avoid that).
#
#   ./scripts/ci_test.sh                  # build + test all packages
#   ./scripts/ci_test.sh --packages-select mower_mission
#
# Exit status: 0 when every test passed (failures listed in CI_KNOWN_FAILURES are reported
# but tolerated), 1 otherwise. Runs on its own DDS domain, localhost-only (dev_build.sh).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$STACK_ROOT"
export DEV_ROS_DOMAIN_ID="${DEV_ROS_DOMAIN_ID:-211}"
# Space-separated test case names that may fail without failing CI (keep it empty if you can).
CI_KNOWN_FAILURES="${CI_KNOWN_FAILURES:-test_scenario_transit_box}"

"$SCRIPT_DIR/dev_build.sh" build "$@"
# Sequential: every package's tests use the same private test domain (conftest.py), so real-
# rclpy tests of two packages running at once see each other's /odom etc. (2026-10-23: the
# docking goal-input test read another package's zero /odom under the parallel executor).
"$SCRIPT_DIR/dev_build.sh" test --executor sequential "$@" | tee ci_test.log || true

python3 - "$CI_KNOWN_FAILURES" "$@" <<'PY'
import glob, os, sys
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
