#!/usr/bin/env bash
set -euo pipefail
# Actual SLAM/localization test. No fabricated trajectories or copied reference output.
case_dir="$(mktemp -d "${SMOKE_RESULTS_DIR:-/results}/ros-smoke-XXXXXX")"
run_case() {
  local input="$case_dir/$1"
  shift
  python3 tools/synthetic_bags.py "$input" --mapping-mode "${GLIM_SMOKE_MODE:-cpu}" "$@"
  python3 -m competition.runner run-all --config "$input/dataset.yaml"
  test -s "$input/results/mapping/map.pcd"
  test -s "$input/results/localization/initialization.json"
  test -s "$input/results/localization/trajectory.tum"
  test -s "$input/results/occupancy/map.pgm"
}
run_case input
if [ "$(cat BACKEND)" = fast-lio ]; then
  run_case timed --point-time
fi
echo "Synthetic ROS smoke finished: $case_dir (not evidence of real robot accuracy)"
