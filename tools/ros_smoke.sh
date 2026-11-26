#!/usr/bin/env bash
set -euo pipefail
# Actual SLAM/localization test. No fabricated trajectories or copied reference output.
case_dir="$(mktemp -d /results/ros-smoke-XXXXXX)"
python3 tools/synthetic_bags.py "$case_dir/input"
python3 -m competition.runner run-all --config "$case_dir/input/dataset.yaml"
test -s "$case_dir/input/results/mapping/map.pcd"
test -s "$case_dir/input/results/localization/trajectory.tum"
test -s "$case_dir/input/results/occupancy/map.pgm"
echo "Synthetic ROS smoke finished: $case_dir (not evidence of real robot accuracy)"
