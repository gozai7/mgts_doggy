#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/competition_ws/install/setup.bash
export PYTHONPATH=/mgts:${PYTHONPATH:-}
exec "$@"
