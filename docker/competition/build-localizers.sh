#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
jobs="${BUILD_JOBS:-2}"
stage="${1:-all}"
if [ "$stage" != adapter ]; then
for pkg in beluga beluga_vdb; do
  cmake -S "/mgts/vendor/beluga/$pkg" -B "/tmp/build-$pkg" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PROJECT_INCLUDE=/mgts/docker/competition/beluga-sophus.cmake
  cmake --build "/tmp/build-$pkg" --parallel "$jobs"
  cmake --install "/tmp/build-$pkg"
done
mkdir -p /opt/competition_ws/src
for pkg in kiss-icp ndt_omp_ros2 lidar_localization_ros2; do
  cp -a "/mgts/vendor/$pkg" /opt/competition_ws/src/
done
cd /opt/competition_ws
MAKEFLAGS="-j$jobs" colcon build --packages-up-to kiss_icp lidar_localization_ros2 \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DUSE_SYSTEM_EIGEN3=ON -DUSE_SYSTEM_SOPHUS=ON -DUSE_SYSTEM_TBB=ON -DUSE_SYSTEM_TSL-ROBIN-MAP=ON
fi
if [ "$stage" != dependencies ]; then
  cp -a /mgts/localization_ros /opt/competition_ws/src/
  cd /opt/competition_ws
  MAKEFLAGS="-j$jobs" colcon build --packages-select mgts_localization --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
fi
ldconfig
