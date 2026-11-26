#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
jobs="${BUILD_JOBS:-2}"
mode="${BUILD_WITH_CUDA:-OFF}"
backend="$(cat /mgts/BACKEND)"
build_cmake() {
  local name="$1"; shift
  cmake -S "/mgts/vendor/$name" -B "/tmp/build-$name-$mode" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local "$@"
  cmake --build "/tmp/build-$name-$mode" --parallel "$jobs"
  cmake --install "/tmp/build-$name-$mode"
}
if [ "$backend" = glim ] && [ "$mode" = OFF ]; then
  build_cmake gtsam -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_UNSTABLE_BUILD_TESTS=OFF -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
  if [ "${WITH_VIEWER:-OFF}" = ON ]; then
    build_cmake iridescence -DBUILD_EXAMPLES=OFF
  fi
  build_cmake gtsam_points -DBUILD_WITH_CUDA="$mode" -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TESTS=OFF
  build_cmake glim -DBUILD_WITH_CUDA="$mode" -DBUILD_WITH_VIEWER="${WITH_VIEWER:-OFF}" \
    -DBUILD_WITH_OPENCV=OFF -DBUILD_WITH_MARCH_NATIVE=OFF
  ldconfig
fi
rm -rf /opt/competition_ws/src
mkdir -p /opt/competition_ws/src
cp -a /mgts/vendor/ndt_omp_ros2 /opt/competition_ws/src/
cp -a /mgts/vendor/lidar_localization_ros2 /opt/competition_ws/src/
cp -a /mgts/vendor/kiss-icp /opt/competition_ws/src/
if [ "$backend" = glim ]; then
  cp -a /mgts/vendor/glim_ros2 /opt/competition_ws/src/
else
  cp -a /mgts/vendor/rko_fork /opt/competition_ws/src/rko_lio
  cp -a /mgts/vendor/lidar_slam_ros2 /opt/competition_ws/src/lidar_slam
fi
cd /opt/competition_ws
# Download/resolve at image build time only, never during competition processing.
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi
rosdep update --rosdistro jazzy
apt-get update
dep_paths=(src/ndt_omp_ros2 src/lidar_localization_ros2 src/kiss-icp)
targets=(lidar_localization_ros2 kiss_icp)
if [ "$backend" = glim ]; then
  dep_paths+=(src/glim_ros2)
  targets+=(glim_ros)
else
  dep_paths+=(src/rko_lio)
  for pkg in graph_based_slam lidarslam_msgs lidarslam_plugin_interfaces lidarslam_default_plugins lidarslam_registration_loader; do
    dep_paths+=("src/lidar_slam/$pkg")
  done
  targets+=(graph_based_slam rko_lio)
fi
rosdep install -y --from-paths "${dep_paths[@]}" --ignore-src --rosdistro jazzy \
  -t build -t buildtool -t build_export -t buildtool_export -t exec \
  --skip-keys 'glim gtsam_points iridescence'
MAKEFLAGS="-j$jobs" colcon build --packages-up-to "${targets[@]}" --executor sequential --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DBUILD_WITH_CUDA="$mode" -DBUILD_WITH_VIEWER="${WITH_VIEWER:-OFF}" -DBUILD_WITH_CV_BRIDGE=OFF \
    -DRKO_LIO_FETCH_CONTENT_DEPS=ON -DCMAKE_BUILD_PARALLEL_LEVEL="$jobs"
ldconfig
dpkg-query -W > /mgts/build-apt-versions.txt
python3 -m pip freeze > /mgts/build-python-versions.txt
rm -rf /var/lib/apt/lists/* /tmp/build-* /opt/competition_ws/build /opt/competition_ws/log
