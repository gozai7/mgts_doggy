FROM ros:jazzy-ros-base@sha256:5147d3ae250ce754afd399f6bfec15b33dd4e473ca357e30432a3182031bfad6 AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /mgts
ENV PYTHONUNBUFFERED=1
ENTRYPOINT ["/mgts/entrypoint"]
COPY entrypoint .

FROM base AS solver-deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server \
    ros-jazzy-rosbag2-storage-mcap python3-scipy python3-pil && rm -rf /var/lib/apt/lists/*

FROM base AS sim-deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-robot-localization ros-jazzy-realsense2-description ros-jazzy-xacro \
    ros-jazzy-teleop-twist-keyboard ros-jazzy-rosbag2-storage-mcap \
    libgl1-mesa-dri && rm -rf /var/lib/apt/lists/*
ARG GO2_REV=55ff151632a0220bf3ae0c0772392615d86da7a8
WORKDIR /go2
RUN git init -q src && git -C src remote add origin https://github.com/khaledgabr77/unitree_go2_ros2 \
    && git -C src fetch --depth=1 origin "${GO2_REV}" && git -C src checkout --detach FETCH_HEAD \
    && source /opt/ros/jazzy/setup.bash && MAKEFLAGS=-j2 colcon build --executor sequential \
       --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && rm -rf build log src/.git
WORKDIR /mgts

FROM sim-deps AS sim
RUN sed -i -e "/^            controller_status_check,$/d" -e "s/' -r'/os.environ.get('MGTS_GZ_ARGS', ' -r')/" /go2/install/unitree_go2_sim/share/unitree_go2_sim/launch/unitree_go2_launch.py
COPY sim sim
COPY configs configs
CMD ["python3", "sim/capture.py"]

FROM solver-deps AS solver
COPY solver/CMakeLists.txt solver/export_graph.cpp solver/
# Build an opt-in Karto library against the headers shipped with slam_toolbox 2.8.5.
ADD --checksum=sha256:ae7dbffdd5850e7f9d86bab65be2d16d0e0e6034090f93c27f095dac07db6816 https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/ec8f7635dea317b531c419f798f87d90a336f32e/lib/karto_sdk/src/Mapper.cpp /tmp/karto/Mapper.cpp
ADD --checksum=sha256:de8c7ec5d468f957e7a1e394636f82c6e1c484933e312993d35c915079a346b0 https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/ec8f7635dea317b531c419f798f87d90a336f32e/lib/karto_sdk/src/Karto.cpp /tmp/karto/Karto.cpp
COPY solver/karto/ solver/karto/
RUN source /opt/ros/jazzy/setup.bash && grep -q '<version>2.8.5</version>' /opt/ros/jazzy/share/slam_toolbox/package.xml \
    && patch -d /tmp/karto -p0 <solver/karto/directional.patch \
    && cmake -DCMAKE_PREFIX_PATH=/opt/ros/jazzy -DCMAKE_BUILD_TYPE=Release -DKARTO_SOURCE=/tmp/karto -S solver -B /tmp/graph \
    && cmake --build /tmp/graph -j2 && ctest --test-dir /tmp/graph --output-on-failure && cmake --install /tmp/graph && rm -rf /tmp/graph /tmp/karto
COPY solver solver
COPY configs configs
CMD ["python3", "solver/solve.py", "--help"]
