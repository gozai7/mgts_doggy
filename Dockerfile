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
RUN source /opt/ros/jazzy/setup.bash && cmake -DCMAKE_PREFIX_PATH=/opt/ros/jazzy -S solver -B /tmp/graph \
    && cmake --build /tmp/graph && mv /tmp/graph/export_graph /usr/local/bin/ && rm -rf /tmp/graph
COPY solver solver
COPY configs configs
CMD ["python3", "solver/solve.py", "--help"]
