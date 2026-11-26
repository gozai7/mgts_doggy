# Base inherited from the supplied repository. Source dependencies are vendored at locked commits.
FROM ros:jazzy-ros-base@sha256:5147d3ae250ce754afd399f6bfec15b33dd4e473ca357e30432a3182031bfad6 AS cpu
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG BUILD_JOBS=2
ARG WITH_VIEWER=OFF
ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1
ENV CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git curl ca-certificates python3-pip python3-venv \
    python3-colcon-common-extensions python3-rosdep python3-numpy python3-scipy python3-yaml \
    libboost-all-dev libeigen3-dev libmetis-dev libomp-dev libfmt-dev libspdlog-dev \
    libglm-dev libglfw3-dev libpng-dev libjpeg-dev libpcl-dev libtbb-dev \
    ros-jazzy-ament-cmake-auto ros-jazzy-rosbag2-storage-mcap ros-jazzy-rosbag2-storage-default-plugins \
    ros-jazzy-tf2-sensor-msgs ros-jazzy-pcl-conversions ros-jazzy-rclcpp-components \
    ros-jazzy-nav2-map-server ros-jazzy-rviz2 ros-jazzy-plotjuggler-ros && rm -rf /var/lib/apt/lists/*
WORKDIR /mgts
COPY requirements-competition.txt .
RUN python3 -m venv --system-site-packages /opt/competition-venv \
    && /opt/competition-venv/bin/pip install --no-cache-dir -r requirements-competition.txt
ENV PATH=/opt/competition-venv/bin:${PATH}
COPY . .
RUN bash docker/competition/resolve-submodules.sh \
    && BUILD_JOBS=${BUILD_JOBS} WITH_VIEWER=${WITH_VIEWER} bash docker/competition/build.sh \
    && chmod +x docker/competition/entrypoint.sh
ENTRYPOINT ["/mgts/docker/competition/entrypoint.sh"]
CMD ["python3", "-m", "competition.runner", "--help"]
