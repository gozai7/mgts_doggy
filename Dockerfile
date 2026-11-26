# Base inherited from the supplied repository. Source dependencies are vendored at locked commits.
FROM ros:jazzy-ros-base@sha256:5147d3ae250ce754afd399f6bfec15b33dd4e473ca357e30432a3182031bfad6 AS base
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
ENTRYPOINT ["/mgts/docker/competition/entrypoint.sh"]
CMD ["python3", "-m", "competition.runner", "--help"]

FROM base AS cpu
COPY . .
RUN bash docker/competition/resolve-submodules.sh \
    && BUILD_JOBS=${BUILD_JOBS} WITH_VIEWER=${WITH_VIEWER} bash docker/competition/build.sh \
    && chmod +x docker/competition/entrypoint.sh

# Use binary GTSAM dependencies; build GLIM and its ROS wrapper at the team revisions.
FROM base AS gpu_deps
ENV PATH=/usr/local/cuda-12.6/bin:/opt/competition-venv/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH}
ADD --checksum=sha256:d2a6b11c096396d868758b86dab1823b25e14d70333f1dfa74da5ddaf6a06dba https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb /tmp/cuda-keyring.deb
ADD --checksum=sha256:08c6f6bff0bf9b991398c693d44a16871932bc7de969848413e3066dc1acc77e https://koide3.github.io/ppa/ubuntu2404/KEY.gpg /tmp/glim-key
RUN dpkg -i /tmp/cuda-keyring.deb && gpg --dearmor -o /usr/share/keyrings/glim.gpg /tmp/glim-key \
    && echo 'deb [signed-by=/usr/share/keyrings/glim.gpg] https://koide3.github.io/ppa/ubuntu2404 ./' > /etc/apt/sources.list.d/glim.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       cuda-nvcc-12-6 cuda-cudart-dev-12-6 \
       libgtsam-points-cuda12.6-dev=1.2.2 libgtsam-notbb-dev=4.3.0 \
    && ldconfig && rm -rf /var/lib/apt/lists/*
FROM gpu_deps AS gpu
# The pinned GLIM uses the header-only has_index() added after the 1.2.2 package.
COPY vendor/gtsam_points/include/gtsam_points/util/indexed_sliding_window.hpp /usr/local/include/gtsam_points/util/
COPY vendor/glim /mgts/vendor/glim
RUN --mount=type=cache,target=/tmp/build-glim \
    source /opt/ros/jazzy/setup.bash && cmake -S vendor/glim -B /tmp/build-glim -DCMAKE_BUILD_TYPE=Release \
       -DBUILD_WITH_CUDA=ON -DBUILD_WITH_VIEWER=${WITH_VIEWER} -DBUILD_WITH_OPENCV=OFF -DBUILD_WITH_MARCH_NATIVE=OFF \
    && cmake --build /tmp/build-glim --parallel ${BUILD_JOBS} && cmake --install /tmp/build-glim && ldconfig
COPY BACKEND .
COPY vendor/ vendor/
COPY docker/competition/build.sh docker/competition/
RUN BUILD_WITH_CUDA=ON bash docker/competition/build.sh
COPY competition/ competition/
COPY configs/ configs/
COPY tools/ tools/
COPY tests/competition/ tests/competition/
COPY THIRD_PARTY.lock.json .
COPY --chmod=755 docker/competition/entrypoint.sh docker/competition/entrypoint.sh
