# LIO_SAM_6AXIS on ROS1 Noetic (arm64 / aarch64 host: GB10).
# The host has no ROS; this image provides Noetic + GTSAM + the SLAM package.
# The project repo is bind-mounted at /work at run time (see docker/build.sh and
# scripts/run_lio_sam.sh), so converted bags and outputs live on the host.
FROM ros:noetic-perception-focal

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- system + ROS build/runtime deps -----------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        git wget cmake build-essential \
        libboost-all-dev libtbb-dev \
        ros-noetic-pcl-ros ros-noetic-pcl-conversions \
        ros-noetic-tf2-sensor-msgs ros-noetic-robot-localization \
        ros-noetic-navigation ros-noetic-roslint \
        python3-catkin-tools python3-osrf-pycommon \
    && rm -rf /var/lib/apt/lists/*

# --- GTSAM 4.0.3 from source (reliable on arm64; PPA arm64 builds are flaky) --
# march=native off (portability), system Eigen on (matches PCL/ROS Eigen).
RUN cd /tmp && \
    wget -q https://github.com/borglab/gtsam/archive/refs/tags/4.0.3.tar.gz && \
    tar xf 4.0.3.tar.gz && cd gtsam-4.0.3 && mkdir build && cd build && \
    cmake -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
          -DGTSAM_BUILD_TESTS=OFF \
          -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
          -DGTSAM_BUILD_UNSTABLE=ON \
          -DCMAKE_BUILD_TYPE=Release .. && \
    make -j"$(nproc)" && make install && ldconfig && \
    cd / && rm -rf /tmp/gtsam-4.0.3 /tmp/4.0.3.tar.gz

# --- LIO_SAM_6AXIS catkin workspace ------------------------------------------
RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src && \
    git clone --depth 1 https://github.com/JokerJohn/LIO_SAM_6AXIS.git

# Build only lio_sam_6axis (+deps); skip the bundled rviz_satellite package,
# which needs OGRE/rviz and is visualization-only (we run headless).
RUN source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    catkin_make -DCMAKE_BUILD_TYPE=Release \
        -DCATKIN_WHITELIST_PACKAGES="lio_sam_6axis" -j"$(nproc)"

# Auto-source the workspace for interactive/`docker run bash -lc` use.
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /work
