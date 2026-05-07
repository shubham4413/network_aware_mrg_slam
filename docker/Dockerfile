# Build:  docker build -t bandwidth_aware_mrg_slam -f docker/Dockerfile .
# Run:    docker run -it --rm bandwidth_aware_mrg_slam

FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-nmea-msgs \
    ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-sensor-msgs ros-humble-tf2-geometry-msgs \
    ros-humble-geodesy ros-humble-libg2o ros-humble-interactive-markers \
    libceres-dev ros-humble-rmw-zenoh-cpp ros-humble-rcutils libgeographic-dev \
    libomp-dev libpcl-dev libsuitesparse-dev \
    python3-vcstool python3-pip wget git \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -ms /bin/bash ubuntu
USER ubuntu

RUN mkdir -p /home/ubuntu/ros2_ws/src
WORKDIR /home/ubuntu/ros2_ws
RUN git clone https://github.com/aserbremen/Multi-Robot-Graph-SLAM /tmp/mrg_slam_meta \
    && cp /tmp/mrg_slam_meta/mrg_slam.repos . \
    && vcs import src < mrg_slam.repos \
    && rm -rf /tmp/mrg_slam_meta

COPY --chown=ubuntu:ubuntu src/adaptive_cloud_transmitter.hpp         src/mrg_slam/apps/adaptive_cloud_transmitter.hpp
COPY --chown=ubuntu:ubuntu src/mrg_slam_component.cpp                 src/mrg_slam/apps/mrg_slam_component.cpp
COPY --chown=ubuntu:ubuntu src/prefiltering_component.cpp             src/mrg_slam/apps/prefiltering_component.cpp
COPY --chown=ubuntu:ubuntu src/scan_matching_odometry_component.cpp   src/mrg_slam/apps/scan_matching_odometry_component.cpp
COPY --chown=ubuntu:ubuntu src/mrg_slam.yaml                          src/mrg_slam/config/mrg_slam.yaml

COPY --chown=ubuntu:ubuntu src/mrg_slam_msgs_changes/LinkQuality.msg     src/mrg_slam_msgs/msg/LinkQuality.msg
COPY --chown=ubuntu:ubuntu src/mrg_slam_msgs_changes/AdaptiveTxStats.msg src/mrg_slam_msgs/msg/AdaptiveTxStats.msg
COPY --chown=ubuntu:ubuntu src/mrg_slam_msgs_changes/SlamStatus.msg      src/mrg_slam_msgs/msg/SlamStatus.msg
COPY --chown=ubuntu:ubuntu src/mrg_slam_msgs_changes/CMakeLists.txt      src/mrg_slam_msgs/CMakeLists.txt

COPY --chown=ubuntu:ubuntu src/distance_bandwidth_shaper/ src/distance_bandwidth_shaper/

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --parallel-workers 1 \
      --packages-select small_gicp fast_gicp ndt_omp mrg_slam_msgs mrg_slam distance_bandwidth_shaper

RUN echo "source /ros_entrypoint.sh" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/ros2_ws/install/setup.sh" >> /home/ubuntu/.bashrc

CMD ["/bin/bash"]
