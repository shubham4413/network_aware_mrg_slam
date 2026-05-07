#!/usr/bin/env bash
# Initialize the Multi-Robot-Graph-SLAM submodule, copy in the modified
# files from src/, and build the workspace.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SUBMODULE_DIR="$REPO_ROOT/Multi-Robot-Graph-SLAM"

HEADER_SRC="$REPO_ROOT/src/adaptive_cloud_transmitter.hpp"
COMPONENT_SRC="$REPO_ROOT/src/mrg_slam_component.cpp"
PREFILT_SRC="$REPO_ROOT/src/prefiltering_component.cpp"
SCANMATCH_SRC="$REPO_ROOT/src/scan_matching_odometry_component.cpp"
MSGS_SRC="$REPO_ROOT/src/mrg_slam_msgs_changes"
YAML_SRC="$REPO_ROOT/src/mrg_slam.yaml"
DBS_SRC="$REPO_ROOT/src/distance_bandwidth_shaper"

HEADER_DST="$SUBMODULE_DIR/src/mrg_slam/apps/adaptive_cloud_transmitter.hpp"
COMPONENT_DST="$SUBMODULE_DIR/src/mrg_slam/apps/mrg_slam_component.cpp"
PREFILT_DST="$SUBMODULE_DIR/src/mrg_slam/apps/prefiltering_component.cpp"
SCANMATCH_DST="$SUBMODULE_DIR/src/mrg_slam/apps/scan_matching_odometry_component.cpp"
MSGS_DST="$SUBMODULE_DIR/src/mrg_slam_msgs"
YAML_DST="$SUBMODULE_DIR/src/mrg_slam/config/mrg_slam.yaml"
DBS_DST="$SUBMODULE_DIR/src/distance_bandwidth_shaper"

cd "$REPO_ROOT"
git submodule update --init --recursive

sudo apt update
sudo apt install -y \
  libomp-dev \
  libpcl-dev \
  libgeographic-dev \
  libsuitesparse-dev \
  python3-vcstool

cd "$SUBMODULE_DIR"
mkdir -p src
vcs import src < mrg_slam.repos
sudo apt install -y \
  ros-humble-geodesy \
  ros-humble-nmea-msgs \
  ros-humble-libg2o \
  ros-humble-pcl-ros
rosdep install --from-paths src --ignore-src -r -y

cp "$HEADER_SRC" "$HEADER_DST"
cp "$COMPONENT_SRC" "$COMPONENT_DST"
cp "$PREFILT_SRC" "$PREFILT_DST"
cp "$SCANMATCH_SRC" "$SCANMATCH_DST"
cp "$YAML_SRC" "$YAML_DST"

cp "$MSGS_SRC/LinkQuality.msg" "$MSGS_DST/msg/LinkQuality.msg"
cp "$MSGS_SRC/AdaptiveTxStats.msg" "$MSGS_DST/msg/AdaptiveTxStats.msg"
cp "$MSGS_SRC/SlamStatus.msg" "$MSGS_DST/msg/SlamStatus.msg"
cp "$MSGS_SRC/CMakeLists.txt" "$MSGS_DST/CMakeLists.txt"

ln -sfn "$DBS_SRC" "$DBS_DST"

cd "$SUBMODULE_DIR"
colcon build --symlink-install --parallel-workers 1

echo
echo "Build done. Source the workspace:"
echo "  source $SUBMODULE_DIR/install/setup.bash"
