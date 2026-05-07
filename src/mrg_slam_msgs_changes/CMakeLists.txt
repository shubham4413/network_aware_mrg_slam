cmake_minimum_required(VERSION 3.8)
project(mrg_slam_msgs)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/EdgeEstimate.msg"
  "msg/EdgeRos.msg"
  "msg/FloorCoeffs.msg"
  "msg/GraphEstimate.msg"
  "msg/GraphRos.msg"
  "msg/KeyFrameEstimate.msg"
  "msg/KeyFrameRos.msg"
  "msg/PoseWithName.msg"
  "msg/PoseWithNameArray.msg"
  "msg/ScanMatchingStatus.msg"
  "msg/SlamStatus.msg"
  "msg/LinkQuality.msg"
  "msg/AdaptiveTxStats.msg"
)

set(srv_files
  "srv/AddStaticKeyFrames.srv"
  "srv/GetGraphEstimate.srv"
  "srv/GetGraphUuids.srv"
  "srv/GetMap.srv"
  "srv/LoadGraph.srv"
  "srv/PublishGraph.srv"
  "srv/RequestGraphs.srv"
  "srv/SaveGids.srv" # TODO: remove
  "srv/SaveGraph.srv"
  "srv/SaveMap.srv"
  "srv/PublishMap.srv"
  "srv/RemoveEdge.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
