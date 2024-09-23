
set(SUPPORTED_ROS_DISTROS humble jazzy)

list(FIND SUPPORTED_ROS_DISTROS $ENV{ROS_DISTRO} ROS_DISTRO_INDEX)
if (ROS_DISTRO_INDEX EQUAL -1)
    message(FATAL_ERROR "ROS $ENV{ROS_DISTRO} not supported. Choose one of: '${SUPPORTED_ROS_DISTROS}'.")
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(radar_msgs REQUIRED)

target_sources(RobotecGPULidar PRIVATE
    src/api/apiRos2.cpp
    src/graph/Ros2PublishPointsNode.cpp
    src/graph/Ros2PublishPointVelocityMarkersNode.cpp
    src/graph/Ros2PublishRadarScanNode.cpp
)

target_include_directories(RobotecGPULidar PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${radar_msgs_INCLUDE_DIRS}
)

# These libraries list, imported from ROS2 packages,
# contain a large number of duplicates, which results
# in linking error (LNK1170) due to exceeded line length in rsp file.
list(REMOVE_DUPLICATES radar_msgs_LIBRARIES)
list(REMOVE_DUPLICATES visualization_msgs_LIBRARIES)
list(REMOVE_DUPLICATES sensor_msgs_LIBRARIES)

target_link_libraries(RobotecGPULidar PRIVATE
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${radar_msgs_LIBRARIES}
)

target_include_directories(RobotecGPULidar
    PUBLIC include
    PRIVATE src
)
