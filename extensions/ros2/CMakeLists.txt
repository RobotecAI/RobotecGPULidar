
if (NOT $ENV{ROS_DISTRO} STREQUAL "humble")
    message(FATAL_ERROR "ROS $ENV{ROS_DISTRO} not supported. Only humble is available.")
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

target_sources(RobotecGPULidar PRIVATE
    src/api/apiRos2.cpp
    src/graph/Ros2PublishPointsNode.cpp
    src/graph/Ros2PublishPointVelocityMarkersNode.cpp
)

target_include_directories(RobotecGPULidar PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
)
target_link_libraries(RobotecGPULidar PRIVATE
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
)

target_include_directories(RobotecGPULidar
    PUBLIC include
    PRIVATE src
)
