cmake_minimum_required(VERSION 3.18) # # 3.18 To have DISCOVERY_MODE option for gtest_discover_tests()

set(RGL_TEST_FILES
    src/apiReadmeExample.cpp
    src/apiGeneralCallsTest.cpp
    src/graph/asyncStressTest.cpp
    src/graph/DistanceFieldTest.cpp
    src/externalLibraryTest.cpp
    src/graph/gaussianStressTest.cpp
    src/graph/gaussianPoseIndependentTest.cpp
    src/testMat3x4f.cpp
    src/graph/VelocityDistortionTest.cpp
    src/graph/addChildTest.cpp
    src/graph/fullLinearTest.cpp
    src/graph/getResultTest.cpp
    src/graph/nodeInputImpactTest.cpp
    src/graph/nodeRemovalTest.cpp
    src/graph/setPriorityTest.cpp
    src/graph/nodes/CompactByFieldPointsNodeTest.cpp
    src/graph/nodes/FormatPointsNodeTest.cpp
    src/graph/nodes/FromArrayPointsNodeTest.cpp
    src/graph/nodes/FromMat3x4fRaysNodeTest.cpp
    src/graph/nodes/FilterGroundPointsNodeTest.cpp
    src/graph/nodes/GaussianNoiseAngularHitpointNodeTest.cpp
    src/graph/nodes/GaussianNoiseAngularRayNodeTest.cpp
    src/graph/nodes/GaussianNoiseDistanceNodeTest.cpp
    src/graph/MaskRaysTest.cpp
    src/graph/nodes/RaytraceNodeTest.cpp
    src/graph/nodes/RadarPostprocessPointsNodeTest.cpp
    src/graph/nodes/RadarTrackObjectsNodeTest.cpp
    src/graph/nodes/SetRingIdsRaysNodeTest.cpp
    src/graph/nodes/SetTimeOffsetsRaysNodeTest.cpp
    src/graph/nodes/SpatialMergePointsNodeTest.cpp
    src/graph/nodes/TemporalMergePointsNodeTest.cpp
    src/graph/nodes/TransformPointsNodeTest.cpp
    src/graph/nodes/TransformPointsNodeTest.cpp
    src/graph/nodes/TransformRaysNodeTest.cpp
    src/graph/nodes/VisualizePointsNodeTest.cpp
    src/graph/nodes/YieldPointsNodeTest.cpp
    src/helpers/pointsTest.cpp
    src/memory/arrayChangeStreamTest.cpp
    src/memory/arrayOpsTest.cpp
    src/memory/arrayTypingTest.cpp
    src/scene/animationVelocityTest.cpp
    src/scene/entityAPITest.cpp
    src/scene/entityIdTest.cpp
    src/scene/entityLaserRetroTest.cpp
    src/scene/entityVelocityTest.cpp
    src/scene/meshAPITest.cpp
    src/scene/textureTest.cpp
    src/synchronization/graphAndCopyStream.cpp
    src/synchronization/graphThreadSynchronization.cpp
    src/synchronization/testKernel.cu
    src/scene/incidentAngleTest.cpp
    src/graph/multiReturnTest.cpp
)

if (RGL_BUILD_ROS2_EXTENSION)
    list(APPEND RGL_TEST_FILES
        src/graph/Ros2NodesTest.cpp
        src/graph/nodes/Ros2PublishRadarScanNodeTest.cpp
        src/graph/nodes/Ros2PublishPointsNodeTest.cpp
        src/scene/radarTest.cpp
    )
endif()

# Only Linux
if ((NOT WIN32))
#    list(APPEND RGL_TEST_FILES
        # Placeholder for Linux-only tests.
#    )
endif()

# On Windows, tape is not available since it uses Linux sys-calls (mmap)
# AutoTape interferes with tape tests (e.g. double rgl_tape_record_begin()) which is non-trivial to fix.
if ((NOT WIN32) AND (NOT RGL_AUTO_TAPE_PATH))
    list(APPEND RGL_TEST_FILES
            src/TapeTest.cpp
    )
endif()

file(COPY ${CMAKE_SOURCE_DIR}/test/data/ DESTINATION ${CMAKE_BINARY_DIR}/data/)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/test)
add_executable(RobotecGPULidar_test ${RGL_TEST_FILES})

target_compile_definitions(RobotecGPULidar_test PRIVATE
    RGL_TEST_DATA_DIR="${CMAKE_CURRENT_LIST_DIR}/data"
)

target_link_libraries(RobotecGPULidar_test PRIVATE
    gtest_main
    gmock_main
    RobotecGPULidar
)

target_include_directories(RobotecGPULidar_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/synchronization
)

if (RGL_BUILD_UDP_EXTENSION)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../extensions/udp/test ${CMAKE_CURRENT_BINARY_DIR}/udp)
endif()

if (RGL_BUILD_WEATHER_EXTENSION)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../extensions/weather/test ${CMAKE_CURRENT_BINARY_DIR}/weather)
endif()

if (RGL_BUILD_ROS2_EXTENSION)
    # This allows accessing Nodes directly (as opposed to through RGL-API)
    # It is needed by EntityVelocity test, which makes a use of non-public Ros2 node.
    find_package(rclcpp REQUIRED)
    find_package(radar_msgs REQUIRED)
    list(REMOVE_DUPLICATES radar_msgs_LIBRARIES)
    target_include_directories(RobotecGPULidar_test PRIVATE
        ${CMAKE_SOURCE_DIR}/extensions/ros2/src
        ${rclcpp_INCLUDE_DIRS}
    )
    target_link_libraries(RobotecGPULidar_test PRIVATE
        ${rclcpp_LIBRARIES}
        ${radar_msgs_LIBRARIES}
    )
endif()

# Set $ORIGIN rpath to search for dependencies in the executable location (Linux only)
set_target_properties(RobotecGPULidar_test PROPERTIES LINK_FLAGS "-Wl,-rpath,$ORIGIN")

include(GoogleTest)

gtest_discover_tests(RobotecGPULidar_test
    DISCOVERY_MODE PRE_TEST)
