cmake_minimum_required(VERSION 3.16)

set(RGL_TEST_FILES
    src/apiReadmeExample.cpp
    src/apiSurfaceTests.cpp
    src/asyncStressTest.cpp
    src/DistanceFieldTest.cpp
    src/externalLibraryTest.cpp
    src/gaussianStressTest.cpp
    src/gaussianPoseIndependentTest.cpp
    src/testMat3x4f.cpp
    src/VelocityDistortionTest.cpp
    src/features/loggingTests.cpp
    src/graph/addChildTest.cpp
    src/graph/graphCaseTest.cpp
    src/graph/setPriorityTest.cpp
    src/graph/nodes/CompactPointsNodeTest.cpp
    src/graph/nodes/FormatPointsNodeTest.cpp
    src/graph/nodes/FromArrayPointsNodeTest.cpp
    src/graph/nodes/FromMat3x4fRaysNodeTest.cpp
    src/graph/nodes/GaussianNoiseAngularHitpointNodeTest.cpp
    src/graph/nodes/GaussianNoiseAngularRayNodeTest.cpp
    src/graph/nodes/GaussianNoiseDistanceNodeTest.cpp
    src/graph/nodes/RaytraceNodeTest.cpp
    src/graph/nodes/SetRingIdsRaysNodeTest.cpp
    src/graph/nodes/SetTimeOffsetsRaysNodeTest.cpp
    src/graph/nodes/SpatialMergePointsNodeTest.cpp
    src/graph/nodes/TemporalMergePointsNodeTest.cpp
    src/graph/nodes/TransformPointsNodeTest.cpp
    src/graph/nodes/TransformPointsNodeTest.cpp
    src/graph/nodes/TransformRaysNodeTest.cpp
    src/graph/nodes/YieldPointsNodeTest.cpp
    src/helpers/pointsTest.cpp
    src/memory/arrayChangeStreamTest.cpp
    src/memory/arrayOpsTest.cpp
    src/memory/arrayTypingTest.cpp
    src/scene/entityIdTest.cpp
    src/scene/textureTest.cpp
    src/scene/entityVelocityTest.cpp
    src/synchronization/graphAndCopyStream.cpp
    src/synchronization/graphThreadSynchronization.cpp
    src/synchronization/testKernel.cu
)


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
        src/tapeSurfaceTest.cpp
        src/features/tapeTests.cpp
    )
endif()

add_executable(RobotecGPULidar_test ${RGL_TEST_FILES})

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

if (RGL_BUILD_SNOW_EXTENSION)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../extensions/snow/test ${CMAKE_CURRENT_BINARY_DIR}/snow)
endif()

if (RGL_BUILD_ROS2_EXTENSION)
    # This allows accessing Nodes directly (as opposed to through RGL-API)
    # It is needed by EntityVelocity test, which makes a use of non-public Ros2 node.
    find_package(rclcpp REQUIRED)
    target_include_directories(RobotecGPULidar_test PRIVATE
        ${CMAKE_SOURCE_DIR}/extensions/ros2/src
        ${rclcpp_INCLUDE_DIRS}
    )
    target_link_libraries(RobotecGPULidar_test PRIVATE
        ${rclcpp_LIBRARIES}
    )
endif()

include(GoogleTest)

gtest_discover_tests(RobotecGPULidar_test)
