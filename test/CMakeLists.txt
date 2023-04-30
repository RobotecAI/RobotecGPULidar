cmake_minimum_required(VERSION 3.16)

set(RGL_TEST_FILES
    src/graphCaseTest.cpp
    src/asyncStressTest.cpp
    src/apiReadmeExample.cpp
    src/gaussianStressTest.cpp
    src/apiSurfaceTests.cpp
    src/testMat3x4f.cpp
    src/graph/nodes/FromArrayPointsNodeTest.cpp
    src/graph/nodes/TransformPointsNodeTest.cpp
    src/graph/nodes/SetRingIdsRaysNodeTest.cpp
    src/graph/nodes/CompactPointsNodeTest.cpp
    src/graph/nodes/FormatPointsNodeTest.cpp
    src/graph/nodes/YieldPointsNodeTest.cpp
    src/graph/nodes/GaussianNoiseAngularRayNodeTest.cpp
    src/graph/nodes/GaussianNoiseDistanceNodeTest.cpp
    src/graph/nodes/GaussianNoiseAngularHitpointNodeTest.cpp
    src/graph/nodes/RaytraceNodeTest.cpp
    src/graph/nodes/TransformRaysNodeTest.cpp
    src/graph/nodes/FromMat3x4fRaysNodeTest.cpp
    src/graph/nodes/SpatialMergePointsNodeTest.cpp
    src/graph/nodes/TemporalMergePointsNodeTest.cpp
    src/graph/addChildTest.cpp
    src/memory/arrayTest.cpp
    src/memory/arrayChangeStreamTest.cpp
    src/synchronization/graphAndCopyStream.cpp
    src/synchronization/testKernel.cu
    src/synchronization/graphThreadSynchronization.cpp
)

# Only Linux
if ((NOT WIN32))
    list(APPEND RGL_TEST_FILES
        src/loggingTests.cpp
        src/VArrayTest.cpp
    )
endif()

# On Windows, tape is not available since it uses Linux sys-calls (mmap)
# AutoTape interferes with tape tests (e.g. double rgl_tape_record_begin()) which is non-trivial to fix.
if ((NOT WIN32) AND (NOT RGL_AUTO_TAPE_PATH))
    list(APPEND RGL_TEST_FILES
        src/tapeSurfaceTest.cpp
        src/features/tapeScene.cpp
    )
endif()

add_executable(RobotecGPULidar_test ${RGL_TEST_FILES})

target_link_libraries(RobotecGPULidar_test
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

include(GoogleTest)

gtest_discover_tests(RobotecGPULidar_test)
