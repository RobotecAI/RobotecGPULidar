cmake_minimum_required(VERSION 3.16)

set(RGL_TEST_FILES
    src/graphCaseTest.cpp
    src/asyncStressTest.cpp
    src/apiReadmeExample.cpp
    src/gaussianStressTest.cpp
    src/apiSurfaceTests.cpp
    #    src/features/range.cpp
    #    src/features/mesh.cpp
    #    src/features/transforms.cpp
    #    src/features/pcdOutput.cpp
    src/testMat3x4f.cpp
    src/graph/nodes/FromArrayPointsNodeTest.cpp
    src/graph/nodes/TransformPointsNodeTest.cpp
    src/ArrayTest.cpp
    src/synchronization/syncTest.cpp
    src/synchronization/testKernel.cu)

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
