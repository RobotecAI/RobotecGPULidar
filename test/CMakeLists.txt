cmake_minimum_required(VERSION 3.16)

set(RGL_TEST_FILES
    src/graphTest.cpp
    src/apiReadmeExample.cpp
#    src/VArrayTest.cpp
#    src/apiSurfaceTests.cpp
#    src/features/range.cpp
#    src/features/gaussianNoise.cpp
#    src/features/mesh.cpp
#    src/features/transforms.cpp
#    src/features/pcdOutput.cpp
    src/testMat3x4f.cpp
)

if (NOT WIN32)
    set(RGL_TEST_FILES ${RGL_TEST_FILES}
        src/tapeSurfaceTest.cpp
        src/features/tapeScene.cpp
        src/VArrayTest.cpp # doesn't work on windows
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
)

include(GoogleTest)

gtest_discover_tests(RobotecGPULidar_test)
