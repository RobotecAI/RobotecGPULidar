cmake_minimum_required(VERSION 3.16)

add_executable(RobotecGPULidar_test
    src/apiReadmeExample.cpp
    src/apiUnitTests.cpp
    src/features/range.cpp
    src/features/gaussianNoise.cpp
    src/features/mesh.cpp
    src/features/transforms.cpp
    src/utils/testUtils.cpp
)

target_link_libraries(RobotecGPULidar_test
    gtest_main
    gmock_main
    RobotecGPULidar
)
target_include_directories(RobotecGPULidar_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

include(GoogleTest)
gtest_discover_tests(RobotecGPULidar_test)
