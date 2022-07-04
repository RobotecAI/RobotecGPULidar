cmake_minimum_required(VERSION 3.16)

add_executable(RobotecGPULidar_test
    src/unit/api/experimental.cpp
    src/unit/api/e2e_extentions.cpp
    src/integration/laser/range.cpp
    src/integration/laser/gaussianNoise.cpp
#   src/apiExample.cpp
#   src/apiUseCases.cpp
#   src/linearAlgebra_test.cpp
#   src/integration_test.cpp
#   src/gaussianNoise_test.cpp
    src/utils/statistical_utils.cpp
)

target_link_libraries(RobotecGPULidar_test
    gtest_main
    gmock_main
    RobotecGPULidar
)
target_include_directories(RobotecGPULidar_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

include(GoogleTest)
gtest_discover_tests(RobotecGPULidar_test)
