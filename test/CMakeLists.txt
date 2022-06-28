cmake_minimum_required(VERSION 3.16)



cuda_compile_and_embed(helloTestCU src/helloTest/helloTest.cu)

add_executable(RobotecGPULidar_test
    ${helloTestCU}
#    src/apiExample.cpp
#    src/apiUnitTests.cpp
    src/apiUseCases.cpp
#   src/linearAlgebra_test.cpp
#   src/integration_test.cpp
#   src/gaussianNoise_test.cpp
#   src/utils/statistical_utils.cpp
)

target_link_libraries(RobotecGPULidar_test
    gtest_main
    gmock_main
    RobotecGPULidar
)
target_include_directories(RobotecGPULidar_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

include(GoogleTest)
gtest_discover_tests(RobotecGPULidar_test)
