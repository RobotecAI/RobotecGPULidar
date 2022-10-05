cmake_minimum_required(VERSION 3.16)
project(RobotecGPULidar-test C CXX)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

cuda_compile_and_embed(helloTestCU src/helloTest/helloTest.cu)

add_executable(RobotecGPULidar_test
#        ${helloTestCU}
        src/api_example.cpp
#        src/linearAlgebra_test.cpp
#        src/integration_test.cpp
#        src/gaussianNoise_test.cpp
#        src/utils/statistical_utils.cpp
)
target_link_libraries(RobotecGPULidar_test
        gdt
        gtest_main
        fmt::fmt # FMT Library for a proper string formatting in C++
        ${CUDA_LIBRARIES}
        ${CUDA_CUDA_LIBRARY}
        RobotecGPULidar
)

set (model_files
    ${CMAKE_CURRENT_SOURCE_DIR}/models/cube.obj
    ${CMAKE_CURRENT_SOURCE_DIR}/models/cube.mtl
)

#file(COPY ${model_files} DESTINATION ${CMAKE_BINARY_DIR})

include(GoogleTest)
gtest_discover_tests(RobotecGPULidar_test)
