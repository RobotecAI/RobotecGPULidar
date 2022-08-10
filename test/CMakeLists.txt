cmake_minimum_required(VERSION 3.16)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(RobotecGPULidar_test
    src/pipelineTest.cpp
#    src/apiReadmeExample.cpp
#    src/apiSurfaceTests.cpp
#    src/features/range.cpp
#    src/features/gaussianNoise.cpp
#    src/features/mesh.cpp
#    src/features/transforms.cpp
#    src/features/pcdOutput.cpp
#    src/testMat3x4f.cpp
)

target_link_libraries(RobotecGPULidar_test
    gtest_main
    gmock_main
    RobotecGPULidar
    ${PCL_LIBRARIES}
)
target_include_directories(RobotecGPULidar_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

include(GoogleTest)
gtest_discover_tests(RobotecGPULidar_test)
