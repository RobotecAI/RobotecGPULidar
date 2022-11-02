cmake_minimum_required(VERSION 3.16)

add_executable(RobotecGPULidar_test
    src/graphTest.cpp
    src/apiReadmeExample.cpp
    src/VArrayTest.cpp
#    src/apiSurfaceTests.cpp
#    src/features/range.cpp
#    src/features/gaussianNoise.cpp
#    src/features/mesh.cpp
#    src/features/transforms.cpp
#    src/features/pcdOutput.cpp
    src/testMat3x4f.cpp
)
target_link_libraries(RobotecGPULidar_test
    gtest_main
    gmock_main
    RobotecGPULidar
)
target_include_directories(RobotecGPULidar_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

include(GoogleTest)
gtest_discover_tests(RobotecGPULidar_test)

#find_package(PCL 1.12 CONFIG REQUIRED COMPONENTS visualization)
add_executable(interactiveTestAWSIM src/interactive/awsim.cpp)
message("LIBS: ${PCL_LIBRARIES}")

target_link_libraries(interactiveTestAWSIM RobotecGPULidar
    gtest_main
    gmock_main
    ${PCL_LIBRARIES}
)
target_include_directories(interactiveTestAWSIM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
