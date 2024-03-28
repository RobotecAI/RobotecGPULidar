cmake_minimum_required(VERSION 3.16)

if(WIN32)
    message(FATAL_ERROR "Tape not supported on Windows")
endif()

set(RGL_TAPED_TEST_FILES
    src/AwsimMeshToPcdTest.cpp
)

include(GoogleTest)

add_executable(RobotecGPULidar_taped_test ${RGL_TAPED_TEST_FILES})

find_package(GTest REQUIRED)

target_link_libraries(RobotecGPULidar_taped_test PRIVATE
        GTest::gtest_main
        GTest::gmock_main
        spdlog
        RobotecGPULidar
)

target_include_directories(RobotecGPULidar_taped_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

gtest_discover_tests(RobotecGPULidar_taped_test)