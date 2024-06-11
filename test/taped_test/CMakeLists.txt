cmake_minimum_required(VERSION 3.18) # # 3.18 To have DISCOVERY_MODE option for gtest_discover_tests()

if(WIN32)
    message(FATAL_ERROR "Tape not supported on Windows")
endif()

set(RGL_TAPED_TEST_FILES
    src/AwsimMeshToPcdTest.cpp
)

include(GoogleTest)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/test)
add_executable(RobotecGPULidar_taped_test ${RGL_TAPED_TEST_FILES})

target_link_libraries(RobotecGPULidar_taped_test PRIVATE
        gtest_main
        gmock_main
        spdlog
        RobotecGPULidar
)

target_include_directories(RobotecGPULidar_taped_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

# Set $ORIGIN rpath to search for dependencies in the executable location (Linux only)
set_target_properties(RobotecGPULidar_taped_test PROPERTIES LINK_FLAGS "-Wl,-rpath,$ORIGIN")

gtest_discover_tests(RobotecGPULidar_taped_test
        DISCOVERY_MODE PRE_TEST)
