add_executable(tapePlayer tapePlayer.cpp)
target_link_libraries(tapePlayer RobotecGPULidar spdlog)

if (RGL_BUILD_PCL_EXTENSION)
    add_executable(tapeVisualizer tapeVisualizer.cpp)
    target_link_libraries(tapeVisualizer RobotecGPULidar spdlog yaml-cpp)
    target_include_directories(tapeVisualizer PRIVATE ${CMAKE_SOURCE_DIR}/src)
endif()
