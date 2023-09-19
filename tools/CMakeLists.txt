add_executable(inspectLibRGL inspectLibRGL.cpp)

# Linux only - tape related tools
if ((NOT WIN32))
    if (RGL_BUILD_PCL_EXTENSION)
        add_executable(tapeVisualizer tapeVisualizer.cpp)
        target_link_libraries(tapeVisualizer RobotecGPULidar spdlog yaml-cpp)
        target_include_directories(tapeVisualizer PRIVATE ${CMAKE_SOURCE_DIR}/src)
    endif()

    add_executable(tapePlayer tapePlayer.cpp)
    target_link_libraries(tapePlayer RobotecGPULidar spdlog)
    target_include_directories(tapePlayer PRIVATE ${CMAKE_SOURCE_DIR}/src)
endif()
