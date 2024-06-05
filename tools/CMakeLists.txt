set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/tools)
add_executable(inspectLibRGL inspectLibRGL.cpp)

# Linux only - tape related tools
if ((NOT WIN32))
    if (RGL_BUILD_PCL_EXTENSION)
        add_executable(tapeVisualizer tapeVisualizer.cpp)
        target_link_libraries(tapeVisualizer RobotecGPULidar spdlog yaml-cpp)
        target_include_directories(tapeVisualizer PRIVATE ${CMAKE_SOURCE_DIR}/src)
        # Set $ORIGIN rpath to search for dependencies in the executable location (Linux only)
        set_target_properties(tapeVisualizer PROPERTIES LINK_FLAGS "-Wl,-rpath,$ORIGIN")
    endif()

    add_executable(tapePlayer tapePlayer.cpp)
    target_link_libraries(tapePlayer RobotecGPULidar spdlog)
    target_include_directories(tapePlayer PRIVATE ${CMAKE_SOURCE_DIR}/src)
    # Set $ORIGIN rpath to search for dependencies in the executable location (Linux only)
    set_target_properties(tapePlayer PROPERTIES LINK_FLAGS "-Wl,-rpath,$ORIGIN")
endif()
