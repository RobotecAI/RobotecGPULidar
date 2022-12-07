add_executable(tapePlayer tapePlayer.cpp)
target_link_libraries(tapePlayer RobotecGPULidar fmt)

add_executable(tapeVisualizer tapeVisualizer.cpp)
target_link_libraries(tapeVisualizer RobotecGPULidar fmt)
target_include_directories(tapeVisualizer PRIVATE ${CMAKE_SOURCE_DIR}/src)
