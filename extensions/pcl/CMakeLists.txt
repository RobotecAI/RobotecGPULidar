
find_package(PCL 1.12 CONFIG REQUIRED COMPONENTS common io filters segmentation visualization)

target_sources(RobotecGPULidar PRIVATE
    src/api/apiPcl.cpp
    src/graph/DownSamplePointsNode.cpp
    src/graph/VisualizePointsNode.cpp
    src/graph/RemoveGroundPointsNode.cpp
)

target_include_directories(RobotecGPULidar PUBLIC ${PCL_INCLUDE_DIRS})
target_link_directories(RobotecGPULidar PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(RobotecGPULidar PRIVATE ${PCL_LIBRARIES})

target_include_directories(RobotecGPULidar
    PUBLIC include
    PRIVATE src
)
