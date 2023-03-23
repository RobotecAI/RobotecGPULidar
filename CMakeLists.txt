cmake_minimum_required(VERSION 3.18)  # 3.18 To automatically detect CUDA_ARCHITECTURES
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CUDA_STANDARD 17)
# Build Release by default; CMAKE_BUILD_TYPE needs to be set before project(...)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel" FORCE)
endif(NOT CMAKE_BUILD_TYPE)
project(RobotecGPULidar C CXX CUDA)

# Logging default settings (can be changed via API call)
set(RGL_LOG_STDOUT ON CACHE BOOL
    "Enables logging to STDOUT")
set(RGL_LOG_LEVEL INFO CACHE STRING
    "Specifies minimal severity of log message to be printed (TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF)")
set(RGL_LOG_FILE "" CACHE STRING  # STRING prevents from expanding relative paths
    "Defines a file path to store RGL log")
set(RGL_AUTO_TAPE_PATH "" CACHE STRING  # STRING prevents from expanding relative paths
    "If non-empty, defines a path for the automatic tape (started on the first API call)")

# Test configuration
set(RGL_BUILD_TESTS ON CACHE BOOL
    "Enables building test. GTest will be automatically downloaded")

# Tools configuration
set(RGL_BUILD_TOOLS ON CACHE BOOL "Enables building RGL executable tools")

# Extensions configuration
set(RGL_BUILD_PCL_EXTENSION OFF CACHE BOOL
    "Enables building PCL extension.")
set(RGL_BUILD_ROS2_EXTENSION OFF CACHE BOOL
    "Enables building ROS2 extension. It requires installed and sourced ROS2.")
set(RGL_BUILD_ROS2_EXTENSION_STANDALONE OFF CACHE BOOL
    "Enables building ROS2 extension in standalone mode. It requires installed and sourced ROS2.")

# Hide automatically generated CTest targets
set_property(GLOBAL PROPERTY CTEST_TARGETS_ADDED 1)

# Fix Windows problems
if (WIN32)
    add_definitions(-DNOMINMAX)  # http://www.suodenjoki.dk/us/archive/2010/min-max.htm
    add_definitions(-D_USE_MATH_DEFINES)
endif()

# External dependencies
add_subdirectory(external)
find_package(CUDAToolkit REQUIRED)
find_program(BIN2C bin2c DOC "Path to the cuda-sdk bin2c executable.")
if (NOT DEFINED ENV{OptiX_INSTALL_DIR})
    message(FATAL_ERROR "Required environment variable OptiX_INSTALL_DIR is empty, aborting build")
endif()

# Includes
include_directories(include)
include_directories($ENV{OptiX_INSTALL_DIR}/include)
include_directories(${spdlog_SOURCE_DIR}/include)

# Compile OptiX device programs (pipeline) and embed the binary in a library as a char[]
add_library(optixProgramsPtx OBJECT src/gpu/optixPrograms.cu)
target_include_directories(optixProgramsPtx PRIVATE src include)
set_target_properties(optixProgramsPtx PROPERTIES CUDA_PTX_COMPILATION ON)
add_custom_command(
    OUTPUT optixProgramsPtx.c
    COMMAND ${BIN2C} -c --padd 0 --type char --name optixProgramsPtx $<TARGET_OBJECTS:optixProgramsPtx> > optixProgramsPtx.c
    DEPENDS optixProgramsPtx $<TARGET_OBJECTS:optixProgramsPtx>  # Should work with just optixProgramsPtx, but CMake..
    VERBATIM)
add_library(optixPrograms optixProgramsPtx.c)

add_library(RobotecGPULidar SHARED
    src/api/apiCommon.cpp
    src/api/apiCore.cpp
    src/Tape.cpp
    src/Logger.cpp
    src/VArray.cpp
    src/Optix.cpp
    src/gpu/gaussianNoiseKernels.cu
    src/gpu/nodeKernels.cu
    src/scene/Scene.cpp
    src/scene/Mesh.cpp
    src/scene/Entity.cpp
    src/scene/ASBuildScratchpad.cpp
    src/graph/Graph.cpp
    src/graph/Node.cpp
    src/graph/GaussianNoiseAngularHitpointNode.cpp
    src/graph/GaussianNoiseAngularRayNode.cpp
    src/graph/GaussianNoiseDistanceNode.cpp
    src/graph/CompactPointsNode.cpp
    src/graph/FormatPointsNode.cpp
    src/graph/RaytraceNode.cpp
    src/graph/TransformPointsNode.cpp
    src/graph/TransformRaysNode.cpp
    src/graph/FromArrayPointsNode.cpp
    src/graph/FromMat3x4fRaysNode.cpp
    src/graph/SetRaysRingIdsRaysNode.cpp
    src/graph/SpatialMergePointsNode.cpp
    src/graph/TemporalMergePointsNode.cpp
    src/graph/YieldPointsNode.cpp)

set_property(TARGET RobotecGPULidar PROPERTY POSITION_INDEPENDENT_CODE ON)

if (RGL_BUILD_PCL_EXTENSION)
    find_package(PCL 1.12 CONFIG REQUIRED COMPONENTS common io filters visualization)
    target_sources(RobotecGPULidar PRIVATE
        src/api/apiPcl.cpp
        src/graph/DownSamplePointsNode.cpp
        src/graph/VisualizePointsNode.cpp
    )
    target_include_directories(RobotecGPULidar PUBLIC ${PCL_INCLUDE_DIRS})
    target_link_directories(RobotecGPULidar PRIVATE ${PCL_LIBRARY_DIRS})
    target_link_libraries(RobotecGPULidar PRIVATE ${PCL_LIBRARIES})
    target_compile_definitions(RobotecGPULidar PUBLIC RGL_BUILD_PCL_EXTENSION=$<BOOL:${RGL_BUILD_PCL_EXTENSION}> ${PCL_DEFINITIONS})
endif()

if (RGL_BUILD_ROS2_EXTENSION OR RGL_BUILD_ROS2_EXTENSION_STANDALONE)
    if (NOT $ENV{ROS_DISTRO} STREQUAL "humble")
      message(FATAL_ERROR "ROS $ENV{ROS_DISTRO} not supported. Only humble is available.")
    endif()
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    target_sources(RobotecGPULidar PRIVATE src/api/apiRos2.cpp src/graph/Ros2PublishPointsNode.cpp)
    target_include_directories(RobotecGPULidar PUBLIC ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})
    target_link_libraries(RobotecGPULidar PRIVATE ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES})
    target_compile_definitions(RobotecGPULidar PUBLIC RGL_BUILD_ROS2_EXTENSION=$<BOOL:${RGL_BUILD_ROS2_EXTENSION}>)
endif()

target_include_directories(RobotecGPULidar
    PUBLIC include
    PRIVATE src
)

target_link_libraries(RobotecGPULidar PRIVATE
    spdlog
    yaml-cpp
    optixPrograms
    cmake_git_version_tracking
)

target_link_libraries(RobotecGPULidar PUBLIC
    CUDA::nvml
    CUDA::cudart_static
    CUDA::cuda_driver
)

# Create a CMake list with available log levels (rgl_log_level_t)
set(RGL_AVAILABLE_LOG_LEVELS
    RGL_LOG_LEVEL_ALL
    RGL_LOG_LEVEL_TRACE
    RGL_LOG_LEVEL_DEBUG
    RGL_LOG_LEVEL_INFO
    RGL_LOG_LEVEL_WARN
    RGL_LOG_LEVEL_ERROR
    RGL_LOG_LEVEL_CRITICAL
    RGL_LOG_LEVEL_OFF
)

# Check if RGL_LOG_LEVEL is a valid variable
if (NOT ("RGL_LOG_LEVEL_${RGL_LOG_LEVEL}" IN_LIST RGL_AVAILABLE_LOG_LEVELS))
    message(FATAL_ERROR "Incorrect RGL_LOG_LEVEL value: ${RGL_LOG_LEVEL}")
endif()

if (WIN32 AND RGL_AUTO_TAPE_PATH)
    message(FATAL_ERROR "(Auto)Tape not supported on Windows")
endif()

# Pass #define-s to RGL compilation
target_compile_definitions(RobotecGPULidar
    PUBLIC RGL_LOG_STDOUT=$<BOOL:${RGL_LOG_STDOUT}>
    PUBLIC RGL_LOG_FILE="${RGL_LOG_FILE}"
    PUBLIC RGL_LOG_LEVEL=RGL_LOG_LEVEL_${RGL_LOG_LEVEL}
    PUBLIC RGL_AUTO_TAPE_PATH="${RGL_AUTO_TAPE_PATH}"
)

# Include tests
if (RGL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# Include tools
if (RGL_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

# Include ros2_standalone
if(RGL_BUILD_ROS2_EXTENSION_STANDALONE)
    add_subdirectory(ros2_standalone)
endif()
