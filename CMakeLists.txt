cmake_minimum_required(VERSION 3.18)  # 3.18 To automatically detect CUDA_ARCHITECTURES
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CUDA_STANDARD 17)

# Build Release by default; CMAKE_BUILD_TYPE needs to be set before project(...)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel" FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Use debug postfix 'd' by default.
if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING
      "Choose the debug postfix used when building Debug configuration")
endif()

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

# Library configuration
set(RGL_BUILD_STATIC OFF CACHE BOOL
    "Builds RobotecGPULidar as a statically linkable library instead of a shared one")

# Test configuration
set(RGL_BUILD_TESTS ON CACHE BOOL
    "Enables building test. GTest will be automatically downloaded")
set(RGL_BUILD_TAPED_TESTS OFF CACHE BOOL
    "Enables building taped test.")

# Tools configuration
set(RGL_BUILD_TOOLS ON CACHE BOOL "Enables building RGL executable tools")

# Extensions configuration
set(RGL_BUILD_PCL_EXTENSION OFF CACHE BOOL
    "Enables building PCL extension.")
set(RGL_BUILD_ROS2_EXTENSION OFF CACHE BOOL
    "Enables building ROS2 extension. It requires installed and sourced ROS2.")
set(RGL_BUILD_UDP_EXTENSION OFF CACHE BOOL
    "Enables building UDP extension.")
set(RGL_BUILD_SNOW_EXTENSION OFF CACHE BOOL
    "Enables building snow simulation extension.")


# Hide automatically generated CTest targets
set_property(GLOBAL PROPERTY CTEST_TARGETS_ADDED 1)

# Fix Windows problems
if (WIN32)
    add_definitions(-DNOMINMAX)  # http://www.suodenjoki.dk/us/archive/2010/min-max.htm
    add_definitions(-D_USE_MATH_DEFINES)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)  # https://www.kitware.com//create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
endif()

if (WIN32 AND RGL_BUILD_TESTS)
    # Sometimes, for convenience, tests reference non-public symbols from the library
    # By default, those symbols are not visible and not annotated with __declspec(dllexport)
    # However, there's a CMake workaround for such scenarios, described further at
    # https://www.kitware.com//create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

# `External` dependencies
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
include_directories(${YAML_CPP_SOURCE_DIR}/include)

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

set(RGL_SOURCE_FILES
    src/api/apiCommon.cpp
    src/api/apiCore.cpp
    src/tape/TapePlayer.cpp
    src/tape/TapeRecorder.cpp
    src/tape/PlaybackState.cpp
    src/Logger.cpp
    src/Optix.cpp
    src/gpu/helpersKernels.cu
    src/gpu/gaussianNoiseKernels.cu
    src/gpu/nodeKernels.cu
    src/scene/Scene.cpp
    src/scene/Mesh.cpp
    src/scene/Entity.cpp
    src/scene/Texture.cpp
    src/scene/ASBuildScratchpad.cpp
    src/graph/GraphRunCtx.cpp
    src/graph/Node.cpp
    src/graph/GaussianNoiseAngularHitpointNode.cpp
    src/graph/GaussianNoiseAngularRayNode.cpp
    src/graph/GaussianNoiseDistanceNode.cpp
    src/graph/CompactByFieldPointsNode.cpp
    src/graph/FormatPointsNode.cpp
    src/graph/RaytraceNode.cpp
    src/graph/TransformPointsNode.cpp
    src/graph/TransformRaysNode.cpp
    src/graph/FromArrayPointsNode.cpp
    src/graph/FromMat3x4fRaysNode.cpp
    src/graph/FilterGroundPointsNode.cpp
    src/graph/RadarPostprocessPointsNode.cpp
    src/graph/RadarTrackObjectsNode.cpp
    src/graph/SetRangeRaysNode.cpp
    src/graph/SetRaysRingIdsRaysNode.cpp
    src/graph/SetTimeOffsetsRaysNode.cpp
    src/graph/SpatialMergePointsNode.cpp
    src/graph/TemporalMergePointsNode.cpp
    src/graph/YieldPointsNode.cpp
)

if (RGL_BUILD_STATIC)
    add_library(RobotecGPULidar STATIC ${RGL_SOURCE_FILES})
    target_compile_definitions(RobotecGPULidar PUBLIC RGL_STATIC)
    set(RGL_SPDLOG_VARIANT spdlog_header_only)
else ()
    add_library(RobotecGPULidar SHARED ${RGL_SOURCE_FILES})
    set(RGL_SPDLOG_VARIANT spdlog)
endif ()

set_property(TARGET RobotecGPULidar PROPERTY POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(RobotecGPULidar PUBLIC RGL_BUILD_PCL_EXTENSION=$<BOOL:${RGL_BUILD_PCL_EXTENSION}>)
target_compile_definitions(RobotecGPULidar PUBLIC RGL_BUILD_ROS2_EXTENSION=$<BOOL:${RGL_BUILD_ROS2_EXTENSION}>)
target_compile_definitions(RobotecGPULidar PUBLIC RGL_BUILD_UDP_EXTENSION=$<BOOL:${RGL_BUILD_UDP_EXTENSION}>)
target_compile_definitions(RobotecGPULidar PUBLIC RGL_BUILD_SNOW_EXTENSION=$<BOOL:${RGL_BUILD_SNOW_EXTENSION}>)

if (RGL_BUILD_PCL_EXTENSION)
    add_subdirectory(extensions/pcl)
endif()

if (RGL_BUILD_ROS2_EXTENSION)
    add_subdirectory(extensions/ros2)
endif()

if (RGL_BUILD_UDP_EXTENSION)
    add_subdirectory(extensions/udp)
endif()

if (RGL_BUILD_SNOW_EXTENSION)
    add_subdirectory(extensions/snow)
endif()

target_include_directories(RobotecGPULidar
    PUBLIC include
    PRIVATE src
)

target_link_libraries(RobotecGPULidar PRIVATE
    ${RGL_SPDLOG_VARIANT}
    yaml-cpp
    optixPrograms
    cmake_git_version_tracking
)

target_link_libraries(RobotecGPULidar PUBLIC
    CUDA::nvml
    CUDA::cudart_static
    CUDA::cuda_driver
    CUDA::nvToolsExt
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

if (WIN32 AND (RGL_AUTO_TAPE_PATH OR RGL_BUILD_TAPED_TESTS))
    message(FATAL_ERROR "(Auto)Tape not supported on Windows")
endif()

# Pass #define-s to RGL compilation
target_compile_definitions(RobotecGPULidar
    PUBLIC RGL_LOG_STDOUT=$<BOOL:${RGL_LOG_STDOUT}>
    PUBLIC RGL_LOG_FILE="${RGL_LOG_FILE}"
    PUBLIC RGL_LOG_LEVEL=RGL_LOG_LEVEL_${RGL_LOG_LEVEL}
    PUBLIC RGL_AUTO_TAPE_PATH="${RGL_AUTO_TAPE_PATH}"
    PRIVATE RGL_BUILD # Used in headers to differentiate whether it is parsed as library or client's code, affects __declspec on Windows.
)

# Include tests
if (RGL_BUILD_TESTS OR RGL_BUILD_TAPED_TESTS)
    enable_testing()

    if (RGL_BUILD_TESTS)
        add_subdirectory(test)
    endif()

    if (RGL_BUILD_TAPED_TESTS)
        add_subdirectory(test/taped_test)
    endif()
endif()


# Include tools
if (RGL_BUILD_TOOLS)
    add_subdirectory(tools)
endif()
