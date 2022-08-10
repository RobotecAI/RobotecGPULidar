cmake_minimum_required(VERSION 3.18)  # 3.18 To automatically detect CUDA_ARCHITECTURES
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CUDA_STANDARD 17)
project(RobotecGPULidar C CXX CUDA)

# Logging default settings (can be changed via API call)
set(RGL_LOG_STDOUT ON CACHE BOOL
    "Enables logging to STDOUT")
set(RGL_LOG_LEVEL INFO CACHE STRING
    "Specifies minimal severity of log message to be printed (TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF)")
set(RGL_LOG_FILE "" CACHE PATH
    "Defines a file path to store RGL log")

# Test configuration
set(RGL_BUILD_TESTS ON CACHE BOOL
    "Enables building test. GTest will be automatically downloaded")

# Build Release by default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Fix Windows problems
if (WIN32)
    add_definitions(-DNOMINMAX)  # http://www.suodenjoki.dk/us/archive/2010/min-max.htm
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

# Compile OptiX device programs (pipeline) and embed the binary in a library as a char[]
add_library(optixProgramsPtx OBJECT src/optixPrograms.cu)
set_target_properties(optixProgramsPtx PROPERTIES CUDA_PTX_COMPILATION ON)
add_custom_command(
    OUTPUT optixProgramsPtx.c
    COMMAND ${BIN2C} -c --padd 0 --type char --name optixProgramsPtx $<TARGET_OBJECTS:optixProgramsPtx> > optixProgramsPtx.c
    DEPENDS optixProgramsPtx $<TARGET_OBJECTS:optixProgramsPtx>  # Should work with just optixProgramsPtx, but CMake..
    VERBATIM)
add_library(optixPrograms optixProgramsPtx.c)

add_library(RobotecGPULidar SHARED
    src/api/api.cpp
    src/Logger.cpp
#    src/Lidar.cpp
#    src/formatPCL.cu
    src/Optix.cpp
    src/scene/Scene.cpp
    src/scene/Mesh.cpp
    src/scene/Entity.cpp
    src/scene/ASBuildScratchpad.cpp
#    src/gaussianNoiseKernels.cu
)

target_include_directories(RobotecGPULidar PUBLIC include)

target_link_libraries(RobotecGPULidar PUBLIC
    spdlog
    optixPrograms
    CUDA::nvml
    CUDA::cudart
    CUDA::cuda_driver
)

# Pass #define-s to RGL compilation
target_compile_definitions(RobotecGPULidar
    PUBLIC RGL_LOG_STDOUT=$<BOOL:${RGL_LOG_STDOUT}>
    PUBLIC RGL_LOG_FILE={"${RGL_LOG_FILE}"}
    PUBLIC RGL_LOG_LEVEL=RGL_LOG_LEVEL_${RGL_LOG_LEVEL}
)

# Include tests
if (${RGL_BUILD_TESTS})
    enable_testing()
    add_subdirectory(test)
endif()
