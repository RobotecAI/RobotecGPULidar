cmake_minimum_required(VERSION 3.17)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_ARCHITECTURES 75-virtual)
project(RobotecGPULidar C CXX CUDA)


# Logging default settings (can be changed via API call)
set(RGL_LOG_STDOUT ON CACHE BOOL
    "Enables logging to STDOUT")
set(RGL_LOG_LEVEL INFO CACHE STRING
    "Specifies minimal severity of log message to be printed (TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF)")
set(RGL_LOG_FILE "" CACHE PATH
    "Defines a file path to store RGL log")

set(RGL_BUILD_TESTS ON CACHE BOOL
    "Enables building test. GTest will be automatically downloaded")


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

add_subdirectory(external)
find_package(CUDAToolkit REQUIRED)

# GDT-related stuff, to be removed in the future
set(gdt_dir ${PROJECT_SOURCE_DIR}/common/gdt/)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${gdt_dir}/cmake/")
include(${gdt_dir}/cmake/configure_build_type.cmake)
include(${gdt_dir}/cmake/configure_optix.cmake)
include_directories(common)
include_directories(${gdt_dir})
add_subdirectory(${gdt_dir} EXCLUDE_FROM_ALL)
mark_as_advanced(CUDA_SDK_ROOT_DIR)

# Project files
include_directories(include)
include_directories(${OptiX_INCLUDE})
cuda_compile_and_embed(embedded_ptx_code src/devicePrograms.cu)

add_library(RobotecGPULidar SHARED
    ${embedded_ptx_code}
    src/api/api.cpp
    src/Logger.cpp
    src/Lidar.cpp
    src/formatPCL.cu
    src/Optix.cpp
    src/scene/Scene.cpp
    src/scene/Mesh.cpp
    src/scene/Entity.cpp
    src/scene/ASBuildScratchpad.cpp
    src/gaussianNoiseKernels.cu
)

target_link_libraries(RobotecGPULidar PUBLIC
    gdt
    spdlog
    CUDA::nvml
    CUDA::cudart
    CUDA::cuda_driver
)

target_compile_definitions(RobotecGPULidar
    PUBLIC RGL_LOG_STDOUT=$<BOOL:${RGL_LOG_STDOUT}>
    PUBLIC RGL_LOG_FILE={"${RGL_LOG_FILE}"}
    PUBLIC RGL_LOG_LEVEL=RGL_LOG_LEVEL_${RGL_LOG_LEVEL}
)

if (${RGL_BUILD_TESTS})
    enable_testing()
    add_subdirectory(test)
endif()
