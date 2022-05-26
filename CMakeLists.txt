cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CUDA_STANDARD 17)
project(RobotecGPULidar C CXX CUDA)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

add_subdirectory(external)
find_package(FMT)

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
    src/Lidar.cpp
    src/formatPCL.cu
    src/Optix.cpp
    src/scene/Scene.cpp
    src/scene/Mesh.cpp
    src/scene/Entity.cpp
    src/scene/ASBuildScratchpad.cpp
    src/gaussianNoiseKernels.cu
    )

target_link_libraries(RobotecGPULidar
    gdt
    fmt::fmt-header-only
    ${CUDA_LIBRARIES}
    ${CUDA_CUDA_LIBRARY}
    )

if (BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif ()
