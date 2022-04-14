# ======================================================================== #
# Copyright 2018-2019 Ingo Wald                                            #
#                                                                          #
# Licensed under the Apache License, Version 2.0 (the "License");          #
# you may not use this file except in compliance with the License.         #
# You may obtain a copy of the License at                                  #
#                                                                          #
#     http://www.apache.org/licenses/LICENSE-2.0                           #
#                                                                          #
# Unless required by applicable law or agreed to in writing, software      #
# distributed under the License is distributed on an "AS IS" BASIS,        #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. #
# See the License for the specific language governing permissions and      #
# limitations under the License.                                           #
# ======================================================================== #

# TODO(prybicki): rewrite it to modern CMake
cmake_minimum_required(VERSION 3.16)
project(RobotecGPULidar C CXX CUDA)

set(CMAKE_CXX_STANDARD 20)

# GDT-Related stuff, to be removed in the future TODO(prybicki)
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

if (UNIX AND NOT APPLE)
    # On Linux, libfmt is expected to be installed separately (apt / compiled from sources)
    find_package(fmt REQUIRED)
else()
    # On Windows, it is easier to clone and build from sources
    add_subdirectory("fmt-8.0.1")
endif()

cuda_compile_and_embed(embedded_ptx_code src/devicePrograms.cu)

add_library(RobotecGPULidar SHARED
    ${embedded_ptx_code}
    src/LidarRenderer.cpp
    src/simple_uid_generator.cpp
    src/ModelInstance.cpp
    src/PerfProbe.cpp
    src/legacy_api.cpp
    src/rgl_tmp_api.cpp
    src/formatPCL.cu
    src/gaussianNoiseKernels.cu
)
target_link_libraries(RobotecGPULidar
    gdt
    fmt::fmt-header-only # FMT Library for a proper string formatting in C++
    ${CUDA_LIBRARIES}
    ${CUDA_CUDA_LIBRARY}
)
target_compile_options(RobotecGPULidar PRIVATE -Wall)


# -------------------
# build dotnet
# -------------------
add_subdirectory(dotnet_plugin/dotnet_cmake_module)
add_subdirectory(dotnet_plugin/gpu-lidar-raycaster-dotnet)

# ------------------------------------------------------------------
# tests
# ------------------------------------------------------------------
 if (BUILD_TESTS)
     enable_testing()
#     add_subdirectory(dotnet_plugin/gpu-lidar-raycaster-dotnet-test)
     add_subdirectory(test)
 endif()
