cmake_minimum_required(VERSION 3.8)

project(gpu-lidar-raycaster-dotnet-test)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CSHARP_TARGET_FRAMEWORK "netcoreapp2.0")

#TODO - this is messy since it is basically a gutted ament_package to remove deps
set(dotnet_cmake_module_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../dotnet_cmake_module/cmake)
set(CMAKE_MODULE_PATH "${dotnet_cmake_module_DIR}/Modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${dotnet_cmake_module_DIR}/Modules/dotnet/" ${CMAKE_MODULE_PATH})
include(${dotnet_cmake_module_DIR}/Modules/dotnet/UseCSharpProjectBuilder.cmake)
find_package(DotNETExtra REQUIRED)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

message("${CMAKE_CURRENT_BINARY_DIR}/../gpu_lidar_raycaster_dotnet/gpu_lidar_raycaster_dotnet.dll")

add_dotnet_executable(SimpleModelTest
  src/SimpleModelTest.cs
  INCLUDE_DLLS
  "${CMAKE_CURRENT_BINARY_DIR}/../gpu-lidar-raycaster-dotnet/gpu_lidar_raycaster_dotnet.dll"
)

install_dotnet(SimpleModelTest DESTINATION lib/SimpleModelTest/dotnet)
