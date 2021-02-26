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

function(configure_csharp_c_extension_library _library_name)
  install(TARGETS ${_library_name} EXPORT ${_library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endfunction()

add_library(
  native_model_loader SHARED
  native/native_model_loader.cpp
)
target_link_libraries(native_model_loader optix_lidar)
target_include_directories(native_model_loader PUBLIC ${optix_lidar_INCLUDE_DIRS})
configure_csharp_c_extension_library(native_model_loader)

add_dotnet_executable(SimpleModelTest
src/SimpleModelTest.cs
src/ModelLoader.cs
INCLUDE_DLLS
"${CMAKE_CURRENT_BINARY_DIR}/../gpu-lidar-raycaster-dotnet/gpu_lidar_raycaster_dotnet.dll"
)

configure_file(${CMAKE_CURRENT_LIST_DIR}/models/test.obj ${CMAKE_CURRENT_BINARY_DIR}/Release/netcoreapp2.0/linux-x64/test.obj COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/models/test.mtl ${CMAKE_CURRENT_BINARY_DIR}/Release/netcoreapp2.0/linux-x64/test.mtl COPYONLY)


set_target_properties(SimpleModelTest PROPERTIES
    VS_DOTNET_REFERENCE_gpu_lidar_raycaster_dotnet /home/pjaroszek/projects/robotec/lidar-raycasting-gpu/build/dotnet_plugin/gpu-lidar-raycaster-dotnet/gpu_lidar_raycaster_dotnet.dll)

install_dotnet(SimpleModelTest DESTINATION lib/SimpleModelTest/dotnet)
