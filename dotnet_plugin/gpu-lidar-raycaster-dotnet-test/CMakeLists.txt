cmake_minimum_required(VERSION 3.8)

project(gpu-lidar-raycaster-dotnet-test)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# This is tricky: 'net5.0' and 'netcoreapp2.1' works out of box on ubuntu 20 (but needs
# to be installed in the system), 'netcoreapp2.0' does not because of invalid (too new) 
# openssl version in ubuntu 19+.
# Download links: https://dotnet.microsoft.com/download/dotnet/
# About ssl issue: https://github.com/OmniSharp/omnisharp-vscode/issues/3010
set(CSHARP_TARGET_FRAMEWORK "net5.0")

#TODO - this is messy since it is basically a gutted ament_package to remove deps
set(dotnet_cmake_module_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../dotnet_cmake_module/cmake)
set(CMAKE_MODULE_PATH "${dotnet_cmake_module_DIR}/Modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${dotnet_cmake_module_DIR}/Modules/dotnet/" ${CMAKE_MODULE_PATH})
include(${dotnet_cmake_module_DIR}/Modules/dotnet/UseCSharpProjectBuilder.cmake)
find_package(DotNETExtra REQUIRED)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

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
target_link_libraries(native_model_loader RobotecGPULidar)
target_include_directories(native_model_loader PUBLIC ${RobotecGPULidar_INCLUDE_DIRS})
configure_csharp_c_extension_library(native_model_loader)

add_dotnet_test(GPULidarTest
src/GPULidarTest.cs
src/ModelLoader.cs
INCLUDE_DLLS
"${CMAKE_CURRENT_BINARY_DIR}/../gpu-lidar-raycaster-dotnet/gpu_lidar_raycaster_dotnet.dll"
)

add_dotnet_test(ModelLoaderTest
src/ModelLoaderTest.cs
src/ModelLoader.cs
INCLUDE_DLLS
"${CMAKE_CURRENT_BINARY_DIR}/../gpu-lidar-raycaster-dotnet/gpu_lidar_raycaster_dotnet.dll"
)

set(TEST_MODELS 
  2cubes 
  cube 
  test
  inverted_sphere
  cylinder_stand
)
foreach(test_model ${TEST_MODELS})
  configure_file(${CMAKE_CURRENT_LIST_DIR}/models/${test_model}.obj ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/${CSHARP_TARGET_FRAMEWORK}/linux-x64/${test_model}.obj COPYONLY)
  configure_file(${CMAKE_CURRENT_LIST_DIR}/models/${test_model}.mtl ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/${CSHARP_TARGET_FRAMEWORK}/linux-x64/${test_model}.mtl COPYONLY)
  configure_file(${CMAKE_CURRENT_LIST_DIR}/models/${test_model}.obj ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/${CSHARP_TARGET_FRAMEWORK}/${test_model}.obj COPYONLY)
  configure_file(${CMAKE_CURRENT_LIST_DIR}/models/${test_model}.mtl ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/${CSHARP_TARGET_FRAMEWORK}/${test_model}.mtl COPYONLY)
endforeach(test_model)

install_dotnet(GPULidarTest DESTINATION lib/GPULidarTest/dotnet)
