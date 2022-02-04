cmake_minimum_required(VERSION 3.8)

project(gpu_lidar_raycaster_dotnet C)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

function(configure_csharp_c_extension_library _library_name)
  install(TARGETS ${_library_name} EXPORT ${_library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endfunction()

set(CSHARP_TARGET_FRAMEWORK "netcoreapp2.0")

#TODO - this is messy since it is basically a gutted ament_package to remove deps
set(dotnet_cmake_module_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../dotnet_cmake_module/cmake)
set(CMAKE_MODULE_PATH "${dotnet_cmake_module_DIR}/Modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${dotnet_cmake_module_DIR}/Modules/dotnet/" ${CMAKE_MODULE_PATH})
include(${dotnet_cmake_module_DIR}/Modules/dotnet/UseCSharpProjectBuilder.cmake)
find_package(DotNETExtra REQUIRED)

configure_csharp_c_extension_library(RobotecGPULidar)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

set(CS_SOURCES
  properties/AssemblyInfo.cs
  src/Raycaster.cs
  src/LidarSource.cs
  src/Mesh.cs
  src/Texture.cs
  src/Primitives.cs
  src/RaycastResult.cs
  src/DllLoadUtils.cs
  src/NativeMethods.cs
)

add_dotnet_library(${PROJECT_NAME}
SOURCES
  ${CS_SOURCES}
  INCLUDE_DLLS
  ${_assembly_deps_dll}
)

#TODO
add_custom_command(
  TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/netcoreapp2.0/linux-x64/publish/${PROJECT_NAME}.dll
          ${CMAKE_CURRENT_BINARY_DIR}/)

install_dotnet(${PROJECT_NAME} DESTINATION lib/dotnet)
