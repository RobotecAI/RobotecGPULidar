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
set(RGL_BUILD_ROS2_EXTENSION ON CACHE BOOL
    "Enables building ROS2 extension. It requires installed and sourced ROS2.")
set(RGL_BUILD_ROS2_EXTENSION_STANDALONE ON CACHE BOOL
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

find_package(PCL 1.12 CONFIG REQUIRED COMPONENTS common io filters visualization)

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
    src/gpu/Optix.cpp
    src/gpu/nodeKernels.cu
    src/scene/Scene.cpp
    src/scene/Mesh.cpp
    src/scene/Entity.cpp
    src/scene/ASBuildScratchpad.cpp
    src/graph/graph.cpp
    src/graph/Node.cpp
    src/graph/CompactPointsNode.cpp
    src/graph/DownSamplePointsNode.cpp
    src/graph/FormatPointsNode.cpp
    src/graph/RaytraceNode.cpp
    src/graph/TransformPointsNode.cpp
    src/graph/TransformRaysNode.cpp
    src/graph/FromMat3x4fRaysNode.cpp
    src/graph/SetRaysRingIdsRaysNode.cpp
    src/graph/WritePCDFilePointsNode.cpp
    src/graph/VisualizePointsNode.cpp
    src/graph/YieldPointsNode.cpp
)

set_property(TARGET RobotecGPULidar PROPERTY POSITION_INDEPENDENT_CODE ON)

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
    PRIVATE src ${PCL_INCLUDE_DIRS}
)

target_link_directories(RobotecGPULidar PRIVATE
    ${PCL_LIBRARY_DIRS}
)

target_link_libraries(RobotecGPULidar PRIVATE
    spdlog
    yaml-cpp
    optixPrograms
    cmake_git_version_tracking
    ${PCL_LIBRARIES}
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
    PUBLIC ${PCL_DEFINITIONS}
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

if(RGL_BUILD_ROS2_EXTENSION_STANDALONE)
  find_program(PATCHELF "patchelf")
  if(UNIX)
    if(NOT PATCHELF)
      message(FATAL_ERROR "'patchelf' executable not found. Linux standalone version requires 'patchelf' for patching 'runpath' of certain libraries. You can install it via 'sudo apt install patchelf'")
    else()
      message("'patchelf' found in ${PATCHELF}")
    endif()
  endif()
  set(REQ_STANDALONE_LIBS "")
  set(REQ_STANDALONE_DLLS "")

  macro(fetch_target_lib _target)
    string(REGEX REPLACE "::" "_" _target_normalized ${_target})
    set(_locations IMPORTED_LOCATION_NONE IMPORTED_LOCATION_NOCONFIG IMPORTED_LOCATION_RELEASE IMPORTED_LOCATION_RELWITHDEBINFO IMPORTED_LOCATION_DEBUG)
    foreach(_location ${_locations})
      get_target_property(${_target_normalized}_LIB_PATH ${_target} ${_location})
      if(NOT "${${_target_normalized}_LIB_PATH}" STREQUAL "${_target_normalized}_LIB_PATH-NOTFOUND")
        break()
      endif()
    endforeach()
  endmacro()

  macro(get_standalone_dependencies _library_name)
    find_package(${_library_name} REQUIRED)

    # Get cyclonedds DDSC
    if("${_library_name}" STREQUAL "CycloneDDS")
      if(WIN32)
        fetch_target_lib(CycloneDDS::ddsc)
        fetch_target_lib(CycloneDDS::idl)
        fetch_target_lib(CycloneDDS::dds_security_ac)
        fetch_target_lib(CycloneDDS::dds_security_auth)
        fetch_target_lib(CycloneDDS::dds_security_crypto)
        list(APPEND REQ_STANDALONE_LIBS
          ${CycloneDDS_ddsc_LIB_PATH}
          ${CycloneDDS_idl_LIB_PATH}
          ${CycloneDDS_dds_security_ac_LIB_PATH}
          ${CycloneDDS_dds_security_auth_LIB_PATH}
          ${CycloneDDS_dds_security_crypto_LIB_PATH})
      elseif(UNIX)
        fetch_target_lib(CycloneDDS::ddsc)
        list(APPEND REQ_STANDALONE_LIBS ${CycloneDDS_ddsc_LIB_PATH})
      endif()
    endif()

    # Get rmw_cyclonedds_cpp
    if("${_library_name}" STREQUAL "rmw_cyclonedds_cpp")
      fetch_target_lib(rmw_cyclonedds_cpp::rmw_cyclonedds_cpp)
      list(APPEND REQ_STANDALONE_LIBS ${rmw_cyclonedds_cpp_rmw_cyclonedds_cpp_LIB_PATH})
    endif()

    # We skip python libs
    set(PYTHON_LIB_REGEX ".*python[0-9]*\.[0-9]*\.so")
    if(WIN32)
      set(PYTHON_LIB_REGEX ".*libs\/python[0-9]*\.lib" )
    endif()
    foreach(entry ${${_library_name}_LIBRARIES})
      string(REGEX MATCH ${PYTHON_LIB_REGEX} _found ${entry})
      if(_found STREQUAL "")
        list(APPEND REQ_STANDALONE_LIBS ${entry})
      endif()
    endforeach()
  endmacro()

  macro(install_standalone_dependencies)
    # Filter valid libraries
    list(FILTER REQ_STANDALONE_LIBS INCLUDE REGEX ".*(lib|dll|so)(\.[0-9])*$")
    list(REMOVE_DUPLICATES REQ_STANDALONE_LIBS)

    if(WIN32)
      foreach(lib_path ${REQ_STANDALONE_LIBS})
        string(REGEX REPLACE "\/(l|L)ib\/" "/bin/" bin_path ${lib_path})
        string(REGEX REPLACE "\.lib$" ".dll" dll_path ${bin_path})
        list(APPEND REQ_STANDALONE_DLLS ${dll_path})
      endforeach()
      install(FILES ${REQ_STANDALONE_DLLS}
         DESTINATION ros_standalone
      )
    elseif(UNIX)
      set(_resolvedFiles "")
      foreach(lib_path ${REQ_STANDALONE_LIBS})
        # Generate soversion files (.so.<major>). Some libs links to soversion symlinks which are not returned by find_package by default.
        get_filename_component(_resolvedFile "${lib_path}" REALPATH)
        list(APPEND _resolvedFiles "${_resolvedFile}")
      endforeach()

      install(FILES ${_resolvedFiles}
         DESTINATION ros_standalone
      )

      # Fix soversion files
      foreach(_resolvedFile ${_resolvedFiles})
        if("${_resolvedFile}" MATCHES "so(\.[0-9]+)+$")
          # Get file path without so
          string(FIND "${_resolvedFile}" ".so." _findPos)
          string(SUBSTRING "${_resolvedFile}" 0 ${_findPos} _cutted)

          set(_remainingPath "${_resolvedFile}")
          while("${_remainingPath}" MATCHES "so(\.[0-9]*)+$")
            string(FIND "${_remainingPath}" "." _lastDotPos REVERSE)
            string(SUBSTRING "${_remainingPath}" 0 ${_lastDotPos} _tempPos)
            get_filename_component(_libPathFilename "${_tempPos}" NAME)
            get_filename_component(_resolvedFilename "${_resolvedFile}" NAME)
            install(CODE "execute_process(COMMAND ln -s ${_resolvedFilename} ${_libPathFilename} WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/ros_standalone ERROR_QUIET)")
            set(_remainingPath "${_tempPos}")
          endwhile()
        endif()
      endforeach()

      # rpath for each standalone lib must be updated so all the libs will see each other
      install(CODE
        "execute_process(COMMAND find ${CMAKE_INSTALL_PREFIX}/ros_standalone -name *.so -exec patchelf --set-rpath $ORIGIN:. {} \;)"
      )
      # soversion files as well
      install(CODE
        "execute_process(COMMAND find ${CMAKE_INSTALL_PREFIX}/ros_standalone -name *.so.* -exec patchelf --set-rpath $ORIGIN:. {} \;)"
      )
    endif()
  endmacro()

  set(ros2_standalone_libs
    rcl
    rclcpp
    visualization_msgs  # TODO: set sensor_msgs instead and make it install properly
    FastRTPS
    rmw_fastrtps_cpp
    CycloneDDS
    rmw_cyclonedds_cpp
    rosidl_runtime_c
    rosidl_typesupport_c
    rosidl_typesupport_cpp
    rosidl_typesupport_introspection_c
    rosidl_typesupport_introspection_cpp
  )

  foreach(ros2_standalone_lib ${ros2_standalone_libs})
    get_standalone_dependencies(${ros2_standalone_lib})
  endforeach()
  install_standalone_dependencies()
endif()
