# This file is based on https://github.com/RobotecAI/ros2cs/blob/master/src/ros2cs/ros2cs_core/CMakeLists.txt

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

# Get location for library target
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

# Extend REQ_STANDALONE_LIBS with _library_name dependencies
macro(get_standalone_dependencies _library_name)
    find_package(${_library_name} REQUIRED)

    # Get cyclonedds DDSC
    if("${_library_name}" STREQUAL "CycloneDDS")
        if(WIN32)
            fetch_target_lib(CycloneDDS::ddsc)
            fetch_target_lib(CycloneDDS::idl)
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
        set(PYTHON_LIB_REGEX ".*(l|L)ib(s|)\/python[0-9]*\.lib" )
    endif()
    foreach(entry ${${_library_name}_LIBRARIES})
        string(REGEX MATCH ${PYTHON_LIB_REGEX} _found ${entry})
        if(_found STREQUAL "")
            list(APPEND REQ_STANDALONE_LIBS ${entry})
        endif()
    endforeach()
endmacro()

# Install all libraries listed in REQ_STANDALONE_LIBS
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
          DESTINATION ros2_standalone
        )
    elseif(UNIX)
        set(_resolvedFiles "")
        foreach(lib_path ${REQ_STANDALONE_LIBS})
            # Generate soversion files (.so.<major>). Some libs links to soversion symlinks which are not returned by find_package by default.
            get_filename_component(_resolvedFile "${lib_path}" REALPATH)
            list(APPEND _resolvedFiles "${_resolvedFile}")
        endforeach()
        install(FILES ${_resolvedFiles}
          DESTINATION ros2_standalone
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
                    install(CODE "execute_process(COMMAND ln -s ${_resolvedFilename} ${_libPathFilename} WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/ros2_standalone ERROR_QUIET)")
                    set(_remainingPath "${_tempPos}")
                endwhile()
            endif()
        endforeach()

        # rpath for each standalone lib must be updated so all the libs will see each other
        install(CODE
          "execute_process(COMMAND find ${CMAKE_INSTALL_PREFIX}/ros2_standalone -name *.so -exec patchelf --set-rpath $ORIGIN:. {} \;)"
        )
        # soversion files as well
        install(CODE
          "execute_process(COMMAND find ${CMAKE_INSTALL_PREFIX}/ros2_standalone -name *.so.* -exec patchelf --set-rpath $ORIGIN:. {} \;)"
        )
    endif()
endmacro()

# Libraries required by RGL
set(ros2_standalone_libs
  rcl
  rclcpp
  sensor_msgs
  visualization_msgs
  rmw_implementation
  FastRTPS
  rmw_fastrtps_cpp
  CycloneDDS
  rmw_cyclonedds_cpp
  rmw_dds_common
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
