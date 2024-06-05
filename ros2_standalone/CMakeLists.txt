# This file is based on https://github.com/RobotecAI/ros2cs/blob/develop/src/ros2cs/ros2cs_core/CMakeLists.txt

cmake_minimum_required(VERSION 3.18)

project(RobotecGPULidar_ros2_standalone)

find_program(PATCHELF "patchelf")
if(UNIX)
    if(NOT PATCHELF)
        message(FATAL_ERROR "'patchelf' executable not found. Linux standalone version requires 'patchelf' for patching 'runpath' of certain libraries. You can install it via 'sudo apt install patchelf'")
    else()
        message("'patchelf' found in ${PATCHELF}")
    endif()
endif()
set(REQ_STANDALONE_LIBS "")
set(REQ_THIRD_PARTY_STANDALONE_LIBS "")
set(REQ_STANDALONE_DLLS "")

set(INSTALL_DESTINATION_DIR "ros2_standalone")

# Extend REQ_THIRD_PARTY_STANDALONE_LIBS with _library_name third party dependencies
macro(get_standalone_third_party_dependencies _library_name)
    find_file(${_library_name}_PATH "${_library_name}")
    if("${${_library_name}_PATH}" STREQUAL "${_library_name}_PATH-NOTFOUND")
        message( FATAL_ERROR "Can't find third party dependency: ${_library_name}" )
    endif()

    list(APPEND REQ_THIRD_PARTY_STANDALONE_LIBS ${${_library_name}_PATH})
endmacro()

# Install all libraries listed in REQ_THIRD_PARTY_STANDALONE_LIBS
macro(install_standalone_third_party)
    install(FILES ${REQ_THIRD_PARTY_STANDALONE_LIBS}
            DESTINATION ${INSTALL_DESTINATION_DIR}/../lib/ros2_standalone
    )
endmacro()

# Fetch location for library target
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
    find_package(${_library_name} QUIET)

    if(${${_library_name}_FOUND})
        # Get rmw_dds_common typesupports for dds implementations
        if("${_library_name}" STREQUAL "rmw_dds_common")
            ament_index_get_resources(middleware_implementations "rmw_typesupport")
            foreach(rmw_impl ${middleware_implementations})
                string(REGEX REPLACE "rmw_" "" rmw_impl ${rmw_impl})
                list(APPEND REQ_STANDALONE_LIBS ${rmw_dds_common_LIBRARIES__rosidl_typesupport_${rmw_impl}})
            endforeach()
        endif()

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

        # Get cyclonedds iceoryx dependency
        if(UNIX)
            if("${_library_name}" STREQUAL "iceoryx_binding_c" )
                fetch_target_lib(iceoryx_binding_c::iceoryx_binding_c)
                list(APPEND REQ_STANDALONE_LIBS ${iceoryx_binding_c_iceoryx_binding_c_LIB_PATH})
            elseif("${_library_name}" STREQUAL "iceoryx_posh")
                fetch_target_lib(iceoryx_posh::iceoryx_posh)
                list(APPEND REQ_STANDALONE_LIBS ${iceoryx_posh_iceoryx_posh_LIB_PATH})
            elseif("${_library_name}" STREQUAL "iceoryx_hoofs")
                fetch_target_lib(iceoryx_hoofs::iceoryx_hoofs)
                fetch_target_lib(iceoryx_hoofs::iceoryx_platform)
                list(APPEND REQ_STANDALONE_LIBS
                  ${iceoryx_hoofs_iceoryx_hoofs_LIB_PATH}
                  ${iceoryx_hoofs_iceoryx_platform_LIB_PATH})
            endif()
        endif()

        # If library is msgs, fetch all targets to get libraries for dynamic type support
        # Those libraries are not listed in <XX>_LIBRARIES (which stands for libraries to link against to use <XX>)
        if(${_library_name} MATCHES ".*\_msgs$")
            foreach(entry ${${_library_name}_TARGETS})
                fetch_target_lib(${entry})
                string(REGEX REPLACE "::" "_" entry_normalized ${entry})
                list(APPEND REQ_STANDALONE_LIBS
                        ${${entry_normalized}_LIB_PATH})
            endforeach()
        endif()

        # Get spdlog and dependency
        if(UNIX AND "${_library_name}" STREQUAL "spdlog")
            include(${${_library_name}_CONFIG})
            fetch_target_lib(spdlog::spdlog)
            list(APPEND REQ_STANDALONE_LIBS ${spdlog_spdlog_LIB_PATH})
            fetch_target_lib(fmt::fmt)
            list(APPEND REQ_STANDALONE_LIBS ${fmt_fmt_LIB_PATH})
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
    else()
        # Try to use pkg config to look for the packages
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(${_library_name} QUIET ${_library_name})
        if (${${_library_name}_FOUND})
            list(APPEND REQ_STANDALONE_LIBS ${${_library_name}_LINK_LIBRARIES})
        else()
            message(FATAL_ERROR "Could not find ${_library_name}")
        endif()
    endif()
endmacro()

# Install all libraries listed in REQ_STANDALONE_LIBS
macro(install_standalone_dependencies)
    # Filter valid libraries
    list(FILTER REQ_STANDALONE_LIBS INCLUDE REGEX ".*(lib|dll|so)(\.[0-9]+)*$")
    list(REMOVE_DUPLICATES REQ_STANDALONE_LIBS)

    if(WIN32)
        foreach(lib_path ${REQ_STANDALONE_LIBS})
            string(REGEX REPLACE "\/(l|L)ib\/" "/bin/" bin_path ${lib_path})
            string(REGEX REPLACE "\.lib$" ".dll" dll_path ${bin_path})
            list(APPEND REQ_STANDALONE_DLLS ${dll_path})
        endforeach()
        list(REMOVE_DUPLICATES REQ_STANDALONE_DLLS)
        install(FILES ${REQ_STANDALONE_DLLS}
            DESTINATION ${INSTALL_DESTINATION_DIR}/../lib/ros2_standalone
        )
    elseif(UNIX)
        set(_resolvedFiles "")
        foreach(lib_path ${REQ_STANDALONE_LIBS})
            # Generate soversion files (.so.<major>). Some libs links to soversion symlinks which are not returned by find_package by default.
            get_filename_component(_resolvedFile "${lib_path}" REALPATH)
            list(APPEND _resolvedFiles "${_resolvedFile}")
        endforeach()

        install(FILES ${_resolvedFiles}
            DESTINATION ${INSTALL_DESTINATION_DIR}/../lib/ros2_standalone
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
                    install(CODE "execute_process(COMMAND ln -s ${_resolvedFilename} ${_libPathFilename} WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${INSTALL_DESTINATION_DIR}/../lib/ros2_standalone ERROR_QUIET)")
                    set(_remainingPath "${_tempPos}")
                endwhile()
            endif()
        endforeach()

        # rpath for each standalone lib must be updated so all the libs will see each other
        install(CODE
          "execute_process(COMMAND find ${CMAKE_INSTALL_PREFIX}/${INSTALL_DESTINATION_DIR}/../lib/ros2_standalone -name *.so -exec patchelf --set-rpath $ORIGIN:. {} \;)"
        )
        # soversion files as well
        install(CODE
          "execute_process(COMMAND find ${CMAKE_INSTALL_PREFIX}/${INSTALL_DESTINATION_DIR}/../lib/ros2_standalone -name *.so.* -exec patchelf --set-rpath $ORIGIN:. {} \;)"
        )
    endif()
endmacro()

# Libraries required by RGL
set(ros2_standalone_libs
  rcl
  rclcpp
  sensor_msgs
  visualization_msgs
  radar_msgs
  rmw_implementation
  rmw_dds_common
  FastRTPS
  rmw_fastrtps_cpp
  CycloneDDS
  rmw_cyclonedds_cpp
  rosidl_runtime_c
  rosidl_typesupport_c
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_c
  rosidl_typesupport_introspection_cpp
  spdlog
  tracetools
  yaml
  tinyxml2
)

if(UNIX)
    list(APPEND ros2_standalone_libs iceoryx_binding_c iceoryx_posh iceoryx_hoofs)
endif()

foreach(ros2_standalone_lib ${ros2_standalone_libs})
    get_standalone_dependencies(${ros2_standalone_lib})
endforeach()
install_standalone_dependencies()

if(WIN32)
    set(third_party_standalone_libs
      libssl-1_1-x64.dll
      libcrypto-1_1-x64.dll
      msvcp140.dll
      vcruntime140.dll
      vcruntime140_1.dll
      tinyxml2.dll
    )
    foreach(third_party_lib ${third_party_standalone_libs})
        get_standalone_third_party_dependencies(${third_party_lib})
    endforeach()
    install_standalone_third_party()
endif()
