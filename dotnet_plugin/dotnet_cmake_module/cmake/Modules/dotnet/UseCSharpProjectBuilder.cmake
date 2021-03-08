# Original Copyright:
# Copyright (C) 2015-2017, Illumina, inc.
#
# Based on
# https://github.com/Illumina/interop/tree/master/cmake/Modules

function(csharp_add_project name)
    if(CSBUILD_PROJECT_DIR)
        set(CURRENT_TARGET_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/${CSBUILD_PROJECT_DIR}")
    else()
        set(CURRENT_TARGET_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
    set(CSBUILD_PROJECT_DIR "")
    file(MAKE_DIRECTORY ${CURRENT_TARGET_BINARY_DIR}/${name})
    cmake_parse_arguments(_csharp_add_project
        "EXECUTABLE"
        ""
        "SOURCES;INCLUDE_DLLS;INCLUDE_NUPKGS;INCLUDE_REFERENCES"
        ${ARGN}
    )

    set(
        _csharp_sources
        ${_csharp_add_project_SOURCES}
        ${_csharp_add_project_UNPARSED_ARGUMENTS}
    )

    foreach(it ${_csharp_add_project_INCLUDE_DLLS})
        file(TO_NATIVE_PATH ${it} nit)
        list(APPEND refs "<Reference;Include=\\\"${nit}\\\";/>")
    endforeach()

    foreach(it ${_csharp_add_project_INCLUDE_NUPKGS})
        file(TO_NATIVE_PATH ${it} nit)
        list(APPEND pkgs "<package;id=\\\"${nit}\\\";version=;/>")
    endforeach()

    foreach(it ${_csharp_add_project_INCLUDE_REFERENCES})
        string(REPLACE "=" ";" PACKAGE_ID "${it}")
        list(GET PACKAGE_ID 0 PACKAGE_NAME)
        list(GET PACKAGE_ID 1 PACKAGE_VERSION)
        list(APPEND packages "<PackageReference;Include=\\\"${PACKAGE_NAME}\\\";Version=\\\"${PACKAGE_VERSION}\\\";/>")
        list(APPEND legacy_packages "<package;id=\\\"${PACKAGE_NAME}\\\";version=\\\"${PACKAGE_VERSION}\\\";/>")
        file(TO_NATIVE_PATH "${CURRENT_TARGET_BINARY_DIR}/${name}/${PACKAGE_NAME}.${PACKAGE_VERSION}/lib/**/*.dll" hint_path)
        list(APPEND refs "<Reference;Include=\\\"${hint_path}\\\";></Reference>")

        file(TO_NATIVE_PATH "${CURRENT_TARGET_BINARY_DIR}/${name}/${PACKAGE_NAME}.${PACKAGE_VERSION}/build/${PACKAGE_NAME}.targets" target_path)
        list(APPEND imports "<Import;Project=\\\"${target_path}\\\";Condition=\\\"Exists('${target_path}')\\\";/>")
    endforeach()

    foreach(it ${_csharp_sources})
        if(EXISTS "${it}")
            file(TO_NATIVE_PATH ${it} nit)
            list(APPEND sources "<Compile;Include=\\\"${nit}\\\";/>")
            list(APPEND sources_dep ${it})
        elseif(EXISTS "${CSBUILD_SOURCE_DIRECTORY}/${it}")
            file(TO_NATIVE_PATH ${CSHARP_SOURCE_DIRECTORY}/${it} nit)
            list(APPEND sources "<Compile;Include=\\\"${nit}\\\";/>")
            list(APPEND sources_dep ${CSHARP_SOURCE_DIRECTORY}/${it})
        elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${it}")
            if(DOTNET_FOUND)
                string(REPLACE "/" "\\" nit "${CMAKE_CURRENT_SOURCE_DIR}/${it}")
            else()
                file(TO_NATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${it} nit)
            endif()
            list(APPEND sources "<Compile;Include=\\\"${nit}\\\";/>")
            list(APPEND sources_dep ${CMAKE_CURRENT_SOURCE_DIR}/${it})
        elseif(${it} MATCHES "[*]")
            file(TO_NATIVE_PATH ${it} nit)
            FILE(GLOB it_glob ${it})
            list(APPEND sources "<Compile;Include=\\\"${nit}\\\";/>")
            list(APPEND sources_dep ${it_glob})
        else()
            get_property(_is_generated SOURCE ${it} PROPERTY GENERATED)
            if(_is_generated)
                file(TO_NATIVE_PATH ${it} nit)
                FILE(GLOB it_glob ${it})
                list(APPEND sources "<Compile;Include=\\\"${nit}\\\";/>")
                list(APPEND sources_dep ${it_glob})
            else()
                message(WARNING "not found ${it}")
            endif()
        endif()
    endforeach()
    list(LENGTH sources SOURCE_FILE_COUNT)
    list(LENGTH refs REFERENCE_COUNT)
    list(LENGTH packages PACKAGE_COUNT)
    list(LENGTH imports IMPORT_COUNT)
    if(SOURCE_FILE_COUNT GREATER 0)
        set(CSHARP_BUILDER_SOURCES "${sources}")
    else()
        message(FATAL_ERROR "No C# source files for library")
    endif()
    if(REFERENCE_COUNT GREATER 0)
        set(CSHARP_BUILDER_ADDITIONAL_REFERENCES "${refs}")
    else()
        set(CSHARP_BUILDER_ADDITIONAL_REFERENCES "")
    endif()
    if(PACKAGE_COUNT GREATER 0)
        set(CSHARP_PACKAGE_REFERENCES "${packages}")
    else()
        set(CSHARP_PACKAGE_REFERENCES "")
    endif()
    if(PACKAGE_COUNT GREATER 0)
        set(CSHARP_LEGACY_PACKAGE_REFERENCES "${legacy_packages}")
    else()
        set(CSHARP_LEGACY_PACKAGE_REFERENCES "")
    endif()
    if(IMPORT_COUNT GREATER 0)
        set(CSHARP_IMPORTS "${imports}")
    else()
        set(CSHARP_IMPORTS "")
    endif()

    if(${_csharp_add_project_EXECUTABLE} AND NOT DOTNET_CORE_FOUND)
        set(ext "exe")
    else()
        set(ext "dll")
    endif()

    if(${_csharp_add_project_EXECUTABLE})
        set(output_type "Exe")
    else()
        set(output_type "library")
    endif()
    # TODO: <RuntimeIdentifier>osx.10.11-x64</RuntimeIdentifier>
    set(CSBUILD_${name}_BINARY "${CSHARP_BUILDER_OUTPUT_PATH}/${CSBUILD_OUTPUT_PREFIX}${name}${CSBUILD_OUTPUT_SUFFIX}.${ext}")
    set(CSBUILD_${name}_BINARY_NAME "${name}${CSBUILD_OUTPUT_SUFFIX}.${ext}")
    if(CSHARP_NUGET_SOURCE)
        set(CSHARP_NUGET_SOURCE_CMD -source ${CSHARP_NUGET_SOURCE})
    endif()

    if(RESTORE_EXE AND CSHARP_NUGET_SOURCE_CMD)
        set(RESTORE_CMD ${RESTORE_EXE} install ${CSHARP_NUGET_SOURCE_CMD})
    else()
        set(RESTORE_CMD ${CMAKE_COMMAND} -version)
    endif()

    set(CSBUILD_${name}_CSPROJ "${name}_${CSBUILD_CSPROJ}")
    file(TO_NATIVE_PATH ${CSHARP_BUILDER_OUTPUT_PATH} CSHARP_BUILDER_OUTPUT_PATH_NATIVE)

    add_custom_target(
        ${name} ALL
        ${CMAKE_COMMAND}
        -DCSHARP_TARGET_FRAMEWORK="${CSHARP_TARGET_FRAMEWORK}"
        -DCSHARP_BUILDER_OUTPUT_TYPE="${output_type}"
        -DCSHARP_BUILDER_OUTPUT_PATH="${CSHARP_BUILDER_OUTPUT_PATH_NATIVE}"
        -DCSHARP_PLATFORM="${CSHARP_PLATFORM}"
        -DCSHARP_BUILDER_OUTPUT_NAME="${name}${CSBUILD_OUTPUT_SUFFIX}"
        -DCSHARP_BUILDER_ADDITIONAL_REFERENCES="${CSHARP_BUILDER_ADDITIONAL_REFERENCES}"
        -DCSHARP_BUILDER_SOURCES="${CSHARP_BUILDER_SOURCES}"
        -DCSHARP_TARGET_FRAMEWORK_VERSION="${CSHARP_TARGET_FRAMEWORK_VERSION}"
        -DCSHARP_PACKAGE_REFERENCES="${CSHARP_PACKAGE_REFERENCES}"
        -DMSBUILD_TOOLSET="${MSBUILD_TOOLSET}"
        -DCSHARP_IMPORTS="${CSHARP_IMPORTS}"
        -DCONFIG_INPUT_FILE="${CSBUILD_CSPROJ_IN}"
        -DCONFIG_OUTPUT_FILE="${CURRENT_TARGET_BINARY_DIR}/${name}/${CSBUILD_${name}_CSPROJ}"
        -P ${dotnet_cmake_module_DIR}/ConfigureFile.cmake

        COMMAND ${CMAKE_COMMAND}
        -DCSHARP_PACKAGE_REFERENCES="${CSHARP_LEGACY_PACKAGE_REFERENCES}"
        -DCONFIG_INPUT_FILE="${dotnet_cmake_module_DIR}/Modules/dotnet/packages.config.in"
        -DCONFIG_OUTPUT_FILE="${CURRENT_TARGET_BINARY_DIR}/${name}/packages.config"
        -P ${dotnet_cmake_module_DIR}/ConfigureFile.cmake

        COMMAND ${CMAKE_COMMAND}
        -DCSHARP_BUILDER_OUTPUT_NAME="${name}${CSBUILD_OUTPUT_SUFFIX}"
        -DCONFIG_INPUT_FILE="${dotnet_cmake_module_DIR}/Modules/dotnet/Directory.Build.props.in"
        -DCONFIG_OUTPUT_FILE="${CURRENT_TARGET_BINARY_DIR}/${name}/Directory.Build.props"
        -P ${dotnet_cmake_module_DIR}/ConfigureFile.cmake

        COMMAND ${RESTORE_CMD}

        COMMAND ${CSBUILD_EXECUTABLE} ${CSBUILD_RESTORE_FLAGS} ${CSBUILD_${name}_CSPROJ}
        COMMAND ${CSBUILD_EXECUTABLE} ${CSBUILD_BUILD_FLAGS} ${CSBUILD_${name}_CSPROJ}
        WORKING_DIRECTORY ${CURRENT_TARGET_BINARY_DIR}/${name}
        COMMENT "${RESTORE_CMD};${CSBUILD_EXECUTABLE} ${CSBUILD_RESTORE_FLAGS} ${CSBUILD_${name}_CSPROJ}; ${CSBUILD_EXECUTABLE} ${CSBUILD_BUILD_FLAGS} ${CSBUILD_${name}_CSPROJ} -> ${CURRENT_TARGET_BINARY_DIR}/${name}"
        DEPENDS ${sources_dep}
    )

    set(DOTNET_OUTPUT_PATH ${CSHARP_BUILDER_OUTPUT_PATH}/${CSHARP_TARGET_FRAMEWORK}/${DOTNET_CORE_RUNTIME}/publish/)

    set_target_properties(${name}
        PROPERTIES
        EXECUTABLE
        ${_csharp_add_project_EXECUTABLE}
        OUTPUT_PATH
        ${DOTNET_OUTPUT_PATH}
        OUTPUT_NAME
        ${name}${CSBUILD_OUTPUT_SUFFIX}.${ext}
        DOTNET_CORE
        ${DOTNET_CORE_FOUND}
    )
endfunction()
