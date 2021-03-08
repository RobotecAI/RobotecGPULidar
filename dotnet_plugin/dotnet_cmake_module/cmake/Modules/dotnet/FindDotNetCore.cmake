# Original Copyright:
# Copyright (C) 2015-2017, Illumina, inc.
#
# Based on
# https://github.com/Illumina/interop/tree/master/cmake/Modules
#
# Find the msbuild tool
#
# DOTNET_CORE_FOUND             System has msbuild
# DOTNET_CORE_EXECUTABLE        Where to find csc
# DOTNET_CORE_VERSION           The version number of the DotNet framework

set(DOTNET_CORE_ROOT "" CACHE PATH "Set the location of the .NET root directory")
set(DOTNET_CORE_VERSION "" CACHE STRING "C# .NET compiler version")

if(DOTNET_CORE_ROOT AND EXISTS "${DOTNET_CORE_ROOT}")
    find_program(DOTNET_CORE_EXECUTABLE dotnet dotnet.exe
            PATHS ${DOTNET_CORE_ROOT}
            PATH_SUFFIXES . bin
            NO_DEFAULT_PATH)
endif()


find_program(DOTNET_CORE_EXECUTABLE dotnet dotnet.exe)

if(EXISTS "${DOTNET_CORE_EXECUTABLE}")
    execute_process(
        COMMAND ${DOTNET_CORE_EXECUTABLE} --version
        OUTPUT_VARIABLE dotnet_core_version_string
        OUTPUT_STRIP_TRAILING_WHITESPACE
   )
    string(REGEX MATCH "([0-9]*)([.])([0-9]*)([.]*)([0-9]*)" dotnet_core_version_string "${dotnet_core_version_string}")
    set(DOTNET_CORE_VERSION ${dotnet_core_version_string} CACHE STRING ".NET coreclr version" FORCE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DOTNET_CORE DEFAULT_MSG DOTNET_CORE_EXECUTABLE)
mark_as_advanced(DOTNET_CORE_EXECUTABLE DOTNET_CORE_VERSION)

