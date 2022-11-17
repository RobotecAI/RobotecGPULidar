include(FetchContent)

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.10.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(spdlog)

FetchContent_Declare(
    cmake_git_version_tracking
    GIT_REPOSITORY https://github.com/andrew-hardin/cmake-git-version-tracking.git
    GIT_TAG 904dbda1336ba4b9a1415a68d5f203f576b696bb
)
FetchContent_MakeAvailable(cmake_git_version_tracking)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG yaml-cpp-0.7.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(yaml-cpp)
set_property(TARGET yaml-cpp PROPERTY POSITION_INDEPENDENT_CODE ON)

# Disable compilation warnings for dependencies
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(DISABLE_WARNINGS_FLAG "/W3")
endif()
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(DISABLE_WARNINGS_FLAG "-w")
endif()
if (DEFINED DISABLE_WARNINGS_FLAG)
    target_compile_options(yaml-cpp PRIVATE ${DISABLE_WARNINGS_FLAG})
    target_compile_options(spdlog PRIVATE ${DISABLE_WARNINGS_FLAG})
endif()

if (UNIX)
    # This is a fix for:
    # relocation R_X86_64_TPOFF32 against `_ZGVZN6spdlog7details2os9thread_idEvE3tid'
    # can not be used when making a shared object; recompile with -fPIC
    set_target_properties(spdlog PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if (${RGL_BUILD_TESTS})
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest
        GIT_TAG release-1.11.0
        GIT_SHALLOW TRUE
    )

    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()
