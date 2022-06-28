include(FetchContent)

# Note: SPDLOG includes FMT
set(SPDLOG_BUILD_SHARED ON)
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.10.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(spdlog)

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
