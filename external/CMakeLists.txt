
add_subdirectory("${PROJECT_SOURCE_DIR}/external/spdlog")
add_subdirectory("${PROJECT_SOURCE_DIR}/external/yaml-cpp")
add_subdirectory("${PROJECT_SOURCE_DIR}/external/cmake_git_version_tracking")

# An attempt to disable targets from yaml-cpp; had difficulties with some of them
# More details here: https://github.com/jbeder/yaml-cpp/issues/1158
set(YAML_CPP_BUILD_TESTS OFF CACHE INTERNAL "Disable yaml-cpp artifacts")
set(YAML_CPP_BUILD_CONTRIB OFF CACHE INTERNAL "Disable yaml-cpp artifacts")
set(YAML_CPP_BUILD_TOOLS OFF CACHE INTERNAL "Disable yaml-cpp artifacts")
set(YAML_CPP_INSTALL OFF CACHE INTERNAL "Disable yaml-cpp artifacts")
set(YAML_CPP_FORMAT_SOURCE OFF CACHE INTERNAL "Disable yaml-cpp artifacts")
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

if (${RGL_BUILD_TESTS} OR ${RGL_BUILD_TAPED_TESTS})
    add_subdirectory("${PROJECT_SOURCE_DIR}/external/googletest")

    set(INSTALL_GTEST OFF CACHE INTERNAL "Disable installation of googletest")

    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif()
