project(optix_lidar)
include_directories(include ${OptiX_INCLUDE})

cuda_compile_and_embed(embedded_ptx_code devicePrograms.cu)

add_library(${PROJECT_NAME} SHARED
    ${embedded_ptx_code}
    optix_lidar.cpp
    lidar_renderer.cpp
        simple_uid_generator.cpp
        model.cpp
    )

find_package(fmt)

target_link_libraries(${PROJECT_NAME}
    gdt
    fmt::fmt # FMT Library for a proper string formatting in C++
    # optix dependencies, for rendering
    ${optix_LIBRARY}
    ${CUDA_LIBRARIES}
    ${CUDA_CUDA_LIBRARY}
    )

target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
