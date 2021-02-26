project(optix_lidar)
include_directories(include ${OptiX_INCLUDE})

cuda_compile_and_embed(embedded_ptx_code devicePrograms.cu)

add_library(${PROJECT_NAME} SHARED
    ${embedded_ptx_code}
    optix_lidar.cpp
    LidarRenderer.cpp
    model_utils.cpp
    simple_uid_generator.cpp
    Lidar.cpp
    )

target_link_libraries(${PROJECT_NAME}
    gdt
    # optix dependencies, for rendering
    ${optix_LIBRARY}
    ${CUDA_LIBRARIES}
    ${CUDA_CUDA_LIBRARY}
    )

target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
