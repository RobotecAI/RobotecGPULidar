project(lidar)
include_directories(${OptiX_INCLUDE})

cuda_compile_and_embed(embedded_ptx_code devicePrograms.cu)

add_library(lidar STATIC
    ${embedded_ptx_code}
    LidarRenderer.cpp
    Model.cpp
    Lidar.cpp
    )

target_link_libraries(lidar
    gdt
    # optix dependencies, for rendering
    ${optix_LIBRARY}
    ${CUDA_LIBRARIES}
    ${CUDA_CUDA_LIBRARY}
    )

target_compile_options(lidar PRIVATE -Wall)
