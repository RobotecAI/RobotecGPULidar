find_package(OpenGL REQUIRED)

include_directories(${OptiX_INCLUDE})

cuda_compile_and_embed(embedded_ptx_code devicePrograms.cu)

add_executable(lidar
  ${embedded_ptx_code}
  optix7.h
  CUDABuffer.h
  SampleRenderer.cpp
  LidarRenderer.cpp
  Model.cpp
  Lidar.cpp
  main.cpp
  )

target_link_libraries(lidar
  gdt
  # optix dependencies, for rendering
  ${optix_LIBRARY}
  ${CUDA_LIBRARIES}
  ${CUDA_CUDA_LIBRARY}
  # glfw and opengl, for display
  glfWindow
  glfw
  ${OPENGL_gl_LIBRARY}
  )

target_compile_options(lidar PRIVATE -Wall)
