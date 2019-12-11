find_package(OpenGL REQUIRED)
project(testwindow)
include_directories(${OptiX_INCLUDE})
include_directories(../optixLidar)

#cuda_compile_and_embed(embedded_ptx_code_2 renderDevicePrograms.cu)

# ------------------------------------------------------------------
# build glfw
# ------------------------------------------------------------------
if (WIN32)
#  set(glfw_dir ${PROJECT_SOURCE_DIR}/submodules/glfw/)
  set(glfw_dir ${PROJECT_SOURCE_DIR}/glfWindow/glfw/)
  include_directories(${glfw_dir}/include)
  add_subdirectory(${glfw_dir} EXCLUDE_FROM_ALL)
else()
  find_package(glfw3 REQUIRED)
endif()
add_subdirectory(glfWindow EXCLUDE_FROM_ALL)


add_executable(testwindow
    #${embedded_ptx_code_2}
    SampleRenderer.cpp
    main.cpp
    )

target_link_libraries(testwindow
    lidar
    glfWindow
    glfw
    ${OPENGL_gl_LIBRARY}
    )
