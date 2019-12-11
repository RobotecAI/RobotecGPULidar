find_package(OpenGL REQUIRED)
project(testwindow)
include_directories(${OptiX_INCLUDE})
include_directories(../optixLidar)
#TODO
#cuda_compile_and_embed(embedded_ptx_code_2 renderDevicePrograms.cu)

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
