#pragma once

// our own classes, partly shared between host and device
#include "CUDABuffer.h"
#include "LaunchParams.h"
#include "Model.h"
#include "gdt/math/vec.h"

struct Camera {
    /*! camera position - *from* where we are looking */
    vec3f from;
    /*! which point we are looking *at* */
    vec3f at;
    /*! general up-vector */
    vec3f up;
};


class SampleRenderer
{
// ------------------------------------------------------------------
// publicly accessible interface
// ------------------------------------------------------------------
public:
    /*! constructor - performs all setup, including initializing
      optix, creates module, pipeline, programs, SBT, etc. */
    SampleRenderer(const Model *model);

    /*! render one frame */
    void render(std::vector<float> &lidarPoints);

    /*! resize frame buffer to given resolution */
    void resize(const vec2i &newSize);

    /*! resize lidar frame buffer to given resolution */
    void resizeLidar(uint32_t lidarSize);

    /*! download the rendered color buffer */
    void downloadPixels(uint32_t h_pixels[]);

    /*! set camera to render with */
    void setCamera(const Camera &camera);
    
    void setModel(const Model *model) {this->model = model;}

protected:
    // ------------------------------------------------------------------
    // internal helper functions
    // ------------------------------------------------------------------

    /*! helper function that initializes optix and checks for errors */
    void initOptix();

    /*! creates and configures a optix device context (in this simple
      example, only for the primary GPU device) */
    void createContext();

    /*! creates the module that contains all the programs we are going
      to use. in this simple example, we use a single module from a
      single .cu file, using a single embedded ptx string */
    void createModule();

    /*! does all setup for the raygen program(s) we are going to use */
    void createRaygenPrograms();

    /*! does all setup for the miss program(s) we are going to use */
    void createMissPrograms();

    /*! does all setup for the hitgroup program(s) we are going to use */
    void createHitgroupPrograms();

    /*! assembles the full pipeline of all programs */
    void createPipeline();

    /*! constructs the shader binding table */
    void buildSBT();

    /*! build an acceleration structure for the given triangle mesh */
    OptixTraversableHandle buildAccel(bool update = false);

    /*! upload textures, and create cuda texture objects for them */
    void createTextures();

protected:
    /*! @{ CUDA device context and stream that optix pipeline will run
        on, as well as device properties for this device */
    CUcontext          cudaContext;
    CUstream           stream;
    cudaDeviceProp     deviceProps;
    /*! @} */

    //! the optix context that our pipeline will run in.
    OptixDeviceContext optixContext;

    /*! @{ the pipeline we're building */
    OptixPipeline               pipeline;
    OptixPipelineCompileOptions pipelineCompileOptions;
    OptixPipelineLinkOptions    pipelineLinkOptions;
    /*! @} */

    /*! @{ the module that contains out device programs */
    OptixModule                 module;
    OptixModuleCompileOptions   moduleCompileOptions;
    /* @} */

    /*! vector of all our program(group)s, and the SBT built around
        them */
    std::vector<OptixProgramGroup> raygenPGs;
    CUDABuffer raygenRecordsBuffer;
    std::vector<OptixProgramGroup> missPGs;
    CUDABuffer missRecordsBuffer;
    std::vector<OptixProgramGroup> hitgroupPGs;
    CUDABuffer hitgroupRecordsBuffer;
    OptixShaderBindingTable sbt = {};

    /*! @{ our launch parameters, on the host, and the buffer to store
        them on the device */
    LaunchParams launchParams;
    CUDABuffer   launchParamsBuffer;
    /*! @} */

    CUDABuffer colorBuffer;
    CUDABuffer lidarBuffer; // buffer to send lidar data to device

    /*! the camera we are to render with. */
    Camera lastSetCamera;

    /*! the model we are going to trace rays against */
    const Model *model;

    /*! @{ one buffer per input mesh */
    std::vector<CUDABuffer> vertexBuffer;
    std::vector<CUDABuffer> normalBuffer;
    std::vector<CUDABuffer> texcoordBuffer;
    std::vector<CUDABuffer> indexBuffer;
    /*! @} */

    //! buffer that keeps the (final, compacted) accel structure
    CUDABuffer asBuffer;
    CUDABuffer outputBuffer;

    /*! @{ one texture object and pixel array per used texture */
    std::vector<cudaArray_t>         textureArrays;
    std::vector<cudaTextureObject_t> textureObjects;
    /*! @} */
};
