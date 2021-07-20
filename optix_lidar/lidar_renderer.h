#pragma once

// our own classes, partly shared between host and device
#include "gdt/utils/CUDABuffer.h"
#include "LaunchParams.h"
#include "model.h"

#include "lidar_source.h"
#include "raycast_result.h"

class LidarRenderer
{
  // ------------------------------------------------------------------
  // publicly accessible interface
  // ------------------------------------------------------------------
public:
  LidarRenderer();
  ~LidarRenderer();

  /*! render one frame */
  void render(std::vector<LidarSource> &lidars);

  /*! resize frame buffer to given resolution */
  void resize(std::vector<LidarSource> &lidars);

  /*! download lidar hit points */
  void downloadPoints(RaycastResults &result);

  void addTextures(Textures tex);
  void removeTexture(const std::string & tex_id);

  void addMeshes(Meshes mesh);
  void updateMeshTransform(const std::string & mesh_id, const TransformMatrix & transform);
  void removeMesh(const std::string & mesh_id);

private:
  void update_structs_for_model();

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
  // in the future for update reload not whole model, only dynamic part
  OptixTraversableHandle buildAccel();

  /*! upload textures, and create cuda texture objects for them */
  void createTextures();

  void uploadRays(std::vector<LidarSource> &lidars);

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
  LaunchLidarParams launchParams;
  CUDABuffer   launchParamsBuffer;
  /*! @} */

  CUDABuffer raysPerLidarBuffer;
  CUDABuffer rayBuffer;
  CUDABuffer rangeBuffer;
  CUDABuffer sourceBuffer;

  CUDABuffer positionBuffer;
  CUDABuffer hitBuffer;

  /*! the model we are going to trace rays against */
  // Model model;
  InstancesMap m_instances_map;
  bool needs_root_rebuild = {false};



  OptixTraversableHandle m_root;  // Scene root
  CUdeviceptr m_d_ias; // Scene root's IAS (instance acceleration structure).

  //! buffer that keeps the (final, compacted) accel structure
  CUDABuffer asBuffer;
  CUDABuffer outputBuffer;
  OptixTraversableHandle meshHandle;
  CUDABuffer compactedSizeBuffer;
  CUDABuffer tempBuffer;

  /*! @{ one texture object and pixel array per used texture */
  std::vector<cudaArray_t>         textureArrays;
  std::vector<cudaTextureObject_t> textureObjects;
  /*! @} */


  // Ex-local buffers moved here to avoid memory allocations
  std::vector<int> raysPerLidar;
  std::vector<float> range;
  std::vector<float> source;
  std::vector<float> allPoints;
  std::vector<int> hits;
};
