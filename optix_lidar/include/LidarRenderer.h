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

  void addTextures(std::vector<std::shared_ptr<Texture>> tex);
  void removeTexture(const std::string & tex_id);

  void addMeshes(std::vector<std::shared_ptr<TriangleMesh>> mesh);
  void updateMeshTransform(const std::string & mesh_id, const TransformMatrix & transform);
  void removeMesh(const std::string & mesh_id);

private:
  std::string getCurrentDeviceName();
  CUcontext getCurrentDeviceContext();

  void update_structs_for_model();

  void initializeStaticOptixStructures();

  /*! constructs the shader binding table */
  void buildSBT();

  /*! build an acceleration structure for the given triangle mesh */
  // in the future for update reload not whole model, only dynamic part
  OptixTraversableHandle buildAccel();

  /*! upload textures, and create cuda texture objects for them */
  void createTextures();

  void uploadRays(std::vector<LidarSource> &lidars);

  OptixModule module;
  OptixPipeline pipeline;
  OptixDeviceContext optixContext;
  OptixProgramGroup raygenPG;
  OptixProgramGroup missPG;
  OptixProgramGroup hitgroupPG;
  OptixShaderBindingTable sbt;

  CUDABuffer raygenRecordsBuffer;
  CUDABuffer missRecordsBuffer;
  CUDABuffer hitgroupRecordsBuffer;

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
