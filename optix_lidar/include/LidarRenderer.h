#pragma once

// our own classes, partly shared between host and device
#include "gdt/utils/CUDABuffer.h"
#include "LaunchParams.h"
#include "model.h"

#include "LidarSource.h"
#include "RaycastResult.h"

#include <cstring>
#include <fmt/format.h>

class LidarRenderer
{
  // ------------------------------------------------------------------
  // publicly accessible interface
  // ------------------------------------------------------------------
public:
  LidarRenderer();
  ~LidarRenderer();

  /*! render one frame */
  void render(const std::vector<LidarSource> &lidars);

  /*! resize frame buffer to given resolution */
  void resize(const std::vector<LidarSource> &lidars);

  // TODO(prybicki): this return type is temporary and should be changed in the future refactor
  const RaycastResults* downloadPoints();

  void addTextures(std::vector<std::shared_ptr<Texture>> tex);
  void removeTexture(const std::string & tex_id);

  void addMeshes(std::vector<std::shared_ptr<TriangleMesh>> mesh);

  void addMeshRawTmp(const char* meshID,
    int meshSize, vec3f* vertices, vec3f* normals, vec2f* texCoords,
    int indicesSize, vec3i* indices,
    int transformSize, float* transform
    )
  {
    //Constructor could already use the pointers
    auto tm = std::make_shared<TriangleMesh>();

    std::vector<vec3f> v(vertices, vertices + meshSize);
    tm->vertex = v;

    std::vector<vec3f> n(normals, normals + meshSize);
    tm->normal = n;

    std::vector<vec2f> tc(texCoords, texCoords + meshSize);
    tm->texcoord = tc;

    std::vector<vec3i> ind(indices, indices + indicesSize);
    tm->index = ind;

    std::string mesh_id(meshID);
    tm->mesh_id = meshID;

    memcpy(tm->transform.matrix_flat, transform, sizeof(TransformMatrix));

    addMeshes({tm});
  }

  void removeMesh(const std::string & mesh_id);
  void removeMeshRawTmp(const char* meshID) {
      removeMesh(std::string(meshID));
  }

  void updateMeshTransform(const std::string & meshID, const TransformMatrix & transform);
  void updateMeshTransformRawTmp(char* meshID, float* transform, int transformSize)
  {
      if (transformSize != sizeof(TransformMatrix) / sizeof(float)) {
          auto message = fmt::format("Invalid transform size: {} (expected {})\n", transformSize, sizeof(TransformMatrix) / sizeof(float));
          throw std::invalid_argument(message);
      }

      TransformMatrix transformMatrix;
      memcpy(transformMatrix.matrix_flat, transform, sizeof(TransformMatrix));
      updateMeshTransform(std::string(meshID), transformMatrix);
  }


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

  void uploadRays(const std::vector<LidarSource> &lidars);

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

  RaycastResults result;

  // Ex-local buffers moved here to avoid memory allocations
  std::vector<int> raysPerLidar;
  std::vector<float> range;
  std::vector<float> source;
  std::vector<float> allPoints;
  std::vector<int> hits;
};
