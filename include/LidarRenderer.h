#pragma once

#include "LaunchParams.h"
#include "ModelInstance.h"
#include "Texture.h"
#include "TransformMatrix.h"
#include "TriangleMesh.h"
#include "gdt/utils/CUDABuffer.h"

#include "LidarSource.h"
#include "RaycastResult.h"

#include <cstring>
#include <fmt/format.h>

struct LidarRenderer {
    LidarRenderer();
    ~LidarRenderer();

    void render(const std::vector<LidarSource>& lidars);

    void resize(const std::vector<LidarSource>& lidars);

    // TODO(prybicki): this return type is temporary and should be changed in the future refactor
    const RaycastResults* downloadPoints();


    void addMeshRaw(const char* meshID,
                    int meshSize, vec3f* vertices, vec3f* normals, vec2f* texCoords,
                    int indicesSize, vec3i* indices,
                    int transformSize, float* transform);

    void removeMesh(const std::string& mesh_id);
    void removeMeshRawTmp(const char* meshID);

    void updateMeshTransform(const std::string& meshID, const TransformMatrix& transform);
    void updateMeshTransformRawTmp(char* meshID, float* transform, int transformSize);

private:
    std::string getCurrentDeviceName();
    CUcontext getCurrentDeviceContext();
    void updateStructsForModel();
    void initializeStaticOptixStructures();
    void buildSBT();
    OptixTraversableHandle buildAccel();
    void createTextures();
    void uploadRays(const std::vector<LidarSource>& lidars);
    void addMeshUnchecked(std::shared_ptr<TriangleMesh> meshes);

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

    LaunchLidarParams launchParams;
    CUDABuffer launchParamsBuffer;

    CUDABuffer raysPerLidarBuffer;
    CUDABuffer rayBuffer;
    CUDABuffer rangeBuffer;
    CUDABuffer sourceBuffer;

    CUDABuffer positionBuffer;
    CUDABuffer hitBuffer;

    std::unordered_map<std::string, std::shared_ptr<ModelInstance>> m_instances_map;
    bool needs_root_rebuild = { false };

    OptixTraversableHandle m_root; // Scene root

    // Buffer that keeps the (final, compacted) accel structure
    CUDABuffer accelerationStructure;

    std::vector<cudaArray_t> textureArrays;
    std::vector<cudaTextureObject_t> textureObjects;

    RaycastResults result;

    // Ex-local buffers moved here to avoid memory allocations
    std::vector<int> raysPerLidar;
    std::vector<float> range;
    std::vector<float> source;
    std::vector<float> allPoints;
    std::vector<int> hits;
};
