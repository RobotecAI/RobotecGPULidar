#pragma once

#include "LaunchParams.h"
#include "ModelInstance.h"
#include "Texture.h"
#include "TransformMatrix.h"
#include "TriangleMesh.h"
#include "gdt/utils/CUDABuffer.h"

#include "LidarSource.h"
#include "RaycastResult.h"
#include "ShaderBindingTableTypes.h"
#include "Logging.h"

#include "DeviceBuffer.hpp"
#include "HostPinnedBuffer.hpp"

#include <cstring>

// RAII object to (de)initialize OptiX
struct OptiXInitializationGuard
{
    OptiXInitializationGuard();
    ~OptiXInitializationGuard();
    OptixDeviceContext context = nullptr;
};


struct LidarRenderer {

    LidarRenderer();
    ~LidarRenderer();

    void render(const std::vector<LidarSource>& lidars);

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
    OptixShaderBindingTable buildSBT();
    OptixTraversableHandle buildAccel();
    void createTextures();
    bool ensureBuffersPreparedBeforeRender(const std::vector<LidarSource>& lidars);
    void addMeshUnchecked(std::shared_ptr<TriangleMesh> meshes);

    OptiXInitializationGuard optix;
    OptixModule module;
    OptixPipeline pipeline;

    OptixProgramGroup raygenPG;
    OptixProgramGroup missPG;
    OptixProgramGroup hitgroupPG;
    OptixShaderBindingTable sbt;

    // GPU INPUT
    DeviceBuffer<int> dRayCountOfLidar;
        std::vector<int> hRayCountOfLidar;
    DeviceBuffer<Point3f> dRayDirs;
    DeviceBuffer<float> dRangeOfLidar;
    DeviceBuffer<Point3f> dPositionOfLidar;

    DeviceBuffer<LaunchLidarParams> dLaunchParams;
        LaunchLidarParams hLaunchParams;

    // GPU OUTPUT
    DeviceBuffer<LidarPoint> dHitXYZI;
        HostPinnedBuffer<LidarPoint> hHitXYZI;

    DeviceBuffer<int> dHitIsFinite;
        HostPinnedBuffer<int> hHitIsFinite;

    // MODEL STUFF
    std::unordered_map<std::string, std::shared_ptr<ModelInstance>> m_instances_map;
    bool needs_root_rebuild = { false };

    OptixTraversableHandle m_root; // Scene root

    // Buffer that keeps the (final, compacted) accel structure
    CUDABuffer accelerationStructure;

    std::vector<cudaArray_t> textureArrays;
    std::vector<cudaTextureObject_t> textureObjects;

    RaycastResults result;

public:
    void softReset() {
        m_instances_map.clear();
        needs_root_rebuild = true;
    }

};
