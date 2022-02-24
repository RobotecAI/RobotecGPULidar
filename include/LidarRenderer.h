#pragma once

#include "../common/Texture.h"
#include "LaunchParams.h"
#include "ModelInstance.h"
#include "TransformMatrix.h"
#include "TriangleMesh.h"
#include "data_types/ShaderBindingTableTypes.h"
#include "data_types/LidarSource.h"
#include "gdt/utils/CUDABuffer.h"

#include "Logging.h"

#include "DeviceBuffer.hpp"
#include "HostPinnedBuffer.hpp"
#include "data_types/PCLFormats.h"

#include <cstring>

#include <thrust/device_vector.h>
#include <thread>

#define restrict __restrict__

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
    int getNextDownloadPointCount();
    void downloadPoints(int maxPointCount,
                        Point3f* restrict outXYZ,
                        PCL12* restrict outPCL12,
                        PCL24* restrict outPCL24,
                        PCL48* restrict outPCL48,
                        double timestamp);

    // This is a slower overload for tests
    void downloadPoints(int maxPointCount, Point3f* outXYZ);

    void addMeshRaw(const char* meshID,
                    int meshSize, gdt::vec3f* vertices, gdt::vec3f* normals, gdt::vec2f* texCoords,
                    int indicesSize, gdt::vec3i* indices,
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
    DeviceBuffer<int> dRayCountOfLidar; // prev: raysPerLidarBuffer
        std::vector<int> hRayCountOfLidar;
    DeviceBuffer<TransformMatrix> dRayPoses;
    DeviceBuffer<int> dLidarArrayRingIds;
    DeviceBuffer<float> dRangeOfLidar;

    DeviceBuffer<LaunchLidarParams> dLaunchParams;
        LaunchLidarParams hLaunchParams;

    // GPU OUTPUT
    DeviceBuffer<int> dHitIsFinite;
    DeviceBuffer<int> dHitsBeforeIndex;

    DeviceBuffer<Point3f> dPoint3f; // Native output
    DeviceBuffer<PCL12> dPCL12; // Native output

    int hitpointCount;
    DeviceBuffer<Point3f> dDensePoint3f;
    DeviceBuffer<PCL12> dDensePCL12;
    DeviceBuffer<PCL24> dDensePCL24;
    DeviceBuffer<PCL48> dDensePCL48;

    // DeviceBuffer<int> dHitIsFinite;
    //     HostPinnedBuffer<int> hHitIsFinite;

    // MODEL STUFF
    std::unordered_map<std::string, std::shared_ptr<ModelInstance>> m_instances_map;
    bool needs_root_rebuild = { false };

    OptixTraversableHandle m_root; // Scene root

    // Buffer that keeps the (final, compacted) accel structure
    CUDABuffer accelerationStructure;

    std::vector<cudaArray_t> textureArrays;
    std::vector<cudaTextureObject_t> textureObjects;

    std::vector<std::thread> threads;

public:
    void softReset() {
        m_instances_map.clear();
        needs_root_rebuild = true;
    }

};
