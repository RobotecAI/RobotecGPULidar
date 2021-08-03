#pragma once

#include <memory>
#include <unordered_map>

#include "TriangleMesh.h"
#include "Texture.h"
#include "optix.h"
#include "utils/CUDABuffer.h"

class ModelInstance {
public:
    ModelInstance(std::shared_ptr<TriangleMesh> mesh);
    ~ModelInstance();
    OptixTraversableHandle buildGAS(OptixDeviceContext optixContext);
    OptixInstance buildIAS(unsigned int id);
    void updateMesh(std::shared_ptr<TriangleMesh> mesh);

    //(TODO) make a map since one model can contain multiple meshes
    std::shared_ptr<TriangleMesh> m_triangle_mesh;
    std::shared_ptr<Texture> m_texture;

    bool needs_rebuild { false };

    CUDABuffer m_vertex_buffer;
    CUDABuffer m_normal_buffer;
    CUDABuffer m_texcoord_buffer;
    CUDABuffer m_index_buffer;

private:
    OptixTraversableHandle _GAS_handle;
    OptixBuildInput triangleInput;
    CUDABuffer outputBuffer;
    CUDABuffer asBuffer;
    OptixInstance instance;
    OptixTraversableHandle meshHandle;
    CUdeviceptr d_vertices;
    CUdeviceptr d_indices;
    CUdeviceptr d_transforms;
    uint32_t triangleInputFlags;
};

typedef std::unordered_map<std::string, std::shared_ptr<ModelInstance>> InstancesMap;