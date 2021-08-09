#pragma once

#include <memory>
#include <unordered_map>

#include "Texture.h"
#include "TriangleMesh.h"
#include "optix.h"
#include "utils/CUDABuffer.h"

class ModelInstance {
public:
    ModelInstance(std::shared_ptr<TriangleMesh> mesh, OptixDeviceContext optixContext);
    ~ModelInstance();

    OptixInstance buildIAS(unsigned int id);

    CUDABuffer m_vertex_buffer;
    CUDABuffer m_normal_buffer;
    CUDABuffer m_texcoord_buffer;
    CUDABuffer m_index_buffer;
    std::shared_ptr<TriangleMesh> m_triangle_mesh;
private:
    CUDABuffer compactedGASBuffer;
    OptixTraversableHandle meshHandle;
};
