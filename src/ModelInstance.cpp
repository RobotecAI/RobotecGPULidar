#pragma once

#include <cstring>

#include "LaunchParams.h"
#include "ModelInstance.h"
#include "gdt/math/vec.h"

using namespace gdt;

ModelInstance::ModelInstance(std::shared_ptr<TriangleMesh> mesh, OptixDeviceContext optixContext)
    : m_triangle_mesh(mesh)
{
    m_vertex_buffer.alloc_and_upload(mesh->vertex);
    m_index_buffer.alloc_and_upload(mesh->index);
    if (!mesh->normal.empty())
        m_normal_buffer.alloc_and_upload(mesh->normal);
    if (!mesh->texcoord.empty())
        m_texcoord_buffer.alloc_and_upload(mesh->texcoord);

    CUdeviceptr d_vertices = m_vertex_buffer.d_pointer();
    CUdeviceptr d_indices = m_index_buffer.d_pointer();
    unsigned triangleInputFlags = OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;

    OptixBuildInput triangleInput = {
        .type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES,
        .triangleArray = {
            .vertexBuffers = &d_vertices,
            .numVertices = static_cast<unsigned>(mesh->vertex.size()),
            .vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3,
            .vertexStrideInBytes = sizeof(vec3f),
            .indexBuffer = d_indices,
            .numIndexTriplets = static_cast<unsigned>(mesh->index.size()),
            .indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3,
            .indexStrideInBytes = sizeof(vec3i),
            .flags = &triangleInputFlags,
            .numSbtRecords = 1,
            .sbtIndexOffsetBuffer = 0,
            .sbtIndexOffsetSizeInBytes = 0,
            .sbtIndexOffsetStrideInBytes = 0,
        }
    };

    OptixAccelBuildOptions accelOptions = {
        .buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE | OPTIX_BUILD_FLAG_ALLOW_UPDATE | OPTIX_BUILD_FLAG_ALLOW_COMPACTION,
        .operation = OPTIX_BUILD_OPERATION_BUILD
    };

    OptixAccelBufferSizes blasBufferSizes;
    OPTIX_CHECK(optixAccelComputeMemoryUsage(optixContext,
                                             &accelOptions,
                                             &triangleInput, 1,
                                             &blasBufferSizes));

    // prepare compaction
    CUDABuffer compactedSizeBuffer, tempGASBuffer, fullGASBuffer;
    tempGASBuffer.alloc(blasBufferSizes.tempSizeInBytes);
    fullGASBuffer.alloc(blasBufferSizes.outputSizeInBytes);
    compactedSizeBuffer.alloc(sizeof(uint64_t));

    OptixAccelEmitDesc emitDesc = {
        .result = compactedSizeBuffer.d_pointer(),
        .type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE,
    };

    // execute build (main stage)
    OPTIX_CHECK(optixAccelBuild(optixContext,
                                nullptr,
                                &accelOptions,
                                &triangleInput, 1,
                                tempGASBuffer.d_pointer(),
                                tempGASBuffer.sizeInBytes,
                                fullGASBuffer.d_pointer(),
                                fullGASBuffer.sizeInBytes,
                                &meshHandle,
                                &emitDesc, 1));
    CUDA_SYNC_CHECK();

    uint64_t compactedSize;
    compactedSizeBuffer.download(&compactedSize, 1);

    compactedGASBuffer.alloc(compactedSize);
    OPTIX_CHECK(optixAccelCompact(optixContext,
                                  nullptr,
                                  meshHandle,
                                  compactedGASBuffer.d_pointer(),
                                  compactedGASBuffer.sizeInBytes,
                                  &meshHandle));
    CUDA_SYNC_CHECK();

    fullGASBuffer.free();
    tempGASBuffer.free();
    compactedSizeBuffer.free();
}

OptixInstance ModelInstance::buildIAS(unsigned int id)
{
    // NOTE: this assumes a single SBT record per GAS
    OptixInstance instance = {
        .instanceId = id,
        .sbtOffset = id * LIDAR_RAY_TYPE_COUNT,
        .visibilityMask = 255,
        .flags = OPTIX_INSTANCE_FLAG_DISABLE_ANYHIT,
        .traversableHandle = meshHandle,
    };
    memcpy(instance.transform, m_triangle_mesh->transform.matrix_flat, sizeof(float) * 12);
    return instance;
}

ModelInstance::~ModelInstance()
{
    m_vertex_buffer.free();
    m_normal_buffer.free();
    m_texcoord_buffer.free();
    m_index_buffer.free();
}
