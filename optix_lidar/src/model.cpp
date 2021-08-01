#include "model.h"
#include <LaunchParams.h>
#include <memory.h>

using namespace gdt;

ModelInstance::ModelInstance(std::shared_ptr<TriangleMesh> mesh)
    : m_triangle_mesh(mesh)
{
    m_vertex_buffer.alloc_and_upload(mesh->vertex);
    m_index_buffer.alloc_and_upload(mesh->index);
    if (!mesh->normal.empty())
        m_normal_buffer.alloc_and_upload(mesh->normal);
    if (!mesh->texcoord.empty())
        m_texcoord_buffer.alloc_and_upload(mesh->texcoord);

    triangleInput = {};
    triangleInput.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

    d_vertices = m_vertex_buffer.d_pointer();
    d_indices = m_index_buffer.d_pointer();

    triangleInput.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
    triangleInput.triangleArray.vertexStrideInBytes = sizeof(vec3f);
    triangleInput.triangleArray.numVertices = (int)mesh->vertex.size();
    triangleInput.triangleArray.vertexBuffers = &d_vertices;

    triangleInput.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
    triangleInput.triangleArray.indexStrideInBytes = sizeof(vec3i);
    triangleInput.triangleArray.numIndexTriplets = (int)mesh->index.size();
    triangleInput.triangleArray.indexBuffer = d_indices;

    triangleInputFlags = OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;

    triangleInput.triangleArray.flags = &triangleInputFlags;
    triangleInput.triangleArray.numSbtRecords = 1;
    triangleInput.triangleArray.sbtIndexOffsetBuffer = 0;
    triangleInput.triangleArray.sbtIndexOffsetSizeInBytes = 0;
    triangleInput.triangleArray.sbtIndexOffsetStrideInBytes = 0;
}

void ModelInstance::updateMesh(std::shared_ptr<TriangleMesh> mesh)
{
    m_triangle_mesh = mesh;
    needs_rebuild = true;
}

OptixTraversableHandle ModelInstance::buildGAS(OptixDeviceContext optixContext)
{
    OptixAccelBuildOptions accelOptions = {};
    accelOptions.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE | OPTIX_BUILD_FLAG_ALLOW_UPDATE | OPTIX_BUILD_FLAG_ALLOW_COMPACTION;
    accelOptions.motionOptions.numKeys = 0; // no motion blur

    if (needs_rebuild)
        accelOptions.operation = OPTIX_BUILD_OPERATION_UPDATE;
    else
        accelOptions.operation = OPTIX_BUILD_OPERATION_BUILD;

    OptixAccelBufferSizes blasBufferSizes;
    OPTIX_CHECK(optixAccelComputeMemoryUsage(optixContext,
        &accelOptions,
        &triangleInput,
        1, // num_build_inputs
        &blasBufferSizes));

    // ==================================================================
    // prepare compaction
    // ==================================================================

    CUDABuffer compactedSizeBuffer;
    compactedSizeBuffer.alloc(sizeof(uint64_t));

    OptixAccelEmitDesc emitDesc;
    emitDesc.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
    emitDesc.result = compactedSizeBuffer.d_pointer();

    // ==================================================================
    // execute build (main stage)
    // ==================================================================

    CUDABuffer tempBuffer;
    tempBuffer.alloc(blasBufferSizes.tempSizeInBytes);

    if (!needs_rebuild) {
        // for update we use existing buffer
        // in rebuild it should not change size
        // if size changes, build from begining
        outputBuffer.free();
        outputBuffer.alloc(blasBufferSizes.outputSizeInBytes);
    }

    OPTIX_CHECK(optixAccelBuild(optixContext,
        /* stream */ 0,
        &accelOptions,
        &triangleInput,
        1,
        tempBuffer.d_pointer(),
        tempBuffer.sizeInBytes,

        outputBuffer.d_pointer(),
        outputBuffer.sizeInBytes,

        &meshHandle,

        &emitDesc, 1));
    CUDA_SYNC_CHECK();

    uint64_t compactedSize;
    compactedSizeBuffer.download(&compactedSize, 1);

    asBuffer.free();
    asBuffer.alloc(compactedSize);
    OPTIX_CHECK(optixAccelCompact(optixContext,
        /*stream:*/ 0,
        meshHandle,
        asBuffer.d_pointer(),
        asBuffer.sizeInBytes,
        &meshHandle));
    CUDA_SYNC_CHECK();

    tempBuffer.free();
    compactedSizeBuffer.free();
    needs_rebuild = false;

    return meshHandle;
}

OptixInstance ModelInstance::buildIAS(unsigned int id)
{
    instance = {};
    memcpy(instance.transform, m_triangle_mesh->transform.matrix_flat, sizeof(float) * 12);
    // auto t = m_triangle_mesh->transform.matrix_flat;
    // std::cout << "Transform: \n"
    //     << t[0] << " " << t[1] << " " << t[2] << " " << t[3] << std::endl
    //     << t[4] << " " << t[5] << " " << t[6] << " " << t[7] << std::endl
    //     << t[8] << " " << t[9] << " " << t[10] << " " << t[11] << std::endl;
    instance.instanceId = id;
    instance.visibilityMask = 255;
    // TODO(prybicki): this assumes a single SBT record per GAS
    instance.sbtOffset = id * LIDAR_RAY_TYPE_COUNT; // This controls the SBT instance offset!
    instance.flags = OPTIX_INSTANCE_FLAG_DISABLE_ANYHIT;
    instance.traversableHandle = meshHandle;
    return instance;
}

ModelInstance::~ModelInstance()
{
    m_vertex_buffer.free();
    m_normal_buffer.free();
    m_texcoord_buffer.free();
    m_index_buffer.free();
    outputBuffer.free();
}
