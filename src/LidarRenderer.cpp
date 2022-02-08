#include <memory.h>

#include <fmt/color.h>
#include <optix_function_table_definition.h> //this include may only appear in a single source file

#include "LidarRenderer.h"
#include "PerfProbe.h"
#include "ShaderBindingTableTypes.h"

extern "C" char embedded_ptx_code[];

#define OPTIX_LOG_LEVEL_NONE 0
#define OPTIX_LOG_LEVEL_FATAL 1
#define OPTIX_LOG_LEVEL_ERROR 2
#define OPTIX_LOG_LEVEL_WARN 3
#define OPTIX_LOG_LEVEL_INFO 4

// TODO: prybicki: this should not be called multiple times
static CUcontext getCurrentDeviceContext()
{
    const char* error = nullptr;
    CUresult status;

    cudaFree(0); // Force CUDA runtime initialization

    CUdevice device;
    status = cuDeviceGet(&device, 0);
    if (status != CUDA_SUCCESS) {
        cuGetErrorString(status, &error);
        throw std::runtime_error(format("failed to get current CUDA device: {} ({})\n", error, status));
    }

    CUcontext cudaContext = nullptr;
    CUresult primaryCtxStatus = cuDevicePrimaryCtxRetain(&cudaContext, device);
    if (primaryCtxStatus != CUDA_SUCCESS) {
        cuGetErrorString(status, &error);
        throw std::runtime_error(format("failed to get primary CUDA context: {} ({})\n", error, status));
    }
    assert(cudaContext != nullptr);
    return cudaContext;
}

OptiXInitializationGuard::OptiXInitializationGuard()
{
    OPTIX_CHECK(optixInit());
    OPTIX_CHECK(optixDeviceContextCreate(getCurrentDeviceContext(), nullptr, &context));

    auto logCallback = [](unsigned level, const char* tag, const char* message, void*) {
        auto fmt = "[RGL][OptiX][{:2}][{:^12}]: {}\n";
        if (level == OPTIX_LOG_LEVEL_FATAL || level == OPTIX_LOG_LEVEL_ERROR) {
            logError(fmt, level, tag, message);
            return;
        }
        if (level == OPTIX_LOG_LEVEL_WARN) {
            logWarn(fmt, level, tag, message);
            return;
        }
        logInfo(fmt, level, tag, message);
    };

    OPTIX_CHECK(optixDeviceContextSetLogCallback(context, logCallback, nullptr, 4));
}

OptiXInitializationGuard::~OptiXInitializationGuard()
{
    if (context) {
        optixDeviceContextDestroy(context);
    }
}

LidarRenderer::LidarRenderer()
  : optix()
  , dRayCountOfLidar("dRayCountOfLidar")
  , dRayDirs("dRayDirs")
  , dRangeOfLidar("dRangeOfLidar")
  , dPositionOfLidar("dPositionOfLidar")
  , dLaunchParams("dLaunchParams")
  , dHitXYZI("dHitXYZI")
  , hHitXYZI("hHitXYZI")
  , dHitIsFinite("dHitIsFinite")
  , hHitIsFinite("hHitIsFinite")
{
    PerfProbe c("init");
    logInfo("[RGL] Initialization started\n");
    logInfo("[RGL] Running on GPU: {}\n", getCurrentDeviceName());
    logInfo("[RGL] PID: {}\n", getpid());

    initializeStaticOptixStructures();

    logInfo("[RGL] Initialization finished successfully\n");
}

LidarRenderer::~LidarRenderer()
{
    logInfo("[RGL] Deinitialization started\n");

    optixPipelineDestroy(pipeline);
    for (auto&& programGroup : { raygenPG, missPG, hitgroupPG }) {
        optixProgramGroupDestroy(programGroup);
    }

    for (auto&& texture_object : textureObjects) {
        cudaDestroyTextureObject(texture_object);
    }

    for (auto&& texture_array : textureArrays) {
        cudaFreeArray(texture_array);
    }

    optixModuleDestroy(module);


    logInfo("[RGL] Deinitialization finished successfully\n");
}

void LidarRenderer::addMeshUnchecked(std::shared_ptr<TriangleMesh> mesh)
{
    logInfo("[RGL] Adding mesh id={}\n", mesh->mesh_id);
    auto modelInstance = std::make_shared<ModelInstance>(mesh, optix.context);
    m_instances_map[mesh->mesh_id] = modelInstance;
    needs_root_rebuild = true;
}

void LidarRenderer::updateMeshTransform(const std::string& mesh_id, const TransformMatrix& transform)
{
    if (m_instances_map.find(mesh_id) == m_instances_map.end()) {
        return;
    }
    PerfProbe c("updateMeshTransform");
    m_instances_map[mesh_id]->m_triangle_mesh->transform = transform;
    needs_root_rebuild = true;
}

void LidarRenderer::removeMesh(const std::string& id)
{
    if (m_instances_map.find(id) == m_instances_map.end()) {
        return;
    }

    m_instances_map.erase(id);
    needs_root_rebuild = true;
}

void LidarRenderer::updateStructsForModel()
{
    if (needs_root_rebuild) {
        hLaunchParams.traversable = buildAccel();
        sbt = buildSBT();
    }
    needs_root_rebuild = false;
}

OptixTraversableHandle LidarRenderer::buildAccel()
{
    std::vector<OptixInstance> instances;
    unsigned int idx = 0;
    for (const auto& kv : m_instances_map) {
        const auto mi = kv.second;
        auto instance = mi->buildIAS(idx++);
        instances.push_back(instance);
    }

    const size_t instancesSizeInBytes = sizeof(OptixInstance) * instances.size();

    CUDABuffer instanceBuffer;
    instanceBuffer.alloc(instancesSizeInBytes);
    instanceBuffer.upload(instances.data(), instances.size());

    OptixBuildInput instanceInput = {};

    instanceInput.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES;
    instanceInput.instanceArray.instances = instanceBuffer.d_pointer();
    instanceInput.instanceArray.numInstances = (unsigned int)instances.size();

    OptixAccelBuildOptions accelBuildOptions = {};

    accelBuildOptions.buildFlags = OPTIX_BUILD_FLAG_ALLOW_UPDATE | OPTIX_BUILD_FLAG_ALLOW_COMPACTION;
    accelBuildOptions.operation = OPTIX_BUILD_OPERATION_BUILD;

    OptixAccelBufferSizes iasBufferSizes = {};
    OPTIX_CHECK(optixAccelComputeMemoryUsage(optix.context, &accelBuildOptions, &instanceInput, 1, &iasBufferSizes));

    CUDABuffer tempBuffer;
    tempBuffer.alloc(iasBufferSizes.tempSizeInBytes);

    CUDABuffer outputBuffer;
    outputBuffer.alloc(iasBufferSizes.outputSizeInBytes);

    CUDABuffer compactedSizeBuffer;
    compactedSizeBuffer.alloc(sizeof(uint64_t));

    OptixAccelEmitDesc emitDesc;
    emitDesc.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
    emitDesc.result = compactedSizeBuffer.d_pointer();

    OPTIX_CHECK(optixAccelBuild(optix.context, 0,
        &accelBuildOptions, &instanceInput, 1,
        tempBuffer.d_pointer(), tempBuffer.sizeInBytes,
        outputBuffer.d_pointer(), outputBuffer.sizeInBytes,
        &m_root, &emitDesc, 1));

    // perform compaction
    uint64_t compactedSize;
    compactedSizeBuffer.download(&compactedSize, 1);

    accelerationStructure.free();
    accelerationStructure.alloc(compactedSize);
    OPTIX_CHECK(optixAccelCompact(optix.context,
        nullptr,
        m_root,
        accelerationStructure.d_pointer(),
        accelerationStructure.sizeInBytes,
        &m_root));
    CUDA_SYNC_CHECK();

    // clean up
    tempBuffer.free();
    outputBuffer.free();
    compactedSizeBuffer.free();
    instanceBuffer.free();

    return m_root;
}

void LidarRenderer::initializeStaticOptixStructures()
{
    OptixModuleCompileOptions moduleCompileOptions = {
        .maxRegisterCount = 100,
        .optLevel = OPTIX_COMPILE_OPTIMIZATION_LEVEL_2,
        .debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_LINEINFO
    };

    OptixPipelineCompileOptions pipelineCompileOptions = {
        .usesMotionBlur = false,
        .traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY,
        .numPayloadValues = 4,
        .numAttributeValues = 2,
        .exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE,
        .pipelineLaunchParamsVariableName = "optixLaunchLidarParams",
    };

    OptixPipelineLinkOptions pipelineLinkOptions = {
        .maxTraceDepth = 2,
        .debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_DEFAULT,
    };

    module = {};
    OPTIX_CHECK(optixModuleCreateFromPTX(optix.context,
        &moduleCompileOptions,
        &pipelineCompileOptions,
        embedded_ptx_code,
        strlen(embedded_ptx_code),
        nullptr, nullptr,
        &module));

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc raygenDesc = {
        .kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN,
        .raygen = {
            .module = module,
            .entryFunctionName = "__raygen__renderLidar" }
    };

    OPTIX_CHECK(optixProgramGroupCreate(
        optix.context, &raygenDesc, 1, &pgOptions, nullptr, nullptr, &raygenPG));

    OptixProgramGroupDesc missDesc = {
        .kind = OPTIX_PROGRAM_GROUP_KIND_MISS,
        .miss = {
            .module = module,
            .entryFunctionName = "__miss__lidar" },
    };

    OPTIX_CHECK(optixProgramGroupCreate(
        optix.context, &missDesc, 1, &pgOptions, nullptr, nullptr, &missPG));

    OptixProgramGroupDesc hitgroupDesc = {
        .kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP,
        .hitgroup = {
            .moduleCH = module,
            .entryFunctionNameCH = "__closesthit__lidar",
            .moduleAH = module,
            .entryFunctionNameAH = "__anyhit__lidar",
        }
    };

    OPTIX_CHECK(optixProgramGroupCreate(
        optix.context, &hitgroupDesc, 1, &pgOptions, nullptr, nullptr, &hitgroupPG));

    OptixProgramGroup programGroups[] = { raygenPG, missPG, hitgroupPG };

    OPTIX_CHECK(optixPipelineCreate(
        optix.context,
        &pipelineCompileOptions,
        &pipelineLinkOptions,
        programGroups,
        sizeof(programGroups) / sizeof(programGroups[0]),
        nullptr, nullptr,
        &pipeline));

    OPTIX_CHECK(optixPipelineSetStackSize(
        pipeline,
        2 * 1024, // directCallableStackSizeFromTraversal
        2 * 1024, // directCallableStackSizeFromState
        2 * 1024, // continuationStackSize
        3 // maxTraversableGraphDepth
        ));
}

OptixShaderBindingTable LidarRenderer::buildSBT()
{
    static DeviceBuffer<HitgroupRecord> dHitgroupRecords("hitgroupRecord");
    static DeviceBuffer<RaygenRecord> dRaygenRecords("raygenRecord");
    static DeviceBuffer<MissRecord> dMissRecords("missRecord");

    std::vector<HitgroupRecord> hHitgroupRecords;
    for (const auto& kv : m_instances_map) {
        auto mesh = kv.second->m_triangle_mesh;
        auto instance = kv.second;
        bool hasTexture = (mesh->diffuseTextureID >= 0);
        hHitgroupRecords.emplace_back();

        HitgroupRecord* hr = &(*hHitgroupRecords.rbegin());
        OPTIX_CHECK(optixSbtRecordPackHeader(hitgroupPG, hr));
        hr->data = TriangleMeshSBTData {
            // .color = mesh->diffuse, // Temporarily comment out to make the struct POD, not used in device code
            .vertex = reinterpret_cast<vec3f*>(instance->m_vertex_buffer.d_pointer()),
            .normal = reinterpret_cast<vec3f*>(instance->m_normal_buffer.d_pointer()),
            .texcoord = reinterpret_cast<vec2f*>(instance->m_texcoord_buffer.d_pointer()),
            .index = reinterpret_cast<vec3i*>(instance->m_index_buffer.d_pointer()),
            .vertex_count = instance->m_vertex_buffer.sizeInBytes / sizeof(vec3f),
            .index_count = instance->m_index_buffer.sizeInBytes / sizeof(vec3i),
            .normal_count = instance->m_normal_buffer.sizeInBytes / sizeof(vec3f),
            .texcoord_count = instance->m_texcoord_buffer.sizeInBytes / sizeof(vec2f),
            .hasTexture = hasTexture,
            .texture = hasTexture ? textureObjects[mesh->diffuseTextureID] : 0,
        };
    }
    dHitgroupRecords.copyFromHost(hHitgroupRecords);

    RaygenRecord hRaygenRecord;
    OPTIX_CHECK(optixSbtRecordPackHeader(raygenPG, &hRaygenRecord));
    dRaygenRecords.copyFromHost(&hRaygenRecord, 1);

    MissRecord hMissRecord;
    OPTIX_CHECK(optixSbtRecordPackHeader(missPG, &hMissRecord));
    dMissRecords.copyFromHost(&hMissRecord, 1);

    return OptixShaderBindingTable {
        .raygenRecord = dRaygenRecords.readDeviceRaw(),
        .missRecordBase = dMissRecords.readDeviceRaw(),
        .missRecordStrideInBytes = sizeof(MissRecord),
        .missRecordCount = 1U,
        .hitgroupRecordBase = dHitgroupRecords.readDeviceRaw(),
        .hitgroupRecordStrideInBytes = sizeof(HitgroupRecord),
        .hitgroupRecordCount = static_cast<unsigned>(dHitgroupRecords.getElemCount()),
    };
}

void LidarRenderer::render(const std::vector<LidarSource>& lidars)
{
    static int renderCallIdx = 0;
    PerfProbe c("render");

    logInfo("[RGL] render start]\n");

    if (!ensureBuffersPreparedBeforeRender(lidars)) {
        return;
    }

    if (m_instances_map.size() == 0) {
        logWarn("[RGL] LidarRender::render called with 0 meshes\n");
        return;
    }

    updateStructsForModel();

    {
        PerfProbe ccc("render-gpu");
        OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
            pipeline, nullptr,
            /*! parameters and SBT */
            dLaunchParams.readDeviceRaw(),
            dLaunchParams.getByteSize(),
            &sbt,
            /*! dimensions of the launch: */
            hLaunchParams.rayCount,
            1,
            1));
    }

    {
        PerfProbe cc("download-memcpy");
        hHitXYZI.copyFromDeviceAsync(dHitXYZI);
        hHitIsFinite.copyFromDeviceAsync(dHitIsFinite);
    }

    renderCallIdx++;
    if (renderCallIdx % 200 == 0) {
        PerfProbe::saveToFileAndReset();
    }
}

bool LidarRenderer::ensureBuffersPreparedBeforeRender(const std::vector<LidarSource>& lidars)
{
    PerfProbe c("updateLidarSource");

    // ########## Ray Count of LiDAR [0] ########## //

    // TODO(prybicki): urgent optimizations and bold assumptions here :)
    if (lidars.size() != 1) {
        auto message = fmt::format("fixme: unexpected number of lidars: {}\n", lidars.size());
        throw std::logic_error(message);
    }

    hRayCountOfLidar.resize((lidars.size()));
    hRayCountOfLidar[0] = static_cast<int>(lidars[0].directions.size());

    auto totalRayCount = hRayCountOfLidar[0];
    if (totalRayCount == 0) {
        logWarn("[RGL] LidarRender::render called with 0 rays\n");
        return false;
    }
    dRayCountOfLidar.copyFromHost(hRayCountOfLidar);

    // ########## Ray directions of LiDAR [0] ########## //
    // Using the aforementioned assumption, upload rays directly from the given buffer.
    // TODO: after typed buffers are introduced, it should be easy to remove the aforementioned assumption
    dRayDirs.copyFromHost(lidars[0].directions);

    // ########## Range of LiDAR [ 0] ########## //
    dRangeOfLidar.copyFromHost(&lidars[0].range, 1UL);

    // ########## Position of LiDAR [0] ########## //
    dPositionOfLidar.copyFromHost(&lidars[0].source, 1UL);


    // ########## GPU Output for LiDAR[0] ########## //
    dHitXYZI.resizeToFit(totalRayCount);
    dHitIsFinite.resizeToFit(totalRayCount);

    hLaunchParams.rayCount = totalRayCount;
    hLaunchParams.lidarCount = lidars.size();
    hLaunchParams.rayCountOfLidar = dRayCountOfLidar.readDevice();
    hLaunchParams.rayDirs = dRayDirs.readDevice();
    hLaunchParams.rangeOfLidar = dRangeOfLidar.readDevice();
    hLaunchParams.positionOfLidar = dPositionOfLidar.readDevice();
    hLaunchParams.hitXYZI = dHitXYZI.writeDevice();
    hLaunchParams.hitIsFinite = dHitIsFinite.writeDevice();
    dLaunchParams.copyFromHost(&hLaunchParams, 1);

    return true;
}

const RaycastResults* LidarRenderer::downloadPoints()
{
    logInfo("[RGL] Downloading points\n");
    PerfProbe c("download");

    // Finish async downloads scheduled in render()
    CUDA_CHECK(StreamSynchronize(nullptr));
    assert(hRayCountOfLidar.size() == 1);
    result.resize(1);

    // TODO(prybicki) move it to the GPU
    {
        PerfProbe cc("download-rewrite");
        const auto* hHitIsFinitePtr = hHitIsFinite.readHost();
        const auto* hHitXYZIPtr = hHitXYZI.readHost();
        for (size_t j = 0; j < hHitIsFinite.getElemCount(); ++j) {
            if (!hHitIsFinitePtr[j]) {
                continue;
            }
            result[0].points.push_back(hHitXYZIPtr[j]);
        }
    }

    return &result;
}

std::string LidarRenderer::getCurrentDeviceName()
{
    int currentDevice = -1;
    cudaDeviceProp deviceProperties {};
    CUDA_CHECK(GetDevice(&currentDevice));
    CUDA_CHECK(GetDeviceProperties(&deviceProperties, currentDevice));
    return std::string(deviceProperties.name);
}

void LidarRenderer::addMeshRaw(const char *meshID, int meshSize, vec3f *vertices, vec3f *normals, vec2f *texCoords,
                               int indicesSize, vec3i *indices, int transformSize, float *transform) {
    if (transformSize != sizeof(TransformMatrix) / sizeof(float)) {
        logError("[RGL] Invalid transform size: {} (expected {})\n", transformSize, sizeof(TransformMatrix) / sizeof(float));
        throw std::invalid_argument("invalid transform size");
    }

    // Since constructing TriangleMesh is costly, we want to check for its existence early.
    if (m_instances_map.find(meshID) != m_instances_map.end()) {
        return; // Already in the map, skip it.
    }

    // TODO(prybicki): upload directly to the GPU
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

    addMeshUnchecked({tm});
}

void LidarRenderer::removeMeshRawTmp(const char *meshID) {
    removeMesh(std::string(meshID));
}

void LidarRenderer::updateMeshTransformRawTmp(char *meshID, float *transform, int transformSize) {
    if (transformSize != sizeof(TransformMatrix) / sizeof(float)) {
        auto message = fmt::format("Invalid transform size: {} (expected {})\n", transformSize, sizeof(TransformMatrix) / sizeof(float));
        throw std::invalid_argument(message);
    }

    TransformMatrix transformMatrix;
    memcpy(transformMatrix.matrix_flat, transform, sizeof(TransformMatrix));
    updateMeshTransform(std::string(meshID), transformMatrix);
}
