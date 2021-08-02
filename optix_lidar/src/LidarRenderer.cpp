#include <memory.h>

#include <fmt/color.h>
#include <optix_function_table_definition.h> //this include may only appear in a single source file

#include "LidarRenderer.h"
#include "PerfProbe.h"
#include "ShaderBindingTableTypes.h"

using namespace fmt;

extern "C" char embedded_ptx_code[];

template<typename... Args>
void log(Args&&... args) { print(stderr, std::forward<Args>(args)...); }

LidarRenderer::LidarRenderer()
{
    PerfProbe c("init");
    log("[RGL] Initialization started\n");
    log("[RGL] Running on GPU: {}\n", getCurrentDeviceName());

    OPTIX_CHECK(optixInit());
    OPTIX_CHECK(optixDeviceContextCreate(getCurrentDeviceContext(), nullptr, &optixContext));
    OPTIX_CHECK(optixDeviceContextSetLogCallback(optixContext,
        [](unsigned level, const char* tag, const char* message, void*) {
            print(stderr, fg(color::red), "[{:2}][{:^12}]: {}\n", (int) level, tag, message);
        },
        nullptr, 4
    ));

    initializeStaticOptixStructures();
    launchParamsBuffer.alloc(sizeof(launchParams));
    log(fg(color::green), "[RGL] Initialization finished successfully\n");
}

LidarRenderer::~LidarRenderer()
{
    log("[RGL] Deinitialization started\n");

    optixPipelineDestroy(pipeline);
    for (auto&& programGroup : {raygenPG, missPG, hitgroupPG}) {
        optixProgramGroupDestroy(programGroup);
    }

    for (auto&& texture_object : textureObjects) {
        cudaDestroyTextureObject(texture_object);
    }

    for (auto&& texture_array : textureArrays) {
        cudaFreeArray(texture_array);
    }

    optixModuleDestroy(module);
    optixDeviceContextDestroy(optixContext);

    log(fg(color::green), "[RGL] Deinitialization finished successfully\n");
}

void LidarRenderer::addTextures(std::vector<std::shared_ptr<Texture>> textures)
{
    // if (textures.size() == 0) {
    //   return;
    // }

    // for (auto texture : textures) {
    //   model.textures_map[texture->texture_id] = texture;
    // }

    // model.textures_changed = true;
    // model.needs_rebuild = true;
    // model.changed = true;
}

void LidarRenderer::addMeshes(std::vector<std::shared_ptr<TriangleMesh>> meshes)
{
    //std::cout << "adding meshes count: " << meshes.size() << std::endl;
    if (meshes.size() == 0) {
        return;
    }

    {
        PerfProbe c("add-mesh");
        for (auto mesh : meshes) {
            if (m_instances_map.find(mesh->mesh_id) == m_instances_map.end()) {
                // std::cout << "Mesh " << mesh->mesh_id << " not in map, creating... " << std::endl;
                auto modelInstance = std::make_shared<ModelInstance>(mesh);
                modelInstance->buildGAS(optixContext);
                m_instances_map[mesh->mesh_id] = modelInstance;
                needs_root_rebuild = true;
            }
        }
    }
}

void LidarRenderer::updateMeshTransform(const std::string& mesh_id, const TransformMatrix& transform)
{
    if (m_instances_map.find(mesh_id) != m_instances_map.end()) {
        PerfProbe cc("update-mesh");
        m_instances_map[mesh_id]->m_triangle_mesh->transform = transform;
        m_instances_map[mesh_id]->needs_rebuild = true;
        needs_root_rebuild = true;
    }
}

void LidarRenderer::removeMesh(const std::string& id)
{
    if (m_instances_map.find(id) == m_instances_map.end()) {
        return;
    }

    m_instances_map.erase(id);
    needs_root_rebuild = true;
}

void LidarRenderer::removeTexture(const std::string& id)
{
    // if (m_instances_map.find(id) != m_instances_map.end()) {
    //   return;
    // }
    // model.textures_map.erase(id);
    // model.needs_rebuild = true;
    // model.textures_changed = true;
    // model.changed = true;
}

void LidarRenderer::update_structs_for_model()
{
    // std::cout << "update structs for model " << std::endl;
    if (needs_root_rebuild) {
        //std::string rebuild = model.needs_rebuild ? " and needs rebuild" : "";
        //std::cout << "update structs: model changed" << rebuild << std::endl;
        launchParams.traversable = buildAccel();
        // if (model.textures_changed) {
        //   //std::cout << "update structs: textures_changed, creating " << std::endl;
        //   createTextures();
        //   model.textures_changed = false;
        // }
        // std::cout << "building SBT" << std::endl;
        buildSBT();

        // model.needs_rebuild = false;
    }
    needs_root_rebuild = false;
    //std::cout << "update structs for model ends " << std::endl;
}

void LidarRenderer::createTextures()
{
    // // std::cout << "create textures " << std::endl;
    // size_t numTextures = m_instances_map.size();

    // { // cleaning old textures. TODO - only clean ones that changed or were removed
    //   for (auto & texture_object : textureObjects) {
    //     CUDA_CHECK(DestroyTextureObject(texture_object));
    //   }
    //   for (auto & texture_array : textureArrays) {
    //     CUDA_CHECK(FreeArray(texture_array));
    //   }

    //   textureArrays.clear();
    //   textureObjects.clear();
    //   textureArrays.resize(numTextures);
    //   textureObjects.resize(numTextures);
    // }

    // int index = 0;
    // for (const auto & kv : m_instances_map) {
    //   auto texture = kv.second->m_texture;
    //   cudaResourceDesc res_desc = {};

    //   cudaChannelFormatDesc channel_desc;
    //   int32_t width = texture->resolution.x;
    //   int32_t height = texture->resolution.y;
    //   int32_t numComponents = 4;
    //   int32_t pitch = width*numComponents*sizeof(uint8_t);
    //   channel_desc = cudaCreateChannelDesc<uchar4>();

    //   cudaArray_t &pixelArray = textureArrays[index];
    //   CUDA_CHECK(MallocArray(&pixelArray,
    //                          &channel_desc,
    //                          width,height));

    //   CUDA_CHECK(Memcpy2DToArray(pixelArray,
    //                              /* offset */0,0,
    //                              texture->pixel,
    //                              pitch,pitch,height,
    //                              cudaMemcpyHostToDevice));

    //   res_desc.resType = cudaResourceTypeArray;
    //   res_desc.res.array.array = pixelArray;

    //   cudaTextureDesc tex_desc = {};
    //   tex_desc.addressMode[0] = cudaAddressModeWrap;
    //   tex_desc.addressMode[1] = cudaAddressModeWrap;
    //   tex_desc.filterMode = cudaFilterModeLinear;
    //   tex_desc.readMode = cudaReadModeNormalizedFloat;
    //   tex_desc.normalizedCoords = 1;
    //   tex_desc.maxAnisotropy = 1;
    //   tex_desc.maxMipmapLevelClamp = 99;
    //   tex_desc.minMipmapLevelClamp = 0;
    //   tex_desc.mipmapFilterMode = cudaFilterModePoint;
    //   tex_desc.borderColor[0] = 1.0f;
    //   tex_desc.sRGB = 0;

    //   // Create texture object
    //   cudaTextureObject_t cuda_tex = 0;
    //   CUDA_CHECK(CreateTextureObject(&cuda_tex, &res_desc, &tex_desc, nullptr));
    //   textureObjects[index] = cuda_tex;
    //   index++;
    // }
    //std::cout << "create textures ends " << std::endl;
}

OptixTraversableHandle LidarRenderer::buildAccel()
{
    std::vector<OptixInstance> instances;
    unsigned int idx = 0;
    for (const auto& kv : m_instances_map) {
        const auto mi = kv.second;
        // std::cout << "build ias " << kv.first << std::endl;
        auto instance = mi->buildIAS(idx++);
        // std::cout << "ias " << kv.first  << " done" << std::endl;
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
    OPTIX_CHECK(optixAccelComputeMemoryUsage(optixContext, &accelBuildOptions, &instanceInput, 1, &iasBufferSizes));

    CUDABuffer tempBuffer;
    tempBuffer.alloc(iasBufferSizes.tempSizeInBytes);

    CUDABuffer outputBuffer;
    outputBuffer.alloc(iasBufferSizes.outputSizeInBytes);

    CUDABuffer compactedSizeBuffer;
    compactedSizeBuffer.alloc(sizeof(uint64_t));

    OptixAccelEmitDesc emitDesc;
    emitDesc.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
    emitDesc.result = compactedSizeBuffer.d_pointer();

    OPTIX_CHECK(optixAccelBuild(optixContext, 0,
        &accelBuildOptions, &instanceInput, 1,
        tempBuffer.d_pointer(), tempBuffer.sizeInBytes,
        outputBuffer.d_pointer(), outputBuffer.sizeInBytes,
        &m_root, &emitDesc, 1));

    // ==================================================================
    // perform compaction
    // ==================================================================
    uint64_t compactedSize;
    compactedSizeBuffer.download(&compactedSize, 1);

    asBuffer.free();
    asBuffer.alloc(compactedSize);
    OPTIX_CHECK(optixAccelCompact(optixContext,
        /*stream:*/ 0,
        m_root,
        asBuffer.d_pointer(),
        asBuffer.sizeInBytes,
        &m_root));
    CUDA_SYNC_CHECK();

    // ==================================================================
    // aaaaaand .... clean up
    // ==================================================================
    tempBuffer.free();
    outputBuffer.free();
    compactedSizeBuffer.free();
    instanceBuffer.free();

    //std::cout << "build accel ends" << std::endl;
    return m_root;
}

/*! creates the module that contains all the programs we are going
      to use. in this simple example, we use a single module from a
      single .cu file, using a single embedded ptx string */
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
    OPTIX_CHECK(optixModuleCreateFromPTX(optixContext,
        &moduleCompileOptions,
        &pipelineCompileOptions,
        embedded_ptx_code,
        strlen(embedded_ptx_code),
        nullptr, nullptr,
        &module
    ));

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc raygenDesc = {
        .kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN,
        .raygen = {
            .module = module,
            .entryFunctionName = "__raygen__renderLidar"
        }
    };

    OPTIX_CHECK(optixProgramGroupCreate(
        optixContext,&raygenDesc, 1, &pgOptions, nullptr, nullptr, &raygenPG
    ));

    OptixProgramGroupDesc missDesc = {
        .kind = OPTIX_PROGRAM_GROUP_KIND_MISS,
        .miss = {
            .module = module,
            .entryFunctionName = "__miss__lidar"
        },
    };

    OPTIX_CHECK(optixProgramGroupCreate(
        optixContext, &missDesc, 1, &pgOptions, nullptr, nullptr, &missPG
    ));

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
        optixContext, &hitgroupDesc, 1,  &pgOptions, nullptr, nullptr, &hitgroupPG
    ));

    OptixProgramGroup programGroups[] = {raygenPG, missPG, hitgroupPG};

    OPTIX_CHECK(optixPipelineCreate(
        optixContext,
        &pipelineCompileOptions,
        &pipelineLinkOptions,
        programGroups,
        sizeof(programGroups) / sizeof(*programGroups),
        nullptr, nullptr,
        &pipeline
    ));

    OPTIX_CHECK(optixPipelineSetStackSize(
        pipeline,
        2 * 1024, // directCallableStackSizeFromTraversal
        2 * 1024, // directCallableStackSizeFromState
        2 * 1024, // continuationStackSize
        3         // maxTraversableGraphDepth
    ));
}

/*! constructs the shader binding table */
void LidarRenderer::buildSBT()
{
    sbt = {};
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<RaygenRecord> raygenRecords;
    RaygenRecord raygenRecord;
    OPTIX_CHECK(optixSbtRecordPackHeader(raygenPG, &raygenRecord));
    raygenRecords.push_back(raygenRecord);
    raygenRecordsBuffer.free();
    raygenRecordsBuffer.alloc_and_upload(raygenRecords);
    sbt.raygenRecord = raygenRecordsBuffer.d_pointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<MissRecord> missRecords;
    MissRecord missRecord;
    OPTIX_CHECK(optixSbtRecordPackHeader(missPG, &missRecord));
    missRecords.push_back(missRecord);
    missRecordsBuffer.free();
    missRecordsBuffer.alloc_and_upload(missRecords);
    sbt.missRecordBase = missRecordsBuffer.d_pointer();
    sbt.missRecordStrideInBytes = sizeof(MissRecord);
    sbt.missRecordCount = (int)missRecords.size();

    // ------------------------------------------------------------------
    // build hitgroup records
    // ------------------------------------------------------------------
    std::vector<HitgroupRecord> hitgroupRecords;
    int meshID = 0;
    for (const auto& kv : m_instances_map) {
        auto mesh = kv.second->m_triangle_mesh;
        auto instance = kv.second;
        HitgroupRecord rec;
        OPTIX_CHECK(optixSbtRecordPackHeader(hitgroupPG, &rec));
        rec.data.color = mesh->diffuse;
        if (mesh->diffuseTextureID >= 0) {
            rec.data.hasTexture = true;
            rec.data.texture = textureObjects[mesh->diffuseTextureID];
        } else {
            rec.data.hasTexture = false;
        }
        rec.data.index = (vec3i*)instance->m_index_buffer.d_pointer();
        rec.data.vertex = (vec3f*)instance->m_vertex_buffer.d_pointer();
        rec.data.normal = (vec3f*)instance->m_normal_buffer.d_pointer();
        rec.data.texcoord = (vec2f*)instance->m_texcoord_buffer.d_pointer();

        rec.data.index_count = instance->m_index_buffer.sizeInBytes / sizeof(vec3i);
        rec.data.vertex_count = instance->m_vertex_buffer.sizeInBytes / sizeof(vec3f);
        rec.data.normal_count = instance->m_normal_buffer.sizeInBytes / sizeof(vec3f);
        rec.data.texcoord_count = instance->m_texcoord_buffer.sizeInBytes / sizeof(vec2f);

        hitgroupRecords.push_back(rec);
        meshID++;
    }
    hitgroupRecordsBuffer.free();
    hitgroupRecordsBuffer.alloc_and_upload(hitgroupRecords);
    sbt.hitgroupRecordBase = hitgroupRecordsBuffer.d_pointer();
    sbt.hitgroupRecordStrideInBytes = sizeof(HitgroupRecord);
    sbt.hitgroupRecordCount = (int)hitgroupRecords.size();
}

/*! render one frame */
void LidarRenderer::render(std::vector<LidarSource>& lidars)
{
    static int renderCallIdx = 0;
    print("Rendering {} lidars\n", lidars.size());
    PerfProbe c("render");
    // sanity check: make sure we launch only after first resize is already done:
    if (launchParams.rayCount == 0) {
        print(fg(color::yellow), "LidarRender::render called with 0 rays\n");
        return;
    }
    if (m_instances_map.size() == 0) {
        print(fg(color::yellow), "LidarRender::render called with 0 meshes\n");
        return;
    }

    {
        PerfProbe cc("render-update-structs");
        update_structs_for_model();
    }

    {
        PerfProbe cc("render-upload-rays");
        uploadRays(lidars);
    }
    {
        PerfProbe cc("render-launch-params");
        launchParamsBuffer.uploadAsync(&launchParams, 1);
    }

    {
        PerfProbe cc("render-gpu");
        OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
            pipeline, nullptr,
            /*! parameters and SBT */
            launchParamsBuffer.d_pointer(),
            launchParamsBuffer.sizeInBytes,
            &sbt,
            /*! dimensions of the launch: */
            launchParams.rayCount,
            1,
            1));
    }

    // Enqueue asynchronous download:
    {
        PerfProbe cc("download-resize");
        allPoints.resize(launchParams.rayCount * 4);
        hits.resize(launchParams.rayCount);
        raysPerLidar.resize(launchParams.lidarCount);
    }

    {
        PerfProbe cc("download-memcpy");
        positionBuffer.downloadAsync(allPoints.data(), launchParams.rayCount * 4);
        hitBuffer.downloadAsync(hits.data(), launchParams.rayCount);
        raysPerLidarBuffer.downloadAsync(raysPerLidar.data(), launchParams.lidarCount);
    }

    renderCallIdx++;
    if (renderCallIdx % 200 == 0) {
	    PerfProbe::saveToFileAndReset();
    }
}

/*! resize frame buffer to given resolution */
void LidarRenderer::resize(std::vector<LidarSource>& lidars)
{
    PerfProbe c("resize");
    int lidarCount = lidars.size();
    int rayCount = 0;
    for (int i = 0; i < lidarCount; ++i) {
        rayCount += lidars[i].directions.size();
    }

    // resize our cuda frame buffer
    raysPerLidarBuffer.resize(lidarCount * sizeof(int));
    rayBuffer.resize(rayCount * 3 * sizeof(float));
    rangeBuffer.resize(lidarCount * sizeof(float));
    sourceBuffer.resize(lidarCount * 3 * sizeof(float));

    positionBuffer.resize(rayCount * 4 * sizeof(float));
    hitBuffer.resize(rayCount * sizeof(int));

    // update the launch parameters that we'll pass to the optix
    // launch:
    launchParams.rayCount = rayCount;
    launchParams.lidarCount = lidarCount;
    launchParams.raysPerLidarBuffer = (int*)raysPerLidarBuffer.d_ptr;
    launchParams.rayBuffer = (float*)rayBuffer.d_ptr;
    launchParams.rangeBuffer = (float*)rangeBuffer.d_ptr;
    launchParams.sourceBuffer = (float*)sourceBuffer.d_ptr;
    launchParams.positionBuffer = (float*)positionBuffer.d_ptr;
    launchParams.hitBuffer = (int*)hitBuffer.d_ptr;
}

void LidarRenderer::uploadRays(std::vector<LidarSource>& lidars)
{
    raysPerLidar.resize(lidars.size());
    for (size_t i = 0; i < lidars.size(); ++i) {
        raysPerLidar[i] = lidars[i].directions.size();
    }
    raysPerLidarBuffer.uploadAsync(raysPerLidar.data(), raysPerLidar.size());

    // TODO(prybicki): urgent optimizations and bold assumptions here :)
    if (lidars.size() != 1) {
        auto message = fmt::format("fixme: unexpected number of lidars: {}\n", lidars.size());
        throw std::logic_error(message);
    }
    // Using the aforementioned assumption, upload rays directly from the given buffer.
    rayBuffer.uploadAsync(lidars[0].directions.data(), lidars[0].directions.size());

    range.resize(lidars.size());
    for (size_t i = 0; i < lidars.size(); ++i) {
        range[i] = lidars[i].range;
    }
    rangeBuffer.uploadAsync(range.data(), range.size());

    source.resize(lidars.size() * 3);
    for (size_t i = 0; i < lidars.size(); ++i) {
        source[i * 3 + 0] = lidars[i].source.x;
        source[i * 3 + 1] = lidars[i].source.y;
        source[i * 3 + 2] = lidars[i].source.z;
    }
    sourceBuffer.uploadAsync(source.data(), source.size());
}

/*! download the rendered color buffer */
void LidarRenderer::downloadPoints(RaycastResults& result)
{
    PerfProbe c("download");
    result.clear();
    // if (model.meshes_map.size() == 0) return;

    CUDA_CHECK(StreamSynchronize(0));

    if (hits.size() != static_cast<size_t>(launchParams.rayCount)) {
        std::cerr << "wrong buffer size " << std::endl;
        return;
    }

    //  CUDA_CHECK(StreamSynchronize(0));
    // now rewrite to RaycastResults
    {
        PerfProbe cc("download-rewrite");
        int index = 0;
        for (int i = 0; i < launchParams.lidarCount; ++i) {
            RaycastResult res;
            for (int j = 0; j < raysPerLidar[i]; ++j) {
                if (hits[index]) {
                    LidarPoint point;
                    auto offset = index * 4;
                    point.x = allPoints[offset];
                    point.y = allPoints[offset + 1];
                    point.z = allPoints[offset + 2];
                    point.i = allPoints[offset + 3];
                    res.points.push_back(point);
                }
                index++;
            }
            result.push_back(res);
        }
    }
}

std::string LidarRenderer::getCurrentDeviceName()
{
    int currentDevice = -1;
    cudaDeviceProp deviceProperties {};
    CUDA_CHECK(GetDevice(&currentDevice));
    CUDA_CHECK(GetDeviceProperties(&deviceProperties, currentDevice));
    return std::string(deviceProperties.name);
}

CUcontext LidarRenderer::getCurrentDeviceContext()
{
    CUcontext cudaContext = {};
    CUresult status = cuCtxGetCurrent(&cudaContext);
    if (status != CUDA_SUCCESS) {
        const char* error = nullptr;
        cuGetErrorString(status, &error);
        throw std::runtime_error(format("failed to get current CUDA context: {} \n", error));
    }
    if (cudaContext == nullptr) {
        cudaFree(nullptr); // Force context initialization
        return getCurrentDeviceContext();
    }
    return cudaContext;
}
