#include "LidarRenderer.h"

#if defined(_WIN32)
#include <chrono>
#include <time.h>

struct timeval {
	long tv_sec;
	long tv_usec;
};


int gettimeofdayWin(struct timeval* tp, struct timezone* tzp) {
	namespace sc = std::chrono;
	sc::system_clock::duration d = sc::system_clock::now().time_since_epoch();
	sc::seconds s = sc::duration_cast<sc::seconds>(d);
	tp->tv_sec = s.count();
	tp->tv_usec = sc::duration_cast<sc::microseconds>(d - s).count();

	return 0;
}

long long current_timestampL()
{
    struct timeval te;
    gettimeofdayWin(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    return milliseconds;
}

#else

long long current_timestampL()
{
	struct timeval te;
	gettimeofday(&te, NULL); // get current time
	long long milliseconds = te.tv_sec * 1000LL + te.tv_usec / 1000; // calculate milliseconds
	return milliseconds;
}

#endif // _WIN32

extern "C" char embedded_ptx_code[];

  /*! SBT record for a raygen program */
struct __align__( OPTIX_SBT_RECORD_ALIGNMENT ) RaygenRecord
{
    __align__( OPTIX_SBT_RECORD_ALIGNMENT ) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    // don't need any data
    void *data;
};

  /*! SBT record for a miss program */
struct __align__( OPTIX_SBT_RECORD_ALIGNMENT ) MissRecord
{
    __align__( OPTIX_SBT_RECORD_ALIGNMENT ) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    // don't need any data
    void *data;
};

  /*! SBT record for a hitgroup program */
struct __align__( OPTIX_SBT_RECORD_ALIGNMENT ) HitgroupRecord
{
    __align__( OPTIX_SBT_RECORD_ALIGNMENT ) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    TriangleMeshSBTData data;
};

  /*! constructor - performs all setup, including initializing
    optix, creates module, pipeline, programs, SBT, etc. */
LidarRenderer::LidarRenderer(const Model *model)
    : model(model)
{
    initOptix();

    std::cout << "creating optix context ..." << std::endl;
    createContext();

    std::cout << "setting up module ..." << std::endl;
    createModule();

    std::cout << "creating raygen programs ..." << std::endl;
    createRaygenPrograms();
    std::cout << "creating miss programs ..." << std::endl;
    createMissPrograms();
    std::cout << "creating hitgroup programs ..." << std::endl;
    createHitgroupPrograms();

    launchParams.traversable = buildAccel();

    std::cout << "setting up optix pipeline ..." << std::endl;
    createPipeline();

    createTextures();

    std::cout << "building SBT ..." << std::endl;
    buildSBT();

    launchParamsBuffer.alloc(sizeof(launchParams));
    std::cout << "context, module, pipeline, etc, all set up ..." << std::endl;

    std::cout << GDT_TERMINAL_GREEN;
    std::cout << "Optix lidar fully set up" << std::endl;
    std::cout << GDT_TERMINAL_DEFAULT;
}

void LidarRenderer::createTextures()
{
    int numTextures = (int)model->textures.size();
    textureArrays.resize(numTextures);
    textureObjects.resize(numTextures);

    for (int textureID = 0; textureID < numTextures; textureID++)
    {
        auto texture = model->textures[textureID];

        cudaResourceDesc res_desc = {};

        cudaChannelFormatDesc channel_desc;
        int32_t width  = texture->resolution.x;
        int32_t height = texture->resolution.y;
        int32_t numComponents = 4;
        int32_t pitch  = width*numComponents*sizeof(uint8_t);
        channel_desc = cudaCreateChannelDesc<uchar4>();

        cudaArray_t   &pixelArray = textureArrays[textureID];
        CUDA_CHECK(MallocArray(&pixelArray,
                               &channel_desc,
                               width,height));

        CUDA_CHECK(Memcpy2DToArray(pixelArray,
                                   /* offset */0,0,
                                   texture->pixel,
                                   pitch,pitch,height,
                                   cudaMemcpyHostToDevice));

        res_desc.resType          = cudaResourceTypeArray;
        res_desc.res.array.array  = pixelArray;

        cudaTextureDesc tex_desc     = {};
        tex_desc.addressMode[0]      = cudaAddressModeWrap;
        tex_desc.addressMode[1]      = cudaAddressModeWrap;
        tex_desc.filterMode          = cudaFilterModeLinear;
        tex_desc.readMode            = cudaReadModeNormalizedFloat;
        tex_desc.normalizedCoords    = 1;
        tex_desc.maxAnisotropy       = 1;
        tex_desc.maxMipmapLevelClamp = 99;
        tex_desc.minMipmapLevelClamp = 0;
        tex_desc.mipmapFilterMode    = cudaFilterModePoint;
        tex_desc.borderColor[0]      = 1.0f;
        tex_desc.sRGB                = 0;

        // Create texture object
        cudaTextureObject_t cuda_tex = 0;
        CUDA_CHECK(CreateTextureObject(&cuda_tex, &res_desc, &tex_desc, nullptr));
        textureObjects[textureID] = cuda_tex;
    }
}

OptixTraversableHandle LidarRenderer::buildAccel(bool update)
{
    const int numMeshes = (int)model->meshes.size();
    
    for (int meshID = 0; meshID < vertexBuffer.size(); meshID++)
    {
        vertexBuffer[meshID].free();
        normalBuffer[meshID].free();
        texcoordBuffer[meshID].free();
        indexBuffer[meshID].free();
    }

    vertexBuffer.resize(numMeshes);
    normalBuffer.resize(numMeshes);
    texcoordBuffer.resize(numMeshes);
    indexBuffer.resize(numMeshes);

    OptixTraversableHandle asHandle { 0 };

    // ==================================================================
    // triangle inputs
    // ==================================================================
    std::vector<OptixBuildInput> triangleInput(numMeshes);
    std::vector<CUdeviceptr> d_vertices(numMeshes);
    std::vector<CUdeviceptr> d_indices(numMeshes);
    std::vector<uint32_t> triangleInputFlags(numMeshes);

    for (int meshID = 0; meshID < numMeshes; meshID++)
    {
        // upload the model to the device: the builder
        TriangleMesh &mesh = *model->meshes[meshID];
        vertexBuffer[meshID].alloc_and_upload(mesh.vertex);
        indexBuffer[meshID].alloc_and_upload(mesh.index);
        if (!mesh.normal.empty())
            normalBuffer[meshID].alloc_and_upload(mesh.normal);
        if (!mesh.texcoord.empty())
            texcoordBuffer[meshID].alloc_and_upload(mesh.texcoord);

        triangleInput[meshID] = {};
        triangleInput[meshID].type
            = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

        // create local variables, because we need a *pointer* to the
        // device pointers
        d_vertices[meshID] = vertexBuffer[meshID].d_pointer();
        d_indices[meshID]  = indexBuffer[meshID].d_pointer();

        triangleInput[meshID].triangleArray.vertexFormat        = OPTIX_VERTEX_FORMAT_FLOAT3;
        triangleInput[meshID].triangleArray.vertexStrideInBytes = sizeof(vec3f);
        triangleInput[meshID].triangleArray.numVertices         = (int)mesh.vertex.size();
        triangleInput[meshID].triangleArray.vertexBuffers       = &d_vertices[meshID];

        triangleInput[meshID].triangleArray.indexFormat         = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
        triangleInput[meshID].triangleArray.indexStrideInBytes  = sizeof(vec3i);
        triangleInput[meshID].triangleArray.numIndexTriplets    = (int)mesh.index.size();
        triangleInput[meshID].triangleArray.indexBuffer         = d_indices[meshID];

        triangleInputFlags[meshID] = OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;

        // in this example we have one SBT entry, and no per-primitive
        // materials:
        triangleInput[meshID].triangleArray.flags               = &triangleInputFlags[meshID];
        triangleInput[meshID].triangleArray.numSbtRecords               = 1;
        triangleInput[meshID].triangleArray.sbtIndexOffsetBuffer        = 0;
        triangleInput[meshID].triangleArray.sbtIndexOffsetSizeInBytes   = 0;
        triangleInput[meshID].triangleArray.sbtIndexOffsetStrideInBytes = 0;
    }
    // ==================================================================
    // BLAS setup
    // ==================================================================

    OptixAccelBuildOptions accelOptions = {};
    accelOptions.buildFlags             = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE 
        | OPTIX_BUILD_FLAG_ALLOW_UPDATE
        | OPTIX_BUILD_FLAG_ALLOW_COMPACTION
        ;
    accelOptions.motionOptions.numKeys  = 0; // no motion blur
    
    if (update)
        accelOptions.operation          = OPTIX_BUILD_OPERATION_UPDATE;
    else
        accelOptions.operation          = OPTIX_BUILD_OPERATION_BUILD;

    OptixAccelBufferSizes blasBufferSizes;
    OPTIX_CHECK(optixAccelComputeMemoryUsage
                (optixContext,
                 &accelOptions,
                 triangleInput.data(),
                 (int)numMeshes,  // num_build_inputs
                 &blasBufferSizes
                 ));

    // ==================================================================
    // prepare compaction
    // ==================================================================

    CUDABuffer compactedSizeBuffer;
    compactedSizeBuffer.alloc(sizeof(uint64_t));

    OptixAccelEmitDesc emitDesc;
    emitDesc.type   = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
    emitDesc.result = compactedSizeBuffer.d_pointer();

    // ==================================================================
    // execute build (main stage)
    // ==================================================================

    CUDABuffer tempBuffer;
    tempBuffer.alloc(blasBufferSizes.tempSizeInBytes);

    if (!update)
    {
        // for update we use existing buffer
        // in rebuild it should not change size
        // if size changes, build from begining
        outputBuffer.free();
        outputBuffer.alloc(blasBufferSizes.outputSizeInBytes);
    }

    OPTIX_CHECK(optixAccelBuild(optixContext,
                                /* stream */0,
                                &accelOptions,
                                triangleInput.data(),
                                (int)numMeshes,
                                tempBuffer.d_pointer(),
                                tempBuffer.sizeInBytes,

                                outputBuffer.d_pointer(),
                                outputBuffer.sizeInBytes,

                                &asHandle,

                                &emitDesc,1
                                ));
    CUDA_SYNC_CHECK();

    // ==================================================================
    // perform compaction
    // ==================================================================
    uint64_t compactedSize;
    compactedSizeBuffer.download(&compactedSize,1);

    asBuffer.free();
    asBuffer.alloc(compactedSize);
    OPTIX_CHECK(optixAccelCompact(optixContext,
                                  /*stream:*/0,
                                  asHandle,
                                  asBuffer.d_pointer(),
                                  asBuffer.sizeInBytes,
                                  &asHandle));
    CUDA_SYNC_CHECK();

    // ==================================================================
    // aaaaaand .... clean up
    // ==================================================================
    tempBuffer.free();
    compactedSizeBuffer.free();

    return asHandle;
}

/*! helper function that initializes optix and checks for errors */
void LidarRenderer::initOptix()
{
    std::cout << "initializing optix..." << std::endl;

    // -------------------------------------------------------
    // check for available optix7 capable devices
    // -------------------------------------------------------
    cudaFree(0);
    int numDevices;
    cudaGetDeviceCount(&numDevices);
    if (numDevices == 0)
      throw std::runtime_error("no CUDA capable devices found!");
    std::cout << "found " << numDevices << " CUDA devices" << std::endl;

    // -------------------------------------------------------
    // initialize optix
    // -------------------------------------------------------
    OPTIX_CHECK( optixInit() );
    std::cout << GDT_TERMINAL_GREEN
              << "successfully initialized optix... yay!"
              << GDT_TERMINAL_DEFAULT << std::endl;
}

static void context_log_cb(unsigned int level,
                           const char *tag,
                           const char *message,
                           void *)
{
    fprintf( stderr, "[%2d][%12s]: %s\n", (int)level, tag, message );
}

  /*! creates and configures a optix device context (in this simple
      example, only for the primary GPU device) */
void LidarRenderer::createContext()
{
    // for this sample, do everything on one device
    const int deviceID = 0;
    CUDA_CHECK(SetDevice(deviceID));
    CUDA_CHECK(StreamCreate(&stream));

    cudaGetDeviceProperties(&deviceProps, deviceID);
    std::cout << "running on device: " << deviceProps.name << std::endl;

    CUresult  cuRes = cuCtxGetCurrent(&cudaContext);
    if( cuRes != CUDA_SUCCESS )
        fprintf( stderr, "Error querying current context: error code %d\n", cuRes );

    OPTIX_CHECK(optixDeviceContextCreate(cudaContext, 0, &optixContext));
    OPTIX_CHECK(optixDeviceContextSetLogCallback
                (optixContext,context_log_cb,nullptr,4));
}



  /*! creates the module that contains all the programs we are going
      to use. in this simple example, we use a single module from a
      single .cu file, using a single embedded ptx string */
void LidarRenderer::createModule()
{
    moduleCompileOptions.maxRegisterCount  = 100;
    moduleCompileOptions.optLevel          = OPTIX_COMPILE_OPTIMIZATION_LEVEL_0;
    moduleCompileOptions.debugLevel        = OPTIX_COMPILE_DEBUG_LEVEL_LINEINFO;

    pipelineCompileOptions = {};
    pipelineCompileOptions.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY;
    pipelineCompileOptions.usesMotionBlur     = false;
    pipelineCompileOptions.numPayloadValues   = 4;
    pipelineCompileOptions.numAttributeValues = 2;
    pipelineCompileOptions.exceptionFlags     = OPTIX_EXCEPTION_FLAG_NONE;
    pipelineCompileOptions.pipelineLaunchParamsVariableName = "optixLaunchLidarParams";

    pipelineLinkOptions.overrideUsesMotionBlur = false;
    pipelineLinkOptions.maxTraceDepth          = 2;

    const std::string ptxCode = embedded_ptx_code;

    char log[2048];
    size_t sizeof_log = sizeof( log );
    OPTIX_CHECK(optixModuleCreateFromPTX(optixContext,
                                         &moduleCompileOptions,
                                         &pipelineCompileOptions,
                                         ptxCode.c_str(),
                                         ptxCode.size(),
                                         log,&sizeof_log,
                                         &module
                                         ));
    if (sizeof_log > 1) PRINT(log);
}



  /*! does all setup for the raygen program(s) we are going to use */
void LidarRenderer::createRaygenPrograms()
{
    // we do a single ray gen program in this example:
    raygenPGs.resize(1);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc    = {};
    pgDesc.kind                     = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    pgDesc.raygen.module            = module;
    pgDesc.raygen.entryFunctionName = "__raygen__renderLidar";

    // OptixProgramGroup raypg;
    char log[2048];
    size_t sizeof_log = sizeof( log );
    OPTIX_CHECK(optixProgramGroupCreate(optixContext,
                                        &pgDesc,
                                        1,
                                        &pgOptions,
                                        log,&sizeof_log,
                                        &raygenPGs[LIDAR_RAY_TYPE]
                                        ));
    if (sizeof_log > 1) PRINT(log);
}

  /*! does all setup for the miss program(s) we are going to use */
void LidarRenderer::createMissPrograms()
{
    // we do a single ray gen program in this example:
    missPGs.resize(LIDAR_RAY_TYPE_COUNT);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc    = {};
    pgDesc.kind                     = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pgDesc.miss.module            = module;
    pgDesc.miss.entryFunctionName = "__miss__lidar";

    // OptixProgramGroup raypg;
    char log[2048];
    size_t sizeof_log = sizeof( log );
    OPTIX_CHECK(optixProgramGroupCreate(optixContext,
                                        &pgDesc,
                                        1,
                                        &pgOptions,
                                        log,&sizeof_log,
                                        &missPGs[LIDAR_RAY_TYPE]
                                        ));
    if (sizeof_log > 1) PRINT(log);
}

  /*! does all setup for the hitgroup program(s) we are going to use */
void LidarRenderer::createHitgroupPrograms()
{
    // for this simple example, we set up a single hit group
    hitgroupPGs.resize(LIDAR_RAY_TYPE_COUNT);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc    = {};
    pgDesc.kind                     = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pgDesc.hitgroup.moduleCH            = module;
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__lidar";
    pgDesc.hitgroup.moduleAH            = module;
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__lidar";

    char log[2048];
    size_t sizeof_log = sizeof( log );
    OPTIX_CHECK(optixProgramGroupCreate(optixContext,
                                        &pgDesc,
                                        1,
                                        &pgOptions,
                                        log,&sizeof_log,
                                        &hitgroupPGs[LIDAR_RAY_TYPE]
                                        ));
    if (sizeof_log > 1) PRINT(log);
}


  /*! assembles the full pipeline of all programs */
void LidarRenderer::createPipeline()
{
    std::vector<OptixProgramGroup> programGroups;
    for (auto pg : raygenPGs)
      programGroups.push_back(pg);
    for (auto pg : missPGs)
      programGroups.push_back(pg);
    for (auto pg : hitgroupPGs)
      programGroups.push_back(pg);

    char log[2048];
    size_t sizeof_log = sizeof( log );
    OPTIX_CHECK(optixPipelineCreate(optixContext,
                                    &pipelineCompileOptions,
                                    &pipelineLinkOptions,
                                    programGroups.data(),
                                    (int)programGroups.size(),
                                    log,&sizeof_log,
                                    &pipeline
                                    ));
    if (sizeof_log > 1) PRINT(log);

    OPTIX_CHECK(optixPipelineSetStackSize
                (/* [in] The pipeline to configure the stack size for */
                 pipeline,
                 /* [in] The direct stack size requirement for direct
                    callables invoked from IS or AH. */
                 2*1024,
                 /* [in] The direct stack size requirement for direct
                    callables invoked from RG, MS, or CH.  */
                 2*1024,
                 /* [in] The continuation stack requirement. */
                 2*1024,
                 /* [in] The maximum depth of a traversable graph
                    passed to trace. */
                 3));
    if (sizeof_log > 1) PRINT(log);
}


/*! constructs the shader binding table */
void LidarRenderer::buildSBT()
{
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<RaygenRecord> raygenRecords;
    for (int i = 0; i < raygenPGs.size(); i++) {
        RaygenRecord rec;
        OPTIX_CHECK(optixSbtRecordPackHeader(raygenPGs[i],&rec));
        rec.data = nullptr; /* for now ... */
        raygenRecords.push_back(rec);
    }
    raygenRecordsBuffer.free();
    raygenRecordsBuffer.alloc_and_upload(raygenRecords);
    sbt.raygenRecord = raygenRecordsBuffer.d_pointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<MissRecord> missRecords;
    for (int i = 0; i < missPGs.size(); i++) {
        MissRecord rec;
        OPTIX_CHECK(optixSbtRecordPackHeader(missPGs[i],&rec));
        rec.data = nullptr; /* for now ... */
        missRecords.push_back(rec);
    }
    missRecordsBuffer.free();
    missRecordsBuffer.alloc_and_upload(missRecords);
    sbt.missRecordBase          = missRecordsBuffer.d_pointer();
    sbt.missRecordStrideInBytes = sizeof(MissRecord);
    sbt.missRecordCount         = (int)missRecords.size();

    // ------------------------------------------------------------------
    // build hitgroup records
    // ------------------------------------------------------------------
    int numObjects = (int)model->meshes.size();
    std::vector<HitgroupRecord> hitgroupRecords;
    for (int meshID = 0; meshID < numObjects; meshID++)
    {
        for (int rayID = 0; rayID < LIDAR_RAY_TYPE_COUNT; rayID++)
        {
            auto mesh = model->meshes[meshID];

            HitgroupRecord rec;
            OPTIX_CHECK(optixSbtRecordPackHeader(hitgroupPGs[rayID],&rec));
            rec.data.color   = mesh->diffuse;
            if (mesh->diffuseTextureID >= 0)
            {
                rec.data.hasTexture = true;
                rec.data.texture    = textureObjects[mesh->diffuseTextureID];
            } else
            {
                rec.data.hasTexture = false;
            }
            rec.data.index    = (vec3i*)indexBuffer[meshID].d_pointer();
            rec.data.vertex   = (vec3f*)vertexBuffer[meshID].d_pointer();
            rec.data.normal   = (vec3f*)normalBuffer[meshID].d_pointer();
            rec.data.texcoord = (vec2f*)texcoordBuffer[meshID].d_pointer();
            hitgroupRecords.push_back(rec);
        }
    }
    hitgroupRecordsBuffer.free();
    hitgroupRecordsBuffer.alloc_and_upload(hitgroupRecords);
    sbt.hitgroupRecordBase          = hitgroupRecordsBuffer.d_pointer();
    sbt.hitgroupRecordStrideInBytes = sizeof(HitgroupRecord);
    sbt.hitgroupRecordCount         = (int)hitgroupRecords.size();
}



  /*! render one frame */
void LidarRenderer::render(std::vector<LidarSource> &lidars)
{
    // sanity check: make sure we launch only after first resize is
    // already done:
    if (launchParams.rayCount == 0) return;
    
    if (model->moved)
    {
        if (model->big == true)
        {
//printf("Przebudowywanie od poczatku\n");
            launchParams.traversable = buildAccel(/*update =*/ false);
        }
        else
        {
            launchParams.traversable = buildAccel(/*update =*/ true);
        }
        buildSBT();
    }
    
    // upload rays data to device
    uploadRays(lidars);

    launchParamsBuffer.upload(&launchParams,1);

    OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
                            pipeline,stream,
                            /*! parameters and SBT */
                            launchParamsBuffer.d_pointer(),
                            launchParamsBuffer.sizeInBytes,
                            &sbt,
                            /*! dimensions of the launch: */
                            launchParams.rayCount,
                            1,
                            1
                            ));
    // sync - make sure the frame is rendered before we download and
    // display (obviously, for a high-performance application you
    // want to use streams and double-buffering, but for this simple
    // example, this will have to do)
    CUDA_SYNC_CHECK();
}

  /*! resize frame buffer to given resolution */
void LidarRenderer::resize(std::vector<LidarSource> &lidars)
{
    int lidarCount = lidars.size();
    int rayCount = 0;
    for (int i = 0; i < lidarCount; ++i)
    {
        rayCount += lidars[i].directions.size();
    }
    
    // resize our cuda frame buffer
    raysPerLidarBuffer.resize(lidarCount*sizeof(int));
    rayBuffer.resize(rayCount*3*sizeof(float));
    rangeBuffer.resize(lidarCount*sizeof(float));
    sourceBuffer.resize(lidarCount*3*sizeof(float));
    
    positionBuffer.resize(rayCount*4*sizeof(float));
    hitBuffer.resize(rayCount*sizeof(int));

    // update the launch parameters that we'll pass to the optix
    // launch:
    launchParams.rayCount       = rayCount;
    launchParams.lidarCount     = lidarCount;
    launchParams.raysPerLidarBuffer = (int*)raysPerLidarBuffer.d_ptr;
    launchParams.rayBuffer      = (float*)rayBuffer.d_ptr;
    launchParams.rangeBuffer    = (float*)rangeBuffer.d_ptr;
    launchParams.sourceBuffer   = (float*)sourceBuffer.d_ptr;
    launchParams.positionBuffer = (float*)positionBuffer.d_ptr;
    launchParams.hitBuffer      = (int*)hitBuffer.d_ptr;
}

void LidarRenderer::uploadRays(std::vector<LidarSource> &lidars)
{
    std::vector<int> raysPerLidar;
    raysPerLidar.resize(lidars.size());
    for (int i = 0; i < lidars.size(); ++i)
    {
        raysPerLidar[i] = lidars[i].directions.size();
    }
    raysPerLidarBuffer.upload(raysPerLidar.data(), raysPerLidar.size());
    
    std::vector<float> rays;
    rays.resize(launchParams.rayCount*3);
    int index = 0;
    for (int i = 0; i < lidars.size(); ++i)
    {
        for (int j = 0; j < lidars[i].directions.size(); j++)
        {
            rays[index++] = lidars[i].directions[j].x;
            rays[index++] = lidars[i].directions[j].y;
            rays[index++] = lidars[i].directions[j].z;
        }
    }
    rayBuffer.upload(rays.data(), rays.size());
    
    std::vector<float> range;
    range.resize(lidars.size());
    for(int i = 0; i < lidars.size(); ++i)
    {
        range[i] = lidars[i].range;
    }
    rangeBuffer.upload(range.data(), range.size());
    
    std::vector<float> source;
    source.resize(lidars.size()*3);
    for (int i = 0; i < lidars.size(); ++i)
    {
        source[i*3+0] = lidars[i].source.x;
        source[i*3+1] = lidars[i].source.y;
        source[i*3+2] = lidars[i].source.z;
    }
    sourceBuffer.upload(source.data(), source.size());
}


  /*! download the rendered color buffer */
void LidarRenderer::downloadPoints(RaycastResults &result)
{
    std::vector<float> allPoints;
    allPoints.resize(launchParams.rayCount*4);
    std::vector<int> hits;
    hits.resize(launchParams.rayCount);
    std::vector<int> raysPerLidar;
    raysPerLidar.resize(launchParams.lidarCount);
    
    positionBuffer.download(allPoints.data(), launchParams.rayCount*4);
    hitBuffer.download(hits.data(), launchParams.rayCount);
    raysPerLidarBuffer.download(raysPerLidar.data(), launchParams.lidarCount);
    
    // now rewrite to RaycastResults
    int index = 0;
    for(int i = 0; i < launchParams.lidarCount; ++i)
    {
        RaycastResult res;
        for(int j = 0; j < raysPerLidar[i]; ++j)
        {
            if(hits[index])
            {
                LidarPoint point;
                point.x = allPoints[index*4];
                point.y = allPoints[index*4+1];
                point.z = allPoints[index*4+2];
                point.i = allPoints[index*4+3];
                res.points.push_back(point);
            }
            index++;
        }
        
        result.push_back(res);
    }
}

