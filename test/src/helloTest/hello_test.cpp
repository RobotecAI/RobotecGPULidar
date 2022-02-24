#include <gtest/gtest.h>

#include "gdt/utils/CUDABuffer.h"
#include "helloTest.h"
#include <optix.h>

extern "C" char helloTestCU[];

#define OPTIX_LOG_LEVEL_NONE 0
#define OPTIX_LOG_LEVEL_FATAL 1
#define OPTIX_LOG_LEVEL_ERROR 2
#define OPTIX_LOG_LEVEL_WARN 3
#define OPTIX_LOG_LEVEL_INFO 4

TEST(HelloTest, BasicAssertions) {
    OptixDeviceContext optixContext = nullptr;
    {
        // Initialize CUDA
        CUDA_CHECK( Free( 0 ) );
        CUcontext cuCtx = 0;  // zero means take the current context
        OPTIX_CHECK( optixInit() );
        OPTIX_CHECK( optixDeviceContextCreate( cuCtx, nullptr, &optixContext ) );
    }

    OPTIX_CHECK(optixDeviceContextSetLogCallback(
        optixContext,
        [](unsigned level, const char* tag, const char* message, void*) {
            auto fmt = "[RGL][OptiX][{:2}][{:^12}]: {}\n";
            if (level == OPTIX_LOG_LEVEL_FATAL || level == OPTIX_LOG_LEVEL_ERROR) {
                return;
            }
            if (level == OPTIX_LOG_LEVEL_WARN) {
                return;
            }
        },
        nullptr, 4));

    OptixModule module = nullptr;
    OptixPipelineCompileOptions pipeline_compile_options = {};
    {
        OptixModuleCompileOptions module_compile_options = {};
        module_compile_options.maxRegisterCount     = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
        module_compile_options.optLevel             = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
        module_compile_options.debugLevel           = OPTIX_COMPILE_DEBUG_LEVEL_LINEINFO;

        pipeline_compile_options.usesMotionBlur        = false;
        pipeline_compile_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY;
        pipeline_compile_options.numPayloadValues      = 2;
        pipeline_compile_options.numAttributeValues    = 2;
        pipeline_compile_options.exceptionFlags        = OPTIX_EXCEPTION_FLAG_NONE;  // TODO: should be OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
        pipeline_compile_options.pipelineLaunchParamsVariableName = "params";

        OPTIX_CHECK( optixModuleCreateFromPTX(
            optixContext,
            &module_compile_options,
            &pipeline_compile_options,
            helloTestCU,
            strlen(helloTestCU),
            nullptr, nullptr,
            &module
        ) );
    }

    OptixProgramGroup raygen_prog_group   = nullptr;
    OptixProgramGroup miss_prog_group     = nullptr;
    {
        OptixProgramGroupOptions program_group_options   = {}; // Initialize to zeros

        OptixProgramGroupDesc raygen_prog_group_desc  = {}; //
        raygen_prog_group_desc.kind                     = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
        raygen_prog_group_desc.raygen.module            = module;
        raygen_prog_group_desc.raygen.entryFunctionName = "__raygen__draw_solid_color";
        OPTIX_CHECK( optixProgramGroupCreate(
            optixContext,
            &raygen_prog_group_desc,
            1,   // num program groups
            &program_group_options,
            nullptr,
            nullptr,
            &raygen_prog_group
        ) );

        // Leave miss group's module and entryfunc name null
        OptixProgramGroupDesc miss_prog_group_desc = {};
        miss_prog_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
        OPTIX_CHECK( optixProgramGroupCreate(
            optixContext,
            &miss_prog_group_desc,
            1,   // num program groups
            &program_group_options,
            nullptr, nullptr,
            &miss_prog_group
        ) );
    }

    //
    // Link pipeline
    //
    OptixPipeline pipeline = nullptr;
    {
        const uint32_t    max_trace_depth  = 0;
        OptixProgramGroup program_groups[] = { raygen_prog_group };

        OptixPipelineLinkOptions pipeline_link_options = {};
        pipeline_link_options.maxTraceDepth          = max_trace_depth;
        pipeline_link_options.debugLevel             = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
        OPTIX_CHECK( optixPipelineCreate(
            optixContext,
            &pipeline_compile_options,
            &pipeline_link_options,
            program_groups,
            sizeof( program_groups ) / sizeof( program_groups[0] ),
            nullptr, nullptr,
            &pipeline
        ) );

        OptixStackSizes stack_sizes = {};
        for( auto& prog_group : program_groups )
        {
            OPTIX_CHECK( optixUtilAccumulateStackSizes( prog_group, &stack_sizes ) );
        }

        uint32_t direct_callable_stack_size_from_traversal;
        uint32_t direct_callable_stack_size_from_state;
        uint32_t continuation_stack_size;
        OPTIX_CHECK( optixUtilComputeStackSizes( &stack_sizes, max_trace_depth,
                                                 0,  // maxCCDepth
                                                 0,  // maxDCDEpth
                                                 &direct_callable_stack_size_from_traversal,
                                                 &direct_callable_stack_size_from_state, &continuation_stack_size ) );
        OPTIX_CHECK( optixPipelineSetStackSize( pipeline, direct_callable_stack_size_from_traversal,
                                                direct_callable_stack_size_from_state, continuation_stack_size,
                                                2  // maxTraversableDepth
        ) );
    }

    //
    // Set up shader binding table
    //
    OptixShaderBindingTable sbt = {};
    {
        CUdeviceptr  raygen_record;
        const size_t raygen_record_size = sizeof( RayGenSbtRecord );
        CUDA_CHECK( Malloc( reinterpret_cast<void**>( &raygen_record ), raygen_record_size ) );
        RayGenSbtRecord rg_sbt;
        OPTIX_CHECK( optixSbtRecordPackHeader( raygen_prog_group, &rg_sbt ) );
        rg_sbt.data = {0.462f, 0.725f, 0.f};
        CUDA_CHECK( Memcpy(
            reinterpret_cast<void*>( raygen_record ),
            &rg_sbt,
            raygen_record_size,
            cudaMemcpyHostToDevice
        ) );

        CUdeviceptr miss_record;
        size_t      miss_record_size = sizeof( MissSbtRecord );
        CUDA_CHECK( Malloc( reinterpret_cast<void**>( &miss_record ), miss_record_size ) );
        RayGenSbtRecord ms_sbt;
        OPTIX_CHECK( optixSbtRecordPackHeader( miss_prog_group, &ms_sbt ) );
        CUDA_CHECK( Memcpy(
            reinterpret_cast<void*>( miss_record ),
            &ms_sbt,
            miss_record_size,
            cudaMemcpyHostToDevice
        ) );

        sbt.raygenRecord                = raygen_record;
        sbt.missRecordBase              = miss_record;
        sbt.missRecordStrideInBytes     = sizeof( MissSbtRecord );
        sbt.missRecordCount             = 1;
    }

    CUDABuffer output_buffer;

    //
    // launch
    //
    {
        CUstream stream;
        CUDA_CHECK( StreamCreate( &stream ) );

        Params params;
        params.image       = (u_char*)output_buffer.d_ptr();
        params.image_width = width;

        CUdeviceptr d_param;
        CUDA_CHECK( Malloc( reinterpret_cast<void**>( &d_param ), sizeof( Params ) ) );
        CUDA_CHECK( Memcpy(
            reinterpret_cast<void*>( d_param ),
            &params, sizeof( params ),
            cudaMemcpyHostToDevice
        ) );

        OPTIX_CHECK( optixLaunch( pipeline, stream, d_param, sizeof( Params ), &sbt, width, height, /*depth=*/1 ) );
        CUDA_SYNC_CHECK();

    }

//    std::vector<
//    output_buffer.downloadAsync(raysPerLidar.data(), launchParams.lidarCount);
//
// Cleanup
    //
//    {
//    CUDA_CHECK( Free( reinterpret_cast<void*>( sbt.raygenRecord       ) ) );
//    CUDA_CHECK( Free( reinterpret_cast<void*>( sbt.missRecordBase     ) ) );
//
//    OPTIX_CHECK( optixPipelineDestroy( pipeline ) );
//    OPTIX_CHECK( optixProgramGroupDestroy( miss_prog_group ) );
//    OPTIX_CHECK( optixProgramGroupDestroy( raygen_prog_group ) );
//    OPTIX_CHECK( optixModuleDestroy( module ) );
//
//    OPTIX_CHECK( optixDeviceContextDestroy( optixContext ) );
//    }

EXPECT_STRNE("hello", "world");
EXPECT_EQ(7 * 6, 42);
}
