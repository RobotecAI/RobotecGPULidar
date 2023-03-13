// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cassert>
#include <stdexcept>
#include <cstring>

#include <cuda.h>
#include <nvml.h>
#include <optix.h>
#include <optix_stubs.h>
#include <optix_function_table_definition.h>
#include <cuda_runtime_api.h>
#include <spdlog/fmt/fmt.h>

#include <Optix.hpp>
#include <Logger.hpp>
#include <gpu/optixProgramsPtx.hpp>
#include <macros/optix.hpp>
#include <macros/cuda.hpp>
#include <git.h>

#define OPTIX_LOG_LEVEL_FATAL 1
#define OPTIX_LOG_LEVEL_ERROR 2
#define OPTIX_LOG_LEVEL_WARN 3
#define OPTIX_LOG_LEVEL_INFO 4

static std::pair<int, int> getCudaMajorMinor(int version)
{
	return {version / 1000, (version % 1000) / 10};
}

static std::optional<std::string> wrapError(nvmlReturn_t status)
{
	if (status == NVML_SUCCESS) {
		return std::nullopt;
	}
	const char* err = nvmlErrorString(status);
	return {std::string(err)};
}

static CUcontext getCurrentDeviceContext()
{
	const char* error = nullptr;
	CUresult status;

	cudaFree(nullptr); // Force CUDA runtime initialization

	CUdevice device;
	status = cuDeviceGet(&device, 0);
	if (status != CUDA_SUCCESS) {
		cuGetErrorString(status, &error);
		throw std::runtime_error(fmt::format("failed to get current CUDA device: {} ({})\n", error, status));
	}

	CUcontext cudaContext = nullptr;
	CUresult primaryCtxStatus = cuDevicePrimaryCtxRetain(&cudaContext, device);
	if (primaryCtxStatus != CUDA_SUCCESS) {
		cuGetErrorString(status, &error);
		throw std::runtime_error(fmt::format("failed to get primary CUDA context: {} ({})\n", error, status));
	}
	assert(cudaContext != nullptr);
	return cudaContext;
}

static std::string getCurrentDeviceName()
{
	int currentDevice = -1;
	cudaDeviceProp deviceProperties {};
	CHECK_CUDA(cudaGetDevice(&currentDevice));
	CHECK_CUDA(cudaGetDeviceProperties(&deviceProperties, currentDevice));
	return std::string(deviceProperties.name);
}


void Optix::logVersions()
{
	// Location of these logs is somewhat non-trivial, so it would be good conceive something better.
	RGL_INFO("RGL Version {}.{}.{} branch={} commitSHA1={}",
	         RGL_VERSION_MAJOR, RGL_VERSION_MINOR, RGL_VERSION_PATCH, git::Branch(), git::CommitSHA1());

	RGL_INFO("Running on GPU: {}", getCurrentDeviceName());

	auto optixMajor =  OPTIX_VERSION / 10000;
	auto optixMinor = (OPTIX_VERSION % 10000) / 100;
	auto optixMicro =  OPTIX_VERSION % 100;
	RGL_INFO("Built against OptiX SDK version: {}.{}.{}", optixMajor, optixMinor, optixMicro);
	RGL_INFO("Built against OptiX ABI version: {}", OPTIX_ABI_VERSION);

	auto [toolkitMajor, toolkitMinor] = getCudaMajorMinor(CUDA_VERSION);
	RGL_INFO("Built against CUDA Toolkit version: {}.{}", toolkitMajor, toolkitMinor);

	int cudaRuntimeVersion = -1;
	CHECK_CUDA(cudaRuntimeGetVersion(&cudaRuntimeVersion));
	auto [cudaRuntimeMajor, cudaRuntimeMinor] = getCudaMajorMinor(cudaRuntimeVersion);
	RGL_INFO("Installed CUDA runtime version: {}.{}", cudaRuntimeMajor, cudaRuntimeMinor);

	int cudaDriverVersion = -1;
	CHECK_CUDA(cudaDriverGetVersion(&cudaDriverVersion));
	auto [cudaDriverMajor, cudaDriverMinor] = getCudaMajorMinor(cudaDriverVersion);
	RGL_INFO("Installed CUDA driver version: {}.{}", cudaDriverMajor, cudaDriverMinor);

	if (auto err = wrapError(nvmlInit())) {
		RGL_WARN("Failed to fillSizeAndOffset NVidia Management Library (NVML): {}", err.value());
		return;
	}

	char driverVersionStr[128] = {0};
	if (auto err = wrapError(nvmlSystemGetDriverVersion(driverVersionStr, sizeof(driverVersionStr)))) {
		std::string msg = fmt::format("(nvml error: {})", err.value());
		strncpy(driverVersionStr, msg.c_str(), sizeof(driverVersionStr));
	}
	RGL_INFO("Installed NVidia kernel driver version: {}", driverVersionStr);

}

Optix& Optix::getOrCreate()
{
	static Optix instance;
	return instance;
}

Optix::Optix()
{
	logVersions();
	CHECK_OPTIX(optixInit());
	CHECK_OPTIX(optixDeviceContextCreate(getCurrentDeviceContext(), nullptr, &context));

	auto cb = [](unsigned level, const char* tag, const char* message, void*) {
		auto fmt = "[OptiX][{:^12}]: {}";
		if (level == OPTIX_LOG_LEVEL_FATAL) {
			RGL_CRITICAL(fmt::runtime(fmt), tag, message);
		}
		if (level == OPTIX_LOG_LEVEL_ERROR) {
			RGL_ERROR(fmt::runtime(fmt), tag, message);
		}
		if (level == OPTIX_LOG_LEVEL_WARN) {
			RGL_WARN(fmt::runtime(fmt), tag, message);
		}
		if (level == OPTIX_LOG_LEVEL_INFO) {
			// Lower log level to prevent leaking backend logs to end user
			RGL_DEBUG(fmt::runtime(fmt), tag, message);
		}
	};

	CHECK_OPTIX(optixDeviceContextSetLogCallback(context, cb, nullptr, OPTIX_LOG_LEVEL_INFO));
	initializeStaticOptixStructures();
}

Optix::~Optix()
{
	if (pipeline) {
		optixPipelineDestroy(pipeline);
	}

	for (auto&& programGroup : { raygenPG, missPG, hitgroupPG }) {
		if (programGroup) {
			optixProgramGroupDestroy(programGroup);
		}
	}

	if (module) {
		optixModuleDestroy(module);
	}

	if (context) {
		optixDeviceContextDestroy(context);
	}
}

void Optix::initializeStaticOptixStructures()
{
	OptixModuleCompileOptions moduleCompileOptions = {
		.maxRegisterCount = 100,
#ifdef NDEBUG
		.optLevel = OPTIX_COMPILE_OPTIMIZATION_LEVEL_2,
		.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE
#else
		.optLevel = OPTIX_COMPILE_OPTIMIZATION_LEVEL_0,
		.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL
#endif
	};

	OptixPipelineCompileOptions pipelineCompileOptions = {
		.usesMotionBlur = false,
		.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY,
		.numPayloadValues = 3,  // Ray origin: X, Y, Z
		.numAttributeValues = 2,  // Triangle barycentrics: X, Y
		.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE,
		.pipelineLaunchParamsVariableName = "ctx",
	};

	OptixPipelineLinkOptions pipelineLinkOptions = {
		.maxTraceDepth = 2,
#ifdef NDEBUG
		.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE,
#else
		.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL,
#endif
	};

	CHECK_OPTIX(optixModuleCreateFromPTX(context,
		&moduleCompileOptions,
		&pipelineCompileOptions,
		optixProgramsPtx,
		strlen(optixProgramsPtx),
		nullptr, nullptr,
		&module
	));

	OptixProgramGroupOptions pgOptions = {};
	OptixProgramGroupDesc raygenDesc = {
		.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN,
		.raygen = {
			.module = module,
			.entryFunctionName = "__raygen__" }
	};

	CHECK_OPTIX(optixProgramGroupCreate(
		context, &raygenDesc, 1, &pgOptions, nullptr, nullptr, &raygenPG));

	OptixProgramGroupDesc missDesc = {
		.kind = OPTIX_PROGRAM_GROUP_KIND_MISS,
		.miss = {
			.module = module,
			.entryFunctionName = "__miss__" },
	};

	CHECK_OPTIX(optixProgramGroupCreate(
		context, &missDesc, 1, &pgOptions, nullptr, nullptr, &missPG));

	OptixProgramGroupDesc hitgroupDesc = {
		.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP,
		.hitgroup = {
			.moduleCH = module,
			.entryFunctionNameCH = "__closesthit__",
			.moduleAH = module,
			.entryFunctionNameAH = "__anyhit__",
		}
	};

	CHECK_OPTIX(optixProgramGroupCreate(
		context, &hitgroupDesc, 1, &pgOptions, nullptr, nullptr, &hitgroupPG));

	OptixProgramGroup programGroups[] = { raygenPG, missPG, hitgroupPG };

	CHECK_OPTIX(optixPipelineCreate(
		context,
		&pipelineCompileOptions,
		&pipelineLinkOptions,
		programGroups,
		sizeof(programGroups) / sizeof(programGroups[0]),
		nullptr, nullptr,
		&pipeline
	));

	CHECK_OPTIX(optixPipelineSetStackSize(
		pipeline,
		2 * 1024, // directCallableStackSizeFromTraversal
		2 * 1024, // directCallableStackSizeFromState
		2 * 1024, // continuationStackSize
		3 // maxTraversableGraphDepth
	));
}
