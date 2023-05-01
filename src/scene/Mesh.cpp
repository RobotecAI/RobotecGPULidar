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

#include <scene/Mesh.hpp>

#include <filesystem>

namespace fs = std::filesystem;

API_OBJECT_INSTANCE(Mesh);

Mesh::Mesh(const Vec3f *vertices, size_t vertexCount, const Vec3i *indices, size_t indexCount)
{
	dVertices->copyFromHost(vertices, vertexCount);
	dIndices->copyFromHost(indices, indexCount);
}

void Mesh::updateVertices(const Vec3f *vertices, std::size_t vertexCount)
{
	if (dVertices->getCount() != vertexCount) {
		auto msg = fmt::format("Invalid argument: cannot update vertices because vertex counts do not match: old={}, new={}",
		                        dVertices->getCount(), vertexCount);
		throw std::invalid_argument(msg);
	}
	dVertices->copyFromHost(vertices, vertexCount);
	gasNeedsUpdate = true;
}

OptixTraversableHandle Mesh::getGAS(CudaStream::Ptr stream)
{
	if (!cachedGAS.has_value()) {
		cachedGAS = buildGAS(stream);
	}
	if (gasNeedsUpdate) {
		updateGAS(stream);
	}
	return *cachedGAS;
}

void Mesh::updateGAS(CudaStream::Ptr stream)
{
	OptixAccelBuildOptions updateOptions = buildOptions;
	updateOptions.operation = OPTIX_BUILD_OPERATION_UPDATE;

	// OptiX update disallows buffer sizes to change
	OptixBuildInput updateInput = buildInput;
	const CUdeviceptr vertexBuffers[1] = {dVertices->getDeviceReadPtr()};
	updateInput.triangleArray.vertexBuffers = vertexBuffers;
	updateInput.triangleArray.indexBuffer = dIndices->getDeviceReadPtr();

	scratchpad.resizeToFit(updateInput, updateOptions);

	// Fun fact: calling optixAccelBuild does not change anything visually, but introduces a significant slowdown
	// Investigation is needed whether it needs to be called at all (OptiX documentation says yes, but it works without)
	CHECK_OPTIX(optixAccelBuild(Optix::getOrCreate().context,
	                            stream->getHandle(),
	                            &updateOptions,
	                            &updateInput,
	                            1,
	                            scratchpad.dTemp->getDeviceReadPtr(),
	                            scratchpad.dTemp->getSizeOf() * scratchpad.dTemp->getCount(),
	                            scratchpad.dFull->getDeviceReadPtr(),
	                            scratchpad.dFull->getSizeOf() * scratchpad.dFull->getCount(),
	                            &cachedGAS.value(),
	                            nullptr, // &emitDesc,
	                            0));

	CHECK_CUDA(cudaStreamSynchronize(nullptr));

	gasNeedsUpdate = false;
}

OptixTraversableHandle Mesh::buildGAS(CudaStream::Ptr stream)
{
	triangleInputFlags = OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;
	vertexBuffers[0] = dVertices->getDeviceReadPtr();

	buildInput = {
		.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES,
		.triangleArray = {
			.vertexBuffers = vertexBuffers,
			.numVertices = static_cast<unsigned int>(dVertices->getCount()),
			.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3,
			.vertexStrideInBytes = sizeof(decltype(dVertices)::element_type::DataType),
			.indexBuffer = dIndices->getDeviceReadPtr(),
			.numIndexTriplets = static_cast<unsigned int>(dIndices->getCount()),
			.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3,
			.indexStrideInBytes = sizeof(decltype(dIndices)::element_type::DataType),
			.flags = &triangleInputFlags,
			.numSbtRecords = 1,
			.sbtIndexOffsetBuffer = 0,
			.sbtIndexOffsetSizeInBytes = 0,
			.sbtIndexOffsetStrideInBytes = 0,
		}
	};

	buildOptions = {
		.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE
		              | OPTIX_BUILD_FLAG_ALLOW_UPDATE,
		              // | OPTIX_BUILD_FLAG_ALLOW_COMPACTION, // Temporarily disabled
		.operation = OPTIX_BUILD_OPERATION_BUILD
	};

	scratchpad.resizeToFit(buildInput, buildOptions);

	// OptixAccelEmitDesc emitDesc = {
		// .result = scratchpad.dCompactedSize.readDeviceRaw(),
		// .type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE,
	// };

	OptixTraversableHandle gasHandle;
	CHECK_OPTIX(optixAccelBuild(Optix::getOrCreate().context,
	                            stream->getHandle(),
	                            &buildOptions,
	                            &buildInput,
	                            1,
	                            scratchpad.dTemp->getDeviceReadPtr(),
	                            scratchpad.dTemp->getSizeOf() * scratchpad.dTemp->getCount(),
	                            scratchpad.dFull->getDeviceReadPtr(),
	                            scratchpad.dFull->getSizeOf() * scratchpad.dFull->getCount(),
	                            &gasHandle,
	                            nullptr, // &emitDesc,
	                            0
	));

	// Compaction yields around 10% of memory save-up and slows down a lot (e.g. 500us per model)
	// scratchpad.doCompaction(gasHandle);

	gasNeedsUpdate = false;
	return gasHandle;
}
