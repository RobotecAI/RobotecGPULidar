// Copyright 2024 Robotec.AI
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

#pragma once

#include <type_traits>

#include <math/Vector.hpp>
#include <memory/Array.hpp>
#include <scene/ASBuildScratchpad.hpp>
#include <macros/optix.hpp>

/*
 * Builder for geometry-AS.
 * It also allows to update GAS (vertex and index counts must be equal to the original counts).
 * All operations are performed asynchronously.
 */
struct GASBuilder
{
	explicit GASBuilder(const CudaStream::Ptr& stream, const DeviceSyncArray<Vec3f>::Ptr& vertices,
	                    const DeviceSyncArray<Vec3i>::Ptr& indices)
	{
		triangleInputFlags = OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;
		vertexBuffers[0] = vertices->getDeviceReadPtr();

		buildInput = {
		    .type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES,
		    .triangleArray = {
		                      .vertexBuffers = vertexBuffers,
		                      .numVertices = static_cast<unsigned int>(vertices->getCount()),
		                      .vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3,
		                      .vertexStrideInBytes = sizeof(std::remove_cvref_t<decltype(vertices)>::element_type::DataType),
		                      .indexBuffer = indices->getDeviceReadPtr(),
		                      .numIndexTriplets = static_cast<unsigned int>(indices->getCount()),
		                      .indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3,
		                      .indexStrideInBytes = sizeof(std::remove_cvref_t<decltype(indices)>::element_type::DataType),
		                      .flags = &triangleInputFlags,
		                      .numSbtRecords = 1,
		                      .sbtIndexOffsetBuffer = 0,
		                      .sbtIndexOffsetSizeInBytes = 0,
		                      .sbtIndexOffsetStrideInBytes = 0,
		                      }
        };

		buildOptions = {.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE | OPTIX_BUILD_FLAG_ALLOW_UPDATE,
		                // | OPTIX_BUILD_FLAG_ALLOW_COMPACTION, // Temporarily disabled
		                .operation = OPTIX_BUILD_OPERATION_BUILD};

		scratchpad.resizeToFit(buildInput, buildOptions);

		// OptixAccelEmitDesc emitDesc = {
		// .result = scratchpad.dCompactedSize.readDeviceRaw(),
		// .type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE,
		// };

		CHECK_OPTIX(optixAccelBuild(
		    Optix::getOrCreate().context, stream->getHandle(), &buildOptions, &buildInput, 1,
		    scratchpad.dTemp->getDeviceReadPtr(), scratchpad.dTemp->getSizeOf() * scratchpad.dTemp->getCount(),
		    scratchpad.dFull->getDeviceReadPtr(), scratchpad.dFull->getSizeOf() * scratchpad.dFull->getCount(), &gas,
		    nullptr, // &emitDesc,
		    0));

		// Compaction yields around 10% of memory save-up and slows down a lot (e.g. 500us per model)
		// scratchpad.doCompaction(gasHandle);
	}

	void updateGAS(const CudaStream::Ptr& stream, const DeviceSyncArray<Vec3f>::Ptr& vertices,
	               const DeviceSyncArray<Vec3i>::Ptr& indices)
	{
		OptixAccelBuildOptions updateOptions = buildOptions;
		updateOptions.operation = OPTIX_BUILD_OPERATION_UPDATE;

		// OptiX update disallows buffer sizes to change
		OptixBuildInput updateInput = buildInput;
		vertexBuffers[0] = vertices->getDeviceReadPtr();
		updateInput.triangleArray.vertexBuffers = vertexBuffers;
		updateInput.triangleArray.indexBuffer = indices->getDeviceReadPtr();

		scratchpad.resizeToFit(updateInput, updateOptions);

		// Fun fact: calling optixAccelBuild does not change anything visually, but introduces a significant slowdown
		// Investigation is needed whether it needs to be called at all (OptiX documentation says yes, but it works without)
		CHECK_OPTIX(optixAccelBuild(
		    Optix::getOrCreate().context, stream->getHandle(), &updateOptions, &updateInput, 1,
		    scratchpad.dTemp->getDeviceReadPtr(), scratchpad.dTemp->getSizeOf() * scratchpad.dTemp->getCount(),
		    scratchpad.dFull->getDeviceReadPtr(), scratchpad.dFull->getSizeOf() * scratchpad.dFull->getCount(), &gas,
		    nullptr, // &emitDesc,
		    0));
	}

	OptixTraversableHandle getGAS() const { return gas; }

private:
	ASBuildScratchpad scratchpad;

	// Shared between constructor (GAS building) and updateGAS()
	OptixBuildInput buildInput;
	unsigned triangleInputFlags;
	CUdeviceptr vertexBuffers[1];
	OptixAccelBuildOptions buildOptions;

	OptixTraversableHandle gas;
};
