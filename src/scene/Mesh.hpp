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

#pragma once

#include <cstddef>

#include <optix_stubs.h>

#include <APIObject.hpp>
#include <Optix.hpp>
#include <math/Vector.hpp>
#include <macros/cuda.hpp>
#include <macros/optix.hpp>
#include <scene/ASBuildScratchpad.hpp>

#include <filesystem>
#include <memory/Array.hpp>
#include <Time.hpp>

/**
 * Represents mesh data (at the moment vertices and indices) stored on the GPU.
 * Mesh, on its own, is not bound to any scene and can be used for different scenes.
 */
struct Mesh : APIObject<Mesh>
{
	friend APIObject<Mesh>;
	friend struct Scene;

	/**
	 * Updates vertices of this mesh. Vertex count must remain unchanged, otherwise an exception is thrown.
	 * After this operation, GAS needs to be rebuilt. This is handled internally in getGAS.
	 */
	void updateVertices(const Vec3f* vertices, std::size_t vertexCount);

	/**
	 * Returns an array describing displacement of each vertex between current and previous state, due to skinning.
	 * If vertices were not skinned in the previous frame, returns NULL (equivalent to an array of zero vectors).
	 * @return Pointer do GPU-accessible array, same size as vertexCount. May be NULL.
	 */
	const Vec3f* getSkinningDisplacementSinceLastFrame() const;

	/**
	 * Sets textures coordinates to the mesh. Vertex count and texture coordinates count must be equal.
	 * After this operation, GAS needs to be rebuilt. This is handled internally in getGAS.
	 */
	void setTexCoords(const Vec2f* texCoords, std::size_t texCoordCount);

	/**
	 * Returns GAS for this mesh.
	 * If no changes occurred (e.g. call to updateVertices), then returns cached GAS.
	 * Otherwise, queues building GAS in the given stream, without synchronizing it.
	 */
	OptixTraversableHandle getGAS(CudaStream::Ptr stream);

private:
	Mesh(const Vec3f* vertices, std::size_t vertexCount, const Vec3i* indices, std::size_t indexCount);

	OptixTraversableHandle buildGAS(CudaStream::Ptr stream);
	void updateGAS(CudaStream::Ptr stream);

private:
	ASBuildScratchpad scratchpad;
	bool gasNeedsUpdate;
	std::optional<OptixTraversableHandle> cachedGAS;

	std::optional<Time> verticesCurrentUpdateTime;
	std::optional<Time> verticesFormerUpdateTime;
	DeviceSyncArray<Vec3f>::Ptr dVertices = DeviceSyncArray<Vec3f>::create();
	DeviceSyncArray<Vec3i>::Ptr dIndices = DeviceSyncArray<Vec3i>::create();
	DeviceSyncArray<Vec3f>::Ptr dVertexSkinningDisplacement = DeviceSyncArray<Vec3f>::create();
	std::optional<DeviceSyncArray<Vec2f>::Ptr> dTextureCoords;

	// Shared between buildGAS() and updateGAS()
	OptixBuildInput buildInput;
	CUdeviceptr vertexBuffers[1];
	unsigned triangleInputFlags;
	OptixAccelBuildOptions buildOptions;
};
