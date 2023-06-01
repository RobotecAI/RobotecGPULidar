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
#include <DeviceBuffer.hpp>
#include <math/Vector.hpp>
#include <macros/cuda.hpp>
#include <macros/optix.hpp>
#include <scene/ASBuildScratchpad.hpp>

#include <filesystem>


struct Mesh : APIObject<Mesh>
{
	void updateVertices(const Vec3f* vertices, std::size_t vertexCount);
	OptixTraversableHandle getGAS();
	int getVertexCount() const { return dVertices.getElemCount(); }

	void setTexCoords(const Vec2f *texCoords, std::size_t texCoordCount);

private:
	Mesh(const Vec3f* vertices, std::size_t vertexCount,
		 const Vec3i* indices, std::size_t indexCount);

	OptixTraversableHandle buildGAS();
	void updateGAS();

private:
	friend APIObject<Mesh>;
	friend struct Scene;
	ASBuildScratchpad scratchpad;
	bool gasNeedsUpdate;
	std::optional<OptixTraversableHandle> cachedGAS;
	DeviceBuffer<Vec3f> dVertices;
	DeviceBuffer<Vec3i> dIndices;
	std::optional<DeviceBuffer<Vec2f>> dTexCoords;

	// Shared between buildGAS() and updateGAS()
	OptixBuildInput buildInput;
	CUdeviceptr vertexBuffers[1];
	unsigned triangleInputFlags;
	OptixAccelBuildOptions buildOptions;
};
