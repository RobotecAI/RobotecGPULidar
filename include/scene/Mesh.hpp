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
	void setVertices(std::size_t vertexCount, Vec3f *vertices);
	void setIndices(std::size_t indexCount, Vec3i *indices);
	OptixTraversableHandle getGAS();

private:
	Mesh(std::size_t vertexCount, Vec3f *vertices,
	     std::size_t indexCount, Vec3i *indices);
	Mesh(std::filesystem::path path);

	OptixTraversableHandle buildGAS();

private:
	friend APIObject<Mesh>;
	friend struct Scene;
	ASBuildScratchpad scratchpad;
	std::optional<OptixTraversableHandle> cachedGAS;
	DeviceBuffer<Vec3f> dVertices{"dVertices"};
	DeviceBuffer<Vec3i> dIndices{"dIndices"};
};
