#include <scene/Mesh.hpp>

// TODO: temporary, for testing purposes
// TODO: remove this cancer
#include "model_utils.h"
#include "TriangleMesh.h"

#include <filesystem>

namespace fs = std::filesystem;

API_OBJECT_INSTANCE(Mesh);

Mesh::Mesh(Vec3f *vertices, size_t vertexCount, Vec3i *indices, size_t indexCount)
{
	setVertices(vertices, vertexCount);
	setIndices(indices, indexCount);
}

Mesh::Mesh(fs::path path)
{
	ModelLoader modelLoader;
	modelLoader.load_obj(path.string());
	std::shared_ptr<TriangleMesh> triangleMesh = modelLoader.get_triangle_mesh(0);

	std::vector<Vec3f> vertices;
	std::vector<Vec3i> indices;
	for (auto&& v : triangleMesh->vertex) {
		vertices.emplace_back(v.x, v.y, v.z);
	}
	for (auto&& i : triangleMesh->index) {
		indices.emplace_back(i.x, i.y, i.z);
	}
	setVertices(vertices.data(), vertices.size());
	setIndices(indices.data(), indices.size());
}

void Mesh::setVertices(Vec3f *vertices, std::size_t vertexCount)
{
	dVertices.copyFromHost(vertices, vertexCount);
	bool sizeChanged = dVertices.getElemCount() != vertexCount;
	if (sizeChanged) {
		cachedGAS.reset();
	}
	gasNeedsUpdate = true;
}

void Mesh::setIndices(Vec3i *indices, std::size_t indexCount)
{
	dIndices.copyFromHost(indices, indexCount);
	bool sizeChanged = dIndices.getElemCount() != indexCount;
	if (sizeChanged) {
		cachedGAS.reset();
	}
	gasNeedsUpdate = true;
}

OptixTraversableHandle Mesh::getGAS()
{
	if (!cachedGAS.has_value()) {
		cachedGAS = buildGAS();
	}
	if (gasNeedsUpdate) {
		updateGAS();
	}
	return *cachedGAS;
}

void Mesh::updateGAS()
{
	OptixAccelBuildOptions updateOptions = buildOptions;
	updateOptions.operation = OPTIX_BUILD_OPERATION_UPDATE;

	// OptiX update disallows buffer sizes to change
	OptixBuildInput updateInput = buildInput;
	const CUdeviceptr vertexBuffers[1] = {dVertices.readDeviceRaw()};
	updateInput.triangleArray.vertexBuffers = vertexBuffers;
	updateInput.triangleArray.indexBuffer = dIndices.readDeviceRaw();

	scratchpad.resizeToFit(updateInput, updateOptions);

	// Fun fact: calling optixAccelBuild does not change anything visually, but introduces a significant slowdown
	// Investigation is needed whether it needs to be called at all (OptiX documentation says yes, but it works without)
	CHECK_OPTIX(optixAccelBuild(Optix::instance().context,
	                            nullptr, // TODO: stream
	                            &updateOptions,
	                            &updateInput,
	                            1,
	                            scratchpad.dTemp.readDeviceRaw(),
	                            scratchpad.dTemp.getByteSize(),
	                            scratchpad.dFull.readDeviceRaw(),
	                            scratchpad.dFull.getByteSize(),
	                            &cachedGAS.value(),
	                            nullptr, // &emitDesc,
	                            0));

	gasNeedsUpdate = false;
}

OptixTraversableHandle Mesh::buildGAS()
{
	constexpr unsigned triangleInputFlags = OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;
	const CUdeviceptr vertexBuffers[1] = {dVertices.readDeviceRaw()};

	buildInput = {
		.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES,
		.triangleArray = {
			.vertexBuffers = vertexBuffers,
			.numVertices = static_cast<unsigned int>(dVertices.getElemCount()),
			.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3,
			.vertexStrideInBytes = sizeof(decltype(dVertices)::ValueType),
			.indexBuffer = dIndices.readDeviceRaw(),
			.numIndexTriplets = static_cast<unsigned int>(dIndices.getElemCount()),
			.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3,
			.indexStrideInBytes = sizeof(decltype(dIndices)::ValueType),
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
	CHECK_OPTIX(optixAccelBuild(Optix::instance().context,
	                            nullptr, // TODO: stream
	                            &buildOptions,
	                            &buildInput,
	                            1,
	                            scratchpad.dTemp.readDeviceRaw(),
	                            scratchpad.dTemp.getByteSize(),
	                            scratchpad.dFull.readDeviceRaw(),
	                            scratchpad.dFull.getByteSize(),
	                            &gasHandle,
	                            nullptr, // &emitDesc,
	                            0
	));

	// Compaction yields around 10% of memory and slows down a lot (e.g. 500us per model)
	// scratchpad.doCompaction(gasHandle);

	gasNeedsUpdate = false;
	return gasHandle;
}

