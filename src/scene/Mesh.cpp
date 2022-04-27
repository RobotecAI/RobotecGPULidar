#include <scene/Mesh.hpp>

// TODO: temporary, for testing purposes
// TODO: remove this cancer
#include "model_utils.h"
#include "TriangleMesh.h"

API_OBJECT_INSTANCE(Mesh);

Mesh::Mesh(std::size_t vertexCount, Vec3f *vertices, std::size_t indexCount, Vec3i *indices)
{
	setVertices(vertexCount, vertices);
	setIndices(indexCount, indices);
}

Mesh::Mesh(std::filesystem::path path)
{
	ModelLoader modelLoader;
	modelLoader.load_obj(path);
	std::shared_ptr<TriangleMesh> triangleMesh = modelLoader.get_triangle_mesh(0);

	std::vector<Vec3f> vertices;
	std::vector<Vec3i> indices;
	for (auto&& v : triangleMesh->vertex) {
		vertices.emplace_back(v.x, v.y, v.z);
	}
	for (auto&& i : triangleMesh->index) {
		indices.emplace_back(i.x, i.y, i.z);
	}
	setVertices(vertices.size(), vertices.data());
	setIndices(indices.size(), indices.data());
}

void Mesh::setVertices(std::size_t vertexCount, Vec3f *vertices)
{
	dVertices.copyFromHost(vertices, vertexCount);
	cachedGAS = std::nullopt;
}

void Mesh::setIndices(std::size_t indexCount, Vec3i *indices)
{
	dIndices.copyFromHost(indices, indexCount);
	cachedGAS = std::nullopt;
}

OptixTraversableHandle Mesh::getGAS()
{
	if (!cachedGAS.has_value()) {
		cachedGAS = buildGAS();
	}
	return *cachedGAS;
}

OptixTraversableHandle Mesh::buildGAS()
{
	constexpr unsigned triangleInputFlags = OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;
	const CUdeviceptr vertexBuffers[1] = {dVertices.readDeviceRaw()};

	OptixBuildInput triangleInput = {
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

	OptixAccelBuildOptions accelBuildOptions = {
		.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE
		              | OPTIX_BUILD_FLAG_ALLOW_UPDATE
		              | OPTIX_BUILD_FLAG_ALLOW_COMPACTION,
		.operation = OPTIX_BUILD_OPERATION_BUILD
	};

	scratchpad.resizeToFit(triangleInput, accelBuildOptions);

	OptixAccelEmitDesc emitDesc = {
		.result = scratchpad.dCompactedSize.readDeviceRaw(),
		.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE,
	};

	OptixTraversableHandle gasHandle;
	CHECK_OPTIX(optixAccelBuild(Optix::instance().context,
	                            nullptr, // TODO: stream
	                            &accelBuildOptions,
	                            &triangleInput,
	                            1,
	                            scratchpad.dTemp.readDeviceRaw(),
	                            scratchpad.dTemp.getByteSize(),
	                            scratchpad.dFull.readDeviceRaw(),
	                            scratchpad.dFull.getByteSize(),
	                            &gasHandle,
	                            &emitDesc,
	                            1
	));

	// TODO(prybicki): check what's the difference between compacted and full
	// TODO(prybicki): we may care more about perf than mem
	scratchpad.doCompaction(gasHandle);

	return gasHandle;
}

