#pragma once

#include <optional>
#include <filesystem>

#include <helpers/commonHelpers.hpp>
#include <helpers/geometryData.hpp>

#include <math/Mat3x4f.hpp>
#include "stl_reader.h"

static rgl_mesh_t makeCubeMesh()
{
	rgl_mesh_t mesh = nullptr;
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, cubeVertices, ARRAY_SIZE(cubeVertices), cubeIndices, ARRAY_SIZE(cubeIndices)));
	EXPECT_THAT(mesh, testing::NotNull());
	return mesh;
}

static rgl_entity_t makeEntity(rgl_mesh_t mesh = nullptr)
{
	if (mesh == nullptr) {
		mesh = makeCubeMesh();
	}
	rgl_entity_t entity = nullptr;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	EXPECT_THAT(entity, ::testing::NotNull());
	return entity;
}

static inline rgl_entity_t spawnCubeOnScene(const Mat3x4f& transform, std::optional<int> id = std::nullopt)
{
	rgl_entity_t boxEntity = makeEntity(makeCubeMesh());

	auto rglTransform = transform.toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_transform(boxEntity, &rglTransform));

	if (id.has_value()) {
		EXPECT_RGL_SUCCESS(rgl_entity_set_id(boxEntity, id.value()));
	}
	return boxEntity;
}

static inline void setupBoxesAlongAxes()
{
	constexpr int BOX_COUNT = 10;
	constexpr float scaleX = 1.0f;
	constexpr float scaleY = 2.0f;
	constexpr float scaleZ = 3.0f;

	for (int i = 0; i < BOX_COUNT; ++i) {
		spawnCubeOnScene(Mat3x4f::TRS({(2 * scaleX + 2) * i, 0, 0}, {45, 0, 0}, {scaleX, 1, 1}));
		spawnCubeOnScene(Mat3x4f::TRS({0, (2 * scaleY + 2) * i, 0}, {0, 45, 0}, {1, scaleY, 1}));
		spawnCubeOnScene(Mat3x4f::TRS({0, 0, (2 * scaleZ + 2) * i}, {0, 0, 45}, {1, 1, scaleZ}));
	}
}

static inline rgl_mesh_t loadFromSTL(std::filesystem::path path)
{
	std::vector<float> vertices, _normals;
	std::vector<int32_t> indices, _solids;
	std::vector<rgl_vec3f> rgl_vertices;
	std::vector<rgl_vec3i> rgl_indices;
	stl_reader::ReadStlFile(path.string().c_str(), vertices, _normals, indices, _solids);
	assert(vertices.size() % 3 == 0);
	assert(indices.size() % 3 == 0);
	for (size_t i = 0; i < vertices.size(); i += 3) {
		rgl_vertices.push_back({vertices[i], vertices[i + 1], vertices[i + 2]});
	}
	for (size_t i = 0; i < indices.size(); i += 3) {
		rgl_indices.push_back({indices[i], indices[i + 1], indices[i + 2]});
	}
	rgl_mesh_t outMesh = nullptr;
	rgl_status_t status = rgl_mesh_create(&outMesh, rgl_vertices.data(), rgl_vertices.size(), rgl_indices.data(),
	                                      rgl_indices.size());
	if (status != RGL_SUCCESS) {
		const char* errorString = nullptr;
		rgl_get_last_error_string(&errorString);
		throw std::runtime_error(fmt::format("rgl_mesh_create: {}", errorString));
	}
	return outMesh;
}
