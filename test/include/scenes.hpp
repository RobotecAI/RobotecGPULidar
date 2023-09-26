#pragma once

#include <testing.hpp>
#include <models.hpp>
#include <files.hpp>

#include <math/Mat3x4f.hpp>

static rgl_mesh_t makeCubeMesh()
{
	rgl_mesh_t mesh = nullptr;
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, cubeVertices, ARRAY_SIZE(cubeVertices), cubeIndices, ARRAY_SIZE(cubeIndices)));
	EXPECT_THAT(mesh, testing::NotNull());
	return mesh;
}

static rgl_entity_t makeEntity(rgl_mesh_t mesh= nullptr, rgl_scene_t scene=nullptr)
{
	if (mesh == nullptr) {
		mesh = makeCubeMesh();
	}
	rgl_entity_t entity = nullptr;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, scene, mesh));
	EXPECT_THAT(entity, ::testing::NotNull());
	return entity;
}

static inline void spawnCubeOnScene(rgl_scene_t scene,
                                    const Mat3x4f &transform,
                                    std::optional<int> id = std::nullopt)
{
	rgl_entity_t boxEntity = makeEntity(makeCubeMesh(), scene);

	auto rglTransform = transform.toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(boxEntity, &rglTransform));

	if (id.has_value()) {
		EXPECT_RGL_SUCCESS(rgl_entity_set_id(boxEntity, id.value()));
	}
}

static inline void setupBoxesAlongAxes(rgl_scene_t scene)
{
	constexpr int BOX_COUNT = 10;
	constexpr float scaleX = 1.0f;
	constexpr float scaleY = 2.0f;
	constexpr float scaleZ = 3.0f;

	for (int i = 0; i < BOX_COUNT; ++i) {
		spawnCubeOnScene(scene, Mat3x4f::TRS({(2 * scaleX + 2) * i, 0, 0}, {45, 0, 0}, {scaleX, 1, 1}));
		spawnCubeOnScene(scene, Mat3x4f::TRS({0, (2 * scaleY + 2) * i, 0}, {0, 45, 0}, {1, scaleY, 1}));
		spawnCubeOnScene(scene, Mat3x4f::TRS({0, 0, (2 * scaleZ + 2) * i}, {0, 0, 45}, {1, 1, scaleZ}));
	}
}

static rgl_mesh_t loadMesh(std::filesystem::path path)
{
	rgl_mesh_t mesh = nullptr;
	std::vector<rgl_vec3f> vs = loadVec<rgl_vec3f>(path.string() + std::string(".vertices"));
	std::vector<rgl_vec3i> is = loadVec<rgl_vec3i>(path.string() + std::string(".indices"));
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, vs.data(), vs.size(), is.data(), is.size()));
	return mesh;
}
