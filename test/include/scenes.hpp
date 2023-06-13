#pragma once

#include <math/Mat3x4f.hpp>
#include <models.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>
#include <scene/Entity.hpp>

static rgl_mesh_t makeCubeMesh()
{
	rgl_mesh_t mesh = nullptr;
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, cubeVertices, ARRAY_SIZE(cubeVertices), cubeIndices, ARRAY_SIZE(cubeIndices)));
	EXPECT_THAT(mesh, ::testing::NotNull());
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
                                    const Vec3f &position = {0, 0, 0},
                                    const Vec3f &rotation = {0, 0, 0},
                                    const Vec3f &scale = {1, 1, 1},
                                    std::optional<int> id = std::nullopt)
{
	rgl_entity_t boxEntity = makeEntity(makeCubeMesh(), scene);

	rgl_mat3x4f boxTransform = Mat3x4f::TRS(position, rotation, scale).toRGL();

	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(boxEntity, &boxTransform));

	if (id.has_value()) {
		EXPECT_RGL_SUCCESS(rgl_entity_set_id(boxEntity, id.value()));
	}
}

static inline void setupBoxesAlongAxes(rgl_scene_t scene) {

	constexpr int BOX_COUNT = 10;
	constexpr float scaleX = 1.0f;
	constexpr float scaleY = 2.0f;
	constexpr float scaleZ = 3.0f;

	for (int i = 0; i < BOX_COUNT; ++i) {

		spawnCubeOnScene(scene, {(2 * scaleX + 2) * i, 0, 0}, {45, 0, 0}, {scaleX, 1, 1});
		spawnCubeOnScene(scene, {0, (2 * scaleY + 2) * i, 0}, {0, 45, 0}, {1, scaleY, 1});
		spawnCubeOnScene(scene, {0, 0, (2 * scaleZ + 2) * i}, {0, 0, 45}, {1, 1, scaleZ});
	}
}
