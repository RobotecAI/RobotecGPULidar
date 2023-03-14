#pragma once

#include <math/Mat3x4f.hpp>
#include <models.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

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

static void setupSceneCube(Vec3f position)
{
	rgl_entity_t cube = makeEntity(makeCubeMesh());
	rgl_mat3x4f pose = Mat3x4f::translation(position.x(), position.y(), position.z()).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube, &pose));
}

static inline void setupBoxesAlongAxes(rgl_scene_t scene)
{
	std::vector<rgl_entity_t> xs, ys, zs;
	constexpr int BOX_COUNT = 10;
	constexpr float scaleX = 1.0f;
	constexpr float scaleY = 2.0f;
	constexpr float scaleZ = 3.0f;

	// rgl_mesh_t cube = ;
	for (int i = 0; i < BOX_COUNT; ++i) {
		xs.push_back(makeEntity(makeCubeMesh(), scene));
		ys.push_back(makeEntity(makeCubeMesh(), scene));
		zs.push_back(makeEntity(makeCubeMesh(), scene));

		rgl_mat3x4f xTf = Mat3x4f::TRS({(2 * scaleX + 2) * i, 0, 0}, {45, 0, 0}, {scaleX, 1, 1}).toRGL();
		rgl_mat3x4f yTf = Mat3x4f::TRS({0, (2 * scaleY + 2) * i, 0}, {0, 45, 0}, {1, scaleY, 1}).toRGL();
		rgl_mat3x4f zTf = Mat3x4f::TRS({0, 0, (2 * scaleZ + 2) * i}, {0, 0, 45}, {1, 1, scaleZ}).toRGL();

		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(xs[i], &xTf));
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(ys[i], &yTf));
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(zs[i], &zTf));
	}
}
