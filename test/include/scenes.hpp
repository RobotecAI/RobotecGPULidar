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

static inline void setupThreeBoxScene(rgl_scene_t scene)
{
        std::vector<rgl_entity_t> xs, ys, zs;

        rgl_entity_t boxA, boxB, boxC;

        boxA = makeEntity(makeCubeMesh(), scene);
        boxB = makeEntity(makeCubeMesh(), scene);
        boxC = makeEntity(makeCubeMesh(), scene);

        rgl_mat3x4f boxATf = Mat3x4f::TRS({ 10, -5, 0 }, { 0, 0, 0 }, { 1, 1, 1 }).toRGL();
        rgl_mat3x4f boxBTf = Mat3x4f::TRS({ 10, 0, 0 }, { 0, 0, 0 }, { 1, 1, 1 }).toRGL();
        rgl_mat3x4f boxCTf = Mat3x4f::TRS({ 10, 5, 0 }, { 0, 0, 0 }, { 1, 1, 1 }).toRGL();

        EXPECT_RGL_SUCCESS(rgl_entity_set_pose(boxA, &boxATf));
        EXPECT_RGL_SUCCESS(rgl_entity_set_pose(boxB, &boxBTf));
        EXPECT_RGL_SUCCESS(rgl_entity_set_pose(boxC, &boxCTf));

        EXPECT_RGL_SUCCESS(rgl_entity_set_id(boxA, 1));
        EXPECT_RGL_SUCCESS(rgl_entity_set_id(boxB, 2));
        EXPECT_RGL_SUCCESS(rgl_entity_set_id(boxC, 3));
}
