#pragma once

#include <math/Mat3x4f.hpp>

static inline void setupBoxesAlongAxes(rgl_scene_t scene)
{
	std::vector<rgl_entity_t> xs, ys, zs;
	constexpr int BOX_COUNT = 10;
	constexpr float scaleX = 1.0f;
	constexpr float scaleY = 2.0f;
	constexpr float scaleZ = 3.0f;

	rgl_mesh_t cube = makeCubeMesh();
	for (int i = 0; i < BOX_COUNT; ++i) {
		xs.push_back(makeEntity(cube, scene));
		ys.push_back(makeEntity(cube, scene));
		zs.push_back(makeEntity(cube, scene));

		rgl_mat3x4f xTf = Mat3x4f::TRS({(2 * scaleX + 2) * i, 0, 0}, {0, 0, 0}, {scaleX, 1, 1}).toRGL();
		rgl_mat3x4f yTf = Mat3x4f::TRS({0, (2 * scaleY + 2) * i, 0}, {0, 0, 0}, {1, scaleY, 1}).toRGL();
		rgl_mat3x4f zTf = Mat3x4f::TRS({0, 0, (2 * scaleZ + 2) * i}, {0, 0, 0}, {1, 1, scaleZ}).toRGL();

		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(xs[i], &xTf));
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(ys[i], &yTf));
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(zs[i], &zTf));
	}
}