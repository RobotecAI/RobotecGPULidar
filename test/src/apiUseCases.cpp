#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rgl/api/apiUnitTests.h>

#include <testModels.hpp>
#include <rglCheck.hpp>

using namespace ::testing;

struct SingleRaySingleCube : public ::testing::Test {
	rgl_mat3x4f ray_tf = {
		.value = {
			{1, 0, 0, 0},
			{0, 1, 0, 0},
			{0, 0, 1, 0},
		}
	};
	rgl_mat3x4f entity_tf = {
		.value = {
			{1, 0, 0, 0},
			{0, 1, 0, 0},
			{0, 0, 1, 5}
		}
	};

	rgl_mesh_t mesh;
	rgl_entity_t entity;
	rgl_lidar_t lidar;

	void SetUp() override {
		RGL_CHECK(rgl_configure_logging(RGL_LOG_LEVEL_ALL, nullptr, true));
		RGL_CHECK(rgl_mesh_create(&mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length));
		RGL_CHECK(rgl_entity_create(&entity, NULL, mesh));
		RGL_CHECK(rgl_entity_set_pose(entity, &entity_tf));
		RGL_CHECK(rgl_lidar_create(&lidar, &ray_tf, 1));
	}

	void TearDown() override {
		if (lidar != nullptr) {
			rgl_lidar_destroy(lidar);
		}
		if (entity != nullptr) {
			rgl_entity_destroy(entity);
		}
		if (mesh != nullptr) {
			rgl_mesh_destroy(mesh);
		}
	}
};


TEST_F(SingleRaySingleCube, DeleteEntity)
{
	// std::vector<rgl_vec3f> verticesModified;
	// for (auto&& vec : cube_vertices) {
	// 	verticesModified.push_back({.value={2.0f * vec.value[0], 2.0f* vec.value[1], 2.0f * vec.value[2]}});
	// }
	// // RGL_CHECK(rgl_mesh_update_vertices(mesh, verticesModified.data(), verticesModified.size()));

	RGL_CHECK(rgl_entity_destroy(entity));
	entity = nullptr;

	RGL_CHECK(rgl_lidar_raytrace_async(nullptr, lidar));

	int hitpoint_count = 0;
	rgl_vec3f results[1] = {0};
	RGL_CHECK(rgl_lidar_get_output_size(lidar, &hitpoint_count));
	RGL_CHECK(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));

	EXPECT_THAT(hitpoint_count, testing::Eq(0));
	// EXPECT_THAT(results[0].value[0], 0.0f);
	// EXPECT_THAT(results[0].value[1], 0.0f);
	// EXPECT_THAT(results[0].value[2], 3.0f);
}
