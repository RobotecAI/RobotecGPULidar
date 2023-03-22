#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rgl/api/core.h>
#include <utils/testUtils.h>
#include <math/Vector.hpp>

using namespace ::testing;

class Mesh : public ::testing::Test
{
protected:
	void SetUp() override { EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, &ray_tf, 1)); }

	void TearDown() override { EXPECT_RGL_SUCCESS(rgl_lidar_destroy(lidar)); }

	rgl_mesh_t cube_mesh = nullptr;
	rgl_entity_t entity = nullptr;
	rgl_lidar_t lidar = nullptr;

	rgl_mat3x4f entity_tf = {
	    .value = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 5}}
    };
	rgl_mat3x4f ray_tf = {
	    .value = {
	              {1, 0, 0, 0},
	              {0, 1, 0, 0},
	              {0, 0, 1, 0},
	              }
    };

	rgl_vec3f results[1];
	int hitpointCount = 0;
};

TEST_F(Mesh, DeleteMesh)
{
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length));
	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, cube_mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entity_tf));
	getLidarResults(lidar, &hitpointCount, results);

	EXPECT_EQ(hitpointCount, 1);
	EXPECT_FLOAT_EQ(results[0].value[2], 4.0f);

	EXPECT_RGL_SUCCESS(rgl_mesh_destroy(cube_mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entity_tf));

	getLidarResults(lidar, &hitpointCount, results);

	EXPECT_EQ(hitpointCount, 1);
	EXPECT_FLOAT_EQ(results[0].value[2], 4.0f);

	EXPECT_RGL_SUCCESS(rgl_entity_destroy(entity));
	getLidarResults(lidar, &hitpointCount, results);

	EXPECT_EQ(hitpointCount, 0);
}

TEST_F(Mesh, SetVertices)
{
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length));
	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, cube_mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entity_tf));

	getLidarResults(lidar, &hitpointCount, results);

	EXPECT_EQ(hitpointCount, 1);
	EXPECT_FLOAT_EQ(results[0].value[2], 4.0f);

	EXPECT_RGL_SUCCESS(rgl_mesh_update_vertices(cube_mesh, cube_vertices_big, cube_vertices_big_length));

	getLidarResults(lidar, &hitpointCount, results);

	EXPECT_EQ(hitpointCount, 1);
	EXPECT_FLOAT_EQ(results[0].value[2], 3.0f);

	EXPECT_RGL_SUCCESS(rgl_mesh_destroy(cube_mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_destroy(entity));
}

class MeshSharing : public ::testing::Test
{
protected:
	static constexpr int ENTITY_COUNT_X = 3;
	static constexpr int ENTITY_COUNT_Y = 3;
	void SetUp() override
	{
		EXPECT_RGL_SUCCESS(
		    rgl_mesh_create(&cube_small, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length));
		EXPECT_RGL_SUCCESS(
		    rgl_mesh_create(&cube_big, cube_vertices_big, cube_vertices_length, cube_indices, cube_indices_length));


		rgl_mat3x4f rays[ENTITY_COUNT_X * ENTITY_COUNT_Y];
		rgl_entity_t entities[ENTITY_COUNT_Y][ENTITY_COUNT_X];
		for (int y = 0; y < ENTITY_COUNT_Y; ++y) {
			for (int x = 0; x < ENTITY_COUNT_X; ++x) {
				rays[ENTITY_COUNT_X * y + x] = {
				    .value = {
				              {1, 0, 0, 10 * static_cast<float>(x)},
				              {0, 1, 0, 10 * static_cast<float>(y)},
				              {0, 0, 1, 0},
				              }
                };
				rgl_mat3x4f entity_tf = {
				    .value = {
				              {1, 0, 0, 10 * static_cast<float>(x)},
				              {0, 1, 0, 10 * static_cast<float>(y)},
				              {0, 0, 1, 10},
				              }
                };
				EXPECT_RGL_SUCCESS(rgl_entity_create(&entities[y][x], nullptr, ((x % 2) == 0) ? cube_small : cube_big));
				EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entities[y][x], &entity_tf));
			}
		}

		EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, rays, sizeof(rays) / sizeof(*rays)));
	}

	void TearDown() override { rgl_cleanup(); }

	rgl_lidar_t lidar = nullptr;
	rgl_mesh_t cube_small = nullptr;
	rgl_mesh_t cube_big = nullptr;
};

TEST_F(MeshSharing, TwoMeshesOneUpdated)
{
	auto check = [&](std::vector<float> distOfX) {
		int hitpointCount;
		Vec3f results[ENTITY_COUNT_X * ENTITY_COUNT_Y];
		EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(nullptr, lidar));
		EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, &hitpointCount));
		EXPECT_EQ(hitpointCount, sizeof(results) / sizeof(*results));
		EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));

		for (int y = 0; y < ENTITY_COUNT_Y; ++y) {
			for (int x = 0; x < ENTITY_COUNT_X; ++x) {
				EXPECT_FLOAT_EQ(results[ENTITY_COUNT_X * y + x].x(), 10.0f * x);
				EXPECT_FLOAT_EQ(results[ENTITY_COUNT_X * y + x].y(), 10.0f * y);
				EXPECT_FLOAT_EQ(results[ENTITY_COUNT_X * y + x].z(), distOfX[x]);
				// fmt::print("[x={}][y={}] = {}\n", x, y, results[y * ENTITY_COUNT_X + x]);
			}
		}
	};

	check({9.0, 8.0, 9.0});
	rgl_mesh_update_vertices(cube_small, cube_vertices_big, cube_vertices_length);
	check({8.0, 8.0, 8.0});
}