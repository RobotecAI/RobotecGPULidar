#include <gtest/gtest.h>
#include <rgl/api/experimental.h>
#include <Logger.h>
#include <utils/testUtils.h>
#include <rgl/api/tape_api.h>

TEST(EndToEnd, RecordReadmeExample)
{
	int major, minor, patch;
	EXPECT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));
	RGL_INFO("RGL version: {}.{}.{}", major, minor, patch);

	EXPECT_RGL_SUCCESS(rgl_tape_record_begin("recording"));

	rgl_mesh_t cube_mesh = 0;
	EXPECT_RGL_SUCCESS(
		rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length));

	// Put an entity on the default scene
	rgl_entity_t cube_entity = 0;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&cube_entity, NULL, cube_mesh));

	// Set position of the cube entity to (0, 0, 5)
	rgl_mat3x4f entity_tf = {
		.value = {
			{1, 0, 0, 0},
			{0, 1, 0, 0},
			{0, 0, 1, 5}
		}
	};
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube_entity, &entity_tf));

	// Create a description of lidar that sends 1 ray
	// By default, lidar will have infinite ray range
	// and will be placed in (0, 0, 0), facing positive Z
	rgl_lidar_t lidar = 0;
	rgl_mat3x4f ray_tf = {
		.value = {
			{1, 0, 0, 0},
			{0, 1, 0, 0},
			{0, 0, 1, 0},
		}
	};
	EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, &ray_tf, 1));

	// Start raytracing on the default scene
	EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(NULL, lidar));

	// Wait for raytracing (if needed) and collect results
	int hitpoint_count = 0;
	rgl_vec3f results[1] = {0};
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, &hitpoint_count));
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));

	printf("Got %d hitpoint(s)\n", hitpoint_count);
	for (int i = 0; i < hitpoint_count; ++i) {
		printf("- (%.2f, %.2f, %.2f)\n", results[i].value[0], results[i].value[1], results[i].value[2]);
	}

	ASSERT_EQ(hitpoint_count, 1);
	ASSERT_FLOAT_EQ(results[0].value[2], 4.0f);

	rgl_tape_record_end();
	EXPECT_RGL_SUCCESS(rgl_cleanup());

	EXPECT_RGL_SUCCESS(rgl_tape_play("recording"));

	hitpoint_count = 0;
	// rerun the test to check if scene was loaded properly from rgl_record
	EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, &ray_tf, 1));

	// Start raytracing on the default scene
	EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(NULL, lidar));

	// Wait for raytracing (if needed) and collect results
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, &hitpoint_count));
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));

	printf("Got %d hitpoint(s)\n", hitpoint_count);
	for (int i = 0; i < hitpoint_count; ++i) {
		printf("- (%.2f, %.2f, %.2f)\n", results[i].value[0], results[i].value[1], results[i].value[2]);
	}

	ASSERT_EQ(hitpoint_count, 1);
	ASSERT_FLOAT_EQ(results[0].value[2], 4.0f);

	EXPECT_RGL_SUCCESS(rgl_cleanup());

	EXPECT_RGL_SUCCESS(rgl_tape_record_begin("recording"));

	EXPECT_RGL_SUCCESS(
		rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length));

	// Put an entity on the default scene
	EXPECT_RGL_SUCCESS(rgl_entity_create(&cube_entity, NULL, cube_mesh));

	// Set position of the cube entity to (0, 0, 5)
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube_entity, &entity_tf));

	EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, &ray_tf, 1));

	// Start raytracing on the default scene
	EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(NULL, lidar));

	// Wait for raytracing (if needed) and collect results
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, &hitpoint_count));
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));

	printf("Got %d hitpoint(s)\n", hitpoint_count);
	for (int i = 0; i < hitpoint_count; ++i) {
		printf("- (%.2f, %.2f, %.2f)\n", results[i].value[0], results[i].value[1], results[i].value[2]);
	}

	ASSERT_EQ(hitpoint_count, 1);
	ASSERT_FLOAT_EQ(results[0].value[2], 4.0f);

	rgl_tape_record_end();
	EXPECT_RGL_SUCCESS(rgl_cleanup());

	EXPECT_RGL_SUCCESS(rgl_tape_play("recording"));
	hitpoint_count = 0;
	// rerun the test to check if scene was loaded properly from rgl_record
	EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, &ray_tf, 1));

	// Start raytracing on the default scene
	EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(NULL, lidar));

	// Wait for raytracing (if needed) and collect results
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, &hitpoint_count));
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));

	printf("Got %d hitpoint(s)\n", hitpoint_count);
	for (int i = 0; i < hitpoint_count; ++i) {
		printf("- (%.2f, %.2f, %.2f)\n", results[i].value[0], results[i].value[1], results[i].value[2]);
	}

	ASSERT_EQ(hitpoint_count, 1);
	ASSERT_FLOAT_EQ(results[0].value[2], 4.0f);
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}