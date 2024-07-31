#include <gtest/gtest.h>
#include <rgl/api/core.h>
#include <helpers/sceneHelpers.hpp>

TEST(EndToEnd, ReadmeExample)
{
	int major, minor, patch;
	EXPECT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));
	printf("RGL version: %d.%d.%d\n", major, minor, patch);

	// Create a mesh by using helper function from test/include/scenes.hpp
	rgl_mesh_t cube_mesh = makeCubeMesh();

	// Put an entity on the default scene
	rgl_entity_t cube_entity = nullptr;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&cube_entity, nullptr, cube_mesh));

	// Set position of the cube entity to (0, 0, 5)
	rgl_mat3x4f entity_tf = {
	    .value = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 5}}
    };
	EXPECT_RGL_SUCCESS(rgl_entity_set_transform(cube_entity, &entity_tf));

	// Create a graph representation of a lidar that sends 1 ray and is situated at (x,y,z) = (0, 0, 0), facing positive Z
	rgl_mat3x4f ray_tf = {
	    .value = {
	              {1, 0, 0, 0},
	              {0, 1, 0, 0},
	              {0, 0, 1, 0},
	              }
    };

	rgl_node_t useRays = nullptr, raytrace = nullptr;

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, &ray_tf, 1));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, raytrace));

	// you can run the graph using any one of its nodes
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Wait for the Graph to run (if needed) and collect results
	int32_t hitpoint_count;
	int32_t point_size;
	rgl_vec3f results[1];
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(raytrace, RGL_FIELD_XYZ_VEC3_F32, &hitpoint_count, &point_size));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(raytrace, RGL_FIELD_XYZ_VEC3_F32, &results));

	printf("Got %d hitpoint(s)\n", hitpoint_count);
	for (int i = 0; i < hitpoint_count; ++i) {
		printf("- (%.2f, %.2f, %.2f)\n", results[i].value[0], results[i].value[1], results[i].value[2]);
	}

	ASSERT_EQ(hitpoint_count, 1);
	ASSERT_FLOAT_EQ(results[0].value[2], 4.0f);

	EXPECT_RGL_SUCCESS(rgl_cleanup());
}