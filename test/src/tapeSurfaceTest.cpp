#include <gtest/gtest.h>
#include <utils.hpp>
#include <models.hpp>
#include <rgl/api/extensions/tape.h>

#ifdef RGL_BUILD_PCL_EXTENSION
#include <rgl/api/extensions/pcl.h>
#endif

#ifdef RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#endif

#include <RGLFields.hpp>
#include <math/Mat3x4f.hpp>

class TapeCase : public RGLTest {};

TEST_F(TapeCase, RecordPlayAllCalls)
{
	EXPECT_RGL_SUCCESS(rgl_tape_record_begin("all_calls_recording"));
	bool isTapeRecordActive = false;
	EXPECT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
	EXPECT_TRUE(isTapeRecordActive);

	rgl_mat3x4f identityTf = Mat3x4f::identity().toRGL();

	int32_t major, minor, patch;
	EXPECT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));

	// Note: the logging using tape test have been moved to the file loggingTests.cpp

	rgl_mesh_t mesh = nullptr;
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, cubeVertices, ARRAY_SIZE(cubeVertices), cubeIndices, ARRAY_SIZE(cubeIndices)));
	EXPECT_RGL_SUCCESS(rgl_mesh_update_vertices(mesh, cubeVertices, ARRAY_SIZE(cubeVertices)));

	rgl_entity_t entity = nullptr;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &identityTf));

	rgl_texture_t texture = nullptr;
	int width = 1024;
	int height = 2048;
	auto textureRawData = generateCheckerboardTexture<float>(width, height);

	EXPECT_RGL_SUCCESS(rgl_texture_create(&texture, textureRawData, width, height));
	EXPECT_RGL_SUCCESS(rgl_mesh_set_texture_coords(mesh, cubeUVs, 8));
	EXPECT_RGL_SUCCESS(rgl_entity_set_intensity_texture(entity, texture));

	EXPECT_RGL_SUCCESS(rgl_scene_set_time(nullptr, 1.5 * 1e9));

	rgl_node_t useRays = nullptr;
	std::vector<rgl_mat3x4f> rays = {identityTf, identityTf};
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));

	rgl_node_t setRingIds = nullptr;
	std::vector<int> rings = {0, 1};
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIds, rings.data(), rings.size()));

	rgl_node_t transformRays = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformRays, &identityTf));

	rgl_node_t transformPoints = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPoints, &identityTf));

	rgl_node_t raytrace = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 100));

	rgl_node_t format = nullptr;
	std::vector<rgl_field_t> fields = {RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32};
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));

	rgl_node_t yield = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&yield, fields.data(), fields.size()));

	rgl_node_t compact = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));

	rgl_node_t spatialMerge = nullptr;
	std::vector<rgl_field_t> sMergeFields = {RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32, RGL_FIELD_PADDING_32};
	EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMerge, sMergeFields.data(), sMergeFields.size()));

	rgl_node_t temporalMerge = nullptr;
	std::vector<rgl_field_t> tMergeFields = {RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32, RGL_FIELD_PADDING_32};
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMerge, tMergeFields.data(), tMergeFields.size()));

	rgl_node_t usePoints = nullptr;
	std::vector<rgl_field_t> usePointsFields = {RGL_FIELD_XYZ_F32};
	std::vector<::Field<XYZ_F32>::type> usePointsData = {{1, 2, 3}, {4, 5, 6}};
	EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePoints, usePointsData.data(), usePointsData.size(), usePointsFields.data(), usePointsFields.size()));

	#ifdef RGL_BUILD_ROS2_EXTENSION
		rgl_node_t ros2pub = nullptr;
		EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2pub, "pointcloud", "rgl"));

		rgl_node_t ros2pubqos = nullptr;
		rgl_qos_policy_reliability_t qos_r = QOS_POLICY_RELIABILITY_BEST_EFFORT;
		rgl_qos_policy_durability_t qos_d = QOS_POLICY_DURABILITY_VOLATILE;
		rgl_qos_policy_history_t qos_h = QOS_POLICY_HISTORY_KEEP_LAST;
		EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish_with_qos(&ros2pubqos, "pointcloud_ex", "rgl", qos_r, qos_d, qos_h, 10));
	#endif

	rgl_node_t noiseAngularRay = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&noiseAngularRay, 0.1f, 0.1f, RGL_AXIS_X));

	rgl_node_t noiseAngularHitpoint = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_hitpoint(&noiseAngularHitpoint, 0.1f, 0.1f, RGL_AXIS_X));

	rgl_node_t noiseDistance = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&noiseDistance, 0.1f, 0.1f, 0.01f));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));

	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(raytrace, compact));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	#ifdef RGL_BUILD_PCL_EXTENSION
		rgl_node_t downsample = nullptr;
		EXPECT_RGL_SUCCESS(rgl_node_points_downsample(&downsample, 1.0f, 1.0f, 1.0f));

		// Have to be executed after rgl_graph_run. Graph::run() must set field XYZ_F32 in raytrace.
		EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(raytrace, "Tape.RecordPlayAllCalls.pcd"));

		// Skipping rgl_node_points_visualize (user interaction needed)
	#endif

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	std::vector<char> tmpVec;
	tmpVec.reserve(outCount * outSizeOf);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, tmpVec.data()));

	EXPECT_RGL_SUCCESS(rgl_graph_destroy(setRingIds));
	EXPECT_RGL_SUCCESS(rgl_entity_destroy(entity));
	EXPECT_RGL_SUCCESS(rgl_mesh_destroy(mesh));
	delete[] textureRawData;

	EXPECT_RGL_SUCCESS(rgl_cleanup());

	EXPECT_RGL_SUCCESS(rgl_tape_record_end());

	EXPECT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
	EXPECT_FALSE(isTapeRecordActive);

	EXPECT_RGL_SUCCESS(rgl_tape_play("all_calls_recording"));
}
