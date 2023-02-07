#include <gtest/gtest.h>
#include <utils.hpp>
#include <models.hpp>
#include <rgl/api/extensions/tape.h>
#include <rgl/api/extensions/ros2.h>

#include <math/Mat3x4f.hpp>

class Tape : public RGLAutoCleanupTest {};

TEST_F(Tape, RecordPlayAllCalls)
{
	EXPECT_RGL_SUCCESS(rgl_tape_record_begin("all_calls_recording"));
	bool isTapeRecordActive = false;
	EXPECT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
	EXPECT_TRUE(isTapeRecordActive);

	rgl_mat3x4f identityTf = Mat3x4f::identity().toRGL();

	int32_t major, minor, patch;
	EXPECT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));
	EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_DEBUG, "Tape.RecordPlayAllCalls.log", true));

	rgl_mesh_t mesh = nullptr;
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, cubeVertices, ARRAY_SIZE(cubeVertices), cubeIndices, ARRAY_SIZE(cubeIndices)));
	EXPECT_RGL_SUCCESS(rgl_mesh_update_vertices(mesh, cubeVertices, ARRAY_SIZE(cubeVertices)));

	rgl_entity_t entity = nullptr;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &identityTf));

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

	rgl_node_t downsample = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_downsample(&downsample, 1.0f, 1.0f, 1.0f));

	rgl_node_t writePcd = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_write_pcd_file(&writePcd, "Tape.RecordPlayAllCalls.pcd"));

	#ifdef RGL_BUILD_ROS2_EXTENSION
		rgl_node_t ros2pub = nullptr;
		EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2pub, "pointcloud", "rgl"));

		rgl_node_t ros2pubqos = nullptr;
		rgl_qos_policy_reliability_t qos_r = QOS_POLICY_RELIABILITY_BEST_EFFORT;
		rgl_qos_policy_durability_t qos_d = QOS_POLICY_DURABILITY_VOLATILE;
		rgl_qos_policy_history_t qos_h = QOS_POLICY_HISTORY_KEEP_LAST;
		EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish_with_qos(&ros2pubqos, "pointcloud_ex", "rgl", qos_r, qos_d, qos_h, 10));
	#endif

	// Skipping rgl_node_points_visualize (user interaction needed)

	EXPECT_RGL_SUCCESS(rgl_graph_node_set_active(yield, false));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));

	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(raytrace, compact));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	std::vector<char> tmpVec;
	tmpVec.reserve(outCount * outSizeOf);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, tmpVec.data()));

	EXPECT_RGL_SUCCESS(rgl_graph_destroy(setRingIds));
	EXPECT_RGL_SUCCESS(rgl_entity_destroy(entity));
	EXPECT_RGL_SUCCESS(rgl_mesh_destroy(mesh));

	EXPECT_RGL_SUCCESS(rgl_cleanup());

	EXPECT_RGL_SUCCESS(rgl_tape_record_end());

	EXPECT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
	EXPECT_FALSE(isTapeRecordActive);

	EXPECT_RGL_SUCCESS(rgl_tape_play("all_calls_recording"));
}
