#include "lidars.hpp"
#include "scenes.hpp"
#include "scene/Scene.hpp"
#include "utils.hpp"

#ifdef RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#endif

class InstanceIDTest : public RGLTest {
};

TEST_F(InstanceIDTest, BaseTest) {
	setupThreeBoxScene(nullptr);

	rgl_node_t useRaysNode = nullptr, raytraceNode = nullptr, compactNode = nullptr, formatNode = nullptr;
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.72, 0.36);

	std::vector<rgl_field_t> formatFields = {
			XYZ_F32,
			ENTITY_ID_I32
	};
	struct FormatStruct {
		Field<XYZ_F32>::type xyz;
		Field<ENTITY_ID_I32>::type entityId;
	} formatStruct;

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, formatFields.data(), formatFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, formatNode));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
	EXPECT_EQ(outSizeOf, sizeof(formatStruct));

	std::vector<FormatStruct> formatData(outCount);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(formatNode, RGL_FIELD_DYNAMIC_FORMAT, formatData.data()));


	// Infinite loop to publish pointcloud to ROS2 topic for visualization purposes. Uncomment to use. Use wisely.
//#ifdef RGL_BUILD_ROS2_EXTENSION
//    rgl_node_t ros2publishNode = nullptr;
//
//    EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2publishNode, "pointcloud", "rgl"));
//    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(formatNode, ros2publishNode));
//    while(true)
//        EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));
//#endif


	// This test is super naive, but it's a start.
	//TODO Make this test object sensitive: check that the xyz values are within the expected range for each object, and that compare its IDs.
	for (int i = 0; i < formatData.size(); ++i) {
		EXPECT_GE(formatData[i].entityId, 1);
		EXPECT_LE(formatData[i].entityId, 3);
	}

}