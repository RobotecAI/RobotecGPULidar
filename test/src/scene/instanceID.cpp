#include "lidars.hpp"
#include "scenes.hpp"
#include "scene/Scene.hpp"
#include "utils.hpp"

#ifdef RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#endif

class InstanceIDTest : public RGLTest {
};

TEST_F(InstanceIDTest, BaseTest)
{
	setupThreeBoxScene(nullptr);

	rgl_node_t useRaysNode = nullptr, raytraceNode = nullptr, compactNode = nullptr, yieldNode = nullptr;
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.72, 0.36);

	std::vector<rgl_field_t> yieldFields = {
			XYZ_F32,
			ENTITY_ID_I32,
			IS_HIT_I32
	};

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, yieldFields.data(), yieldFields.size()));


	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldNode));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	std::vector<::Field<XYZ_F32>::type> outPoints;
	std::vector<::Field<ENTITY_ID_I32>::type> outID;
	std::vector<::Field<IS_HIT_I32>::type> outIsHit;

	outPoints.resize(rays.size());
	outID.resize(rays.size());
	outIsHit.resize(rays.size());

	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldNode, XYZ_F32, outPoints.data()));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldNode, ENTITY_ID_I32, outID.data()));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldNode, IS_HIT_I32, outIsHit.data()));


	// Infinite loop to publish pointcloud to ROS2 topic for visualization purposes. Uncomment to use. Use wisely.
//#ifdef RGL_BUILD_ROS2_EXTENSION
//    rgl_node_t ros2publishNode = nullptr;
//
//    EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2publishNode, "pointcloud", "rgl"));
//    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(formatNode, ros2publishNode));
//    while(true)
//        EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));
//#endif

	for (int i = 0; i < rays.size(); ++i)
	{
		if (outIsHit[i]) {
			// Check that the points are within the expected range for each object.
			if (outPoints[i][1] < -2.0f) {
				EXPECT_EQ(outID[i], 1);
			}
			if (outPoints[i][1] > -2.0f && outPoints[i][1] < 2.0f) {
				EXPECT_EQ(outID[i], 2);
			}
			if (outPoints[i][1] > 2.0f) {
				EXPECT_EQ(outID[i], DEFAULT_ENTITY_ID);
			}
		}
		else {
			EXPECT_EQ(outID[i], 0);
		}
	}
}