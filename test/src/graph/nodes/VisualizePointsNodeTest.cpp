// Test for rgl_node_points_visualize. May not work inside docker (graphical environment needed). Uncomment to use.

//#if RGL_BUILD_PCL_EXTENSION
//#include <helpers/commonHelpers.hpp>
//#include <helpers/sceneHelpers.hpp>
//#include <helpers/lidarHelpers.hpp>
//
//#include <math/Mat3x4f.hpp>
//#include <rgl/api/extensions/pcl.h>
//
//#include <thread>
//
//class VisualizePointsNodeTest : public RGLTest
//{};
//
//TEST_F(VisualizePointsNodeTest, Visualize)
//{
//	using namespace std::chrono_literals;
//	auto mesh = makeCubeMesh();
//
//	auto entity = makeEntity(mesh);
//	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
//	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));
//
//	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, transformPts=nullptr, compact=nullptr;
//	rgl_node_t visualize=nullptr;
//	rgl_node_t visualize2=nullptr;
//	rgl_node_t visualize3=nullptr;
//
//	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 3.6, 1.8);
//	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
//	rgl_mat3x4f zeroTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
//
//	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
//	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
//	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
//	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
//	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &zeroTf));
//	EXPECT_RGL_SUCCESS(rgl_node_points_visualize(&visualize,  "aaa", 800, 600, false));
//	EXPECT_RGL_SUCCESS(rgl_node_points_visualize(&visualize2,  "bbb", 800, 600, false));
//
//	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
//	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
//	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
//	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPts));
//	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPts, visualize));
//	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPts, visualize2));
//
//	for (int i = 0; i < 300; ++i) {
//		rgl_mat3x4f ePoseTf = Mat3x4f::TRS({0,0,0},{i,i,0}).toRGL();
//		ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &ePoseTf));
//		EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
//		std::this_thread::sleep_for(16ms);
//
//		if (i == 200) {
//		EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(transformPts, visualize2));
//		EXPECT_RGL_SUCCESS(rgl_graph_destroy(visualize2));
//		}
//
//		if (i == 100) {
//		EXPECT_RGL_SUCCESS(rgl_node_points_visualize(&visualize3,  "ccc", 800, 600, false));
//		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPts, visualize3));
//		}
//	}
//}
//#endif