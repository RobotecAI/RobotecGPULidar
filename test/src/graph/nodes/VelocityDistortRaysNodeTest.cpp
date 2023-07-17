#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include "utils.hpp"
#include "scenes.hpp"
#include "lidars.hpp"

#ifdef RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#endif

using namespace ::testing;


class VelocityDistortRaysNodeTest : public RGLTest, public RGLPointTestHelper {};

TEST_F(VelocityDistortRaysNodeTest, invalid_arguments)
{
	rgl_node_t velocityDistortRaysNode;
	rgl_vec3f velocity;

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_velocity_distort(nullptr, nullptr), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_velocity_distort(nullptr, &velocity), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_velocity_distort(&velocityDistortRaysNode, nullptr), "velocity != nullptr");
}

TEST_F(VelocityDistortRaysNodeTest, valid_arguments)
{
	rgl_node_t velocityDistortRaysNode = nullptr;
	rgl_vec3f velocity = { 1.0f, 1.0f, 1.0f };

	EXPECT_RGL_SUCCESS(rgl_node_rays_velocity_distort(&velocityDistortRaysNode, &velocity));
	ASSERT_THAT(velocityDistortRaysNode, testing::NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_rays_velocity_distort(&velocityDistortRaysNode, &velocity));
}

TEST_F(VelocityDistortRaysNodeTest, use_case)
{
	rgl_node_t useRays = nullptr;
	rgl_node_t transformRays = nullptr;
	rgl_node_t offsetsNode = nullptr;
	rgl_node_t velocityDistortRaysNode = nullptr;
	rgl_node_t raytrace = nullptr;

	// Prepare scene
	setupBoxesAlongAxes(nullptr);
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 360, 0.36, 0.36);
	const float rayCount = rays.size();
	std::vector<float> timeOffsets = generateTimeOffsetsForRays(rayCount);
	const rgl_vec3f lidarVelocity = { 0.0f, 1.0f, 0.0f };

	// Create nodes
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rayCount));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformRays, &translationTestTransform));
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_time_offsets(&offsetsNode, timeOffsets.data(), rayCount));
	EXPECT_RGL_SUCCESS(rgl_node_rays_velocity_distort(&velocityDistortRaysNode, &lidarVelocity));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));

	// Connect nodes into graph
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, transformRays));
	//EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformRays, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformRays, offsetsNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(offsetsNode, velocityDistortRaysNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(velocityDistortRaysNode, raytrace));

	//rgl_configure_logging(RGL_LOG_LEVEL_DEBUG,nullptr,true);
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	std::vector<::Field<XYZ_F32>::type> outPoints;
	outPoints.resize(rayCount);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(raytrace, XYZ_F32, outPoints.data()));

	// Infinite loop to publish point cloud to ROS2 topic for visualization purposes. Uncomment to use. Use wisely.
#ifdef RGL_BUILD_ROS2_EXTENSION
//		rgl_node_t formatNode = nullptr, ros2publishNode = nullptr;
//
//		std::vector<rgl_field_t> yieldFields = {
//				XYZ_F32,
//				INTENSITY_F32,
//				IS_HIT_I32
//		};
//
//		EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, yieldFields.data(), yieldFields.size()));
//		EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2publishNode, "pointcloud", "rgl"));
//
//		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, formatNode));
//		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(formatNode, ros2publishNode));
//
//		while(true)
//		{
//				EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
//		}
#endif


}
