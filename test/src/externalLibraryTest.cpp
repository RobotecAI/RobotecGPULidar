#include <gtest/gtest.h>
#include <gtest/gtest-death-test.h>
#include <helpers/commonHelpers.hpp>
#include <helpers/testPointCloud.hpp>

#if RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#include <rclcpp/rclcpp.hpp>
#endif

class ExternalLibraryTest : public RGLTest
{};

#if RGL_BUILD_ROS2_EXTENSION
/**
 * rclcpp library (part of the ROS2 extension) depends on spdlog library.
 * RGL also uses spdlog for logging purposes.
 * Using RGL with rclcpp causes an error (segmentation fault)
 * when the version of spdlog with which libraries (RGL and rclcpp) were built differs.
 *
 * This test checks if rclcpp initializes and shuts down properly.
 * rclcpp is initialized when creating the first `rgl_node_points_ros2_publish` node,
 * and shut down when destroying the last of the `rgl_node_points_ros2_publish` nodes.
 */
TEST_F(ExternalLibraryTest, RclcppInitializeAndShutDownProperly)
{
	::testing::GTEST_FLAG(death_test_style) = "threadsafe";
	ASSERT_EXIT(
	    {
		    rgl_node_t ros2pub = nullptr;
		    rgl_node_points_ros2_publish(&ros2pub, "rglTopic", "rglFrame");
		    rgl_cleanup();
		    exit(0);
	    },
	    ::testing::ExitedWithCode(0), "")
	    << "Test for rclcpp (de)initialization failed. "
	    << "It is probably caused by a mismatched version of the spdlog library for RGL and ROS2. "
	    << "Check its compatibility.";
}

TEST_F(ExternalLibraryTest, ValidateNodeROS2PublishBehaviorWhenROS2Shutdown)
{
	std::vector<rgl_field_t> fields{XYZ_VEC3_F32};
	TestPointCloud pointCloud(fields, 10);
	rgl_node_t points = pointCloud.createUsePointsNode();

	rgl_node_t format = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));

	rgl_node_t ros2pub = nullptr;

	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2pub, "pointcloud", "rglFrame"));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(points, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(format, ros2pub));

	EXPECT_RGL_SUCCESS(rgl_graph_run(points));

	rclcpp::shutdown();
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(points), "Unable to execute Ros2Node because ROS2 has been shut down.");
}

TEST_F(ExternalLibraryTest, ValidateNodeROS2PublishWithQosBehaviorWhenROS2Shutdown)
{
	std::vector<rgl_field_t> fields{XYZ_VEC3_F32};
	TestPointCloud pointCloud(fields, 10);
	rgl_node_t points = pointCloud.createUsePointsNode();

	rgl_node_t format = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));

	rgl_node_t ros2pubWithQos = nullptr;
	rgl_qos_policy_reliability_t qos_r = QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rgl_qos_policy_durability_t qos_d = QOS_POLICY_DURABILITY_VOLATILE;
	rgl_qos_policy_history_t qos_h = QOS_POLICY_HISTORY_KEEP_LAST;

	EXPECT_RGL_SUCCESS(
	    rgl_node_points_ros2_publish_with_qos(&ros2pubWithQos, "pointcloud_ex", "rglFrame", qos_r, qos_d, qos_h, 10));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(points, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(format, ros2pubWithQos));

	EXPECT_RGL_SUCCESS(rgl_graph_run(points));

	rclcpp::shutdown();
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(points), "Unable to execute Ros2Node because ROS2 has been shut down.");
}

TEST_F(ExternalLibraryTest, ValidateNodeROS2RadarscanBehaviorWhenROS2Shutdown)
{
	std::vector<rgl_field_t> fields{XYZ_VEC3_F32, DISTANCE_F32, AZIMUTH_F32, ELEVATION_F32, RADIAL_SPEED_F32};
	TestPointCloud pointCloud(fields, 10);
	rgl_node_t points = pointCloud.createUsePointsNode();

	rgl_node_t ros2radarscan = nullptr;
	rgl_qos_policy_reliability_t qos_r = QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rgl_qos_policy_durability_t qos_d = QOS_POLICY_DURABILITY_VOLATILE;
	rgl_qos_policy_history_t qos_h = QOS_POLICY_HISTORY_KEEP_LAST;

	EXPECT_RGL_SUCCESS(rgl_node_publish_ros2_radarscan(&ros2radarscan, "radarscan", "rglFrame", qos_r, qos_d, qos_h, 10));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(points, ros2radarscan));

	EXPECT_RGL_SUCCESS(rgl_graph_run(points));

	rclcpp::shutdown();
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(points), "Unable to execute Ros2Node because ROS2 has been shut down.");
}

TEST_F(ExternalLibraryTest, ValidateAllNodesROS2BehaviorsWhenROS2Shutdown)
{
	std::vector<rgl_field_t> fields{XYZ_VEC3_F32, DISTANCE_F32, AZIMUTH_F32, ELEVATION_F32, RADIAL_SPEED_F32};
	TestPointCloud pointCloud(fields, 10);
	rgl_node_t points = pointCloud.createUsePointsNode();

	rgl_node_t format = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));

	rgl_node_t ros2pub = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2pub, "pointcloud", "rglFrame"));

	rgl_node_t ros2pubWithQos = nullptr;
	rgl_qos_policy_reliability_t qos_r = QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rgl_qos_policy_durability_t qos_d = QOS_POLICY_DURABILITY_VOLATILE;
	rgl_qos_policy_history_t qos_h = QOS_POLICY_HISTORY_KEEP_LAST;
	EXPECT_RGL_SUCCESS(
	    rgl_node_points_ros2_publish_with_qos(&ros2pubWithQos, "pointcloud_ex", "rglFrame", qos_r, qos_d, qos_h, 10));

	rgl_node_t ros2radarscan = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_publish_ros2_radarscan(&ros2radarscan, "radarscan", "rglFrame", qos_r, qos_d, qos_h, 10));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(points, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(format, ros2pub));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(format, ros2pubWithQos));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(points, ros2radarscan));

	EXPECT_RGL_SUCCESS(rgl_graph_run(points));

	rclcpp::shutdown();

	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(points), "Unable to execute Ros2Node because ROS2 has been shut down.");
}
#endif
