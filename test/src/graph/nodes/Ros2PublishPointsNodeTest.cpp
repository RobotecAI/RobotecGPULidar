#include <helpers/commonHelpers.hpp>
#include <helpers/graphHelpers.hpp>
#include <helpers/testPointCloud.hpp>

#include <rgl/api/extensions/ros2.h>

#include <rclcpp/rclcpp.hpp>

class Ros2PublishPointsNodeTest : public RGLTest
{};

TEST_F(Ros2PublishPointsNodeTest, should_throw_invalid_pipeline_when_ros2_shutdown)
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

TEST_F(Ros2PublishPointsNodeTest, should_throw_invalid_pipeline_when_ros2_shutdown_with_qos)
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


TEST_F(Ros2PublishPointsNodeTest, should_throw_invalid_pipeline_when_no_formatted_point_cloud)
{
	std::vector<rgl_field_t> fields{XYZ_VEC3_F32};
	TestPointCloud pointCloud(fields, 10);
	rgl_node_t points = pointCloud.createUsePointsNode();

	rgl_node_t ros2pub = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2pub, "pointcloud", "rglFrame"));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(points, ros2pub));

	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(ros2pub), "requires a formatted point cloud");

	rgl_node_t ros2pubWithQos = nullptr;
	rgl_qos_policy_reliability_t qos_r = QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rgl_qos_policy_durability_t qos_d = QOS_POLICY_DURABILITY_VOLATILE;
	rgl_qos_policy_history_t qos_h = QOS_POLICY_HISTORY_KEEP_LAST;

	EXPECT_RGL_SUCCESS(
	    rgl_node_points_ros2_publish_with_qos(&ros2pubWithQos, "pointcloud_ex", "rglFrame", qos_r, qos_d, qos_h, 10));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(points, ros2pubWithQos));

	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(ros2pubWithQos), "requires a formatted point cloud");
}