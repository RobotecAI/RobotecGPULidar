#include <helpers/commonHelpers.hpp>
#include <helpers/graphHelpers.hpp>
#include <helpers/testPointCloud.hpp>

#include <rgl/api/extensions/ros2.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

TEST_F(Ros2PublishPointsNodeTest, should_receive_sent_data)
{
	const auto POINT_COUNT = 5;
	const auto TOPIC_NAME = "rgl_test_pointcloud2";
	const auto FRAME_ID = "rgl_test_frame_id";
	const auto NODE_NAME = "rgl_test_node";
	const auto WAIT_TIME_SECS = 1;
	const auto MESSAGE_REPEATS = 3;
	const std::vector<rgl_field_t> fields{DISTANCE_F32};
	assert(fields.size() == 1 && fields[0] == DISTANCE_F32); // Test is hardcoded to work with distance field only.
	TestPointCloud input(fields, POINT_COUNT);

	// Create nodes
	rgl_node_t inputNode = input.createUsePointsNode(), pointcloud2Node = nullptr, format = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));
	ASSERT_RGL_SUCCESS(
	    rgl_node_points_ros2_publish_with_qos(&pointcloud2Node, TOPIC_NAME, FRAME_ID, QOS_POLICY_RELIABILITY_RELIABLE,
	                                          QOS_POLICY_DURABILITY_SYSTEM_DEFAULT, QOS_POLICY_HISTORY_SYSTEM_DEFAULT, 0));

	// Connect nodes
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(inputNode, format));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(format, pointcloud2Node));

	// Synchronization primitives
	std::atomic<int> messageCount = 0;

	// Create ROS2 subscriber to receive PointCloud2 messages
	auto node = std::make_shared<rclcpp::Node>(NODE_NAME, rclcpp::NodeOptions{});
	auto qos = rclcpp::QoS(10);
	qos.reliability(static_cast<rmw_qos_reliability_policy_t>(QOS_POLICY_RELIABILITY_RELIABLE));
	qos.durability(static_cast<rmw_qos_durability_policy_t>(QOS_POLICY_DURABILITY_SYSTEM_DEFAULT));
	qos.history(static_cast<rmw_qos_history_policy_t>(QOS_POLICY_HISTORY_SYSTEM_DEFAULT));
	auto subscriber = node->create_subscription<sensor_msgs::msg::PointCloud2>(
	    TOPIC_NAME, qos, [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
		    EXPECT_EQ(msg->height * msg->width, POINT_COUNT);
		    EXPECT_EQ(msg->header.frame_id, FRAME_ID);
		    EXPECT_NE(msg->header.stamp.sec + msg->header.stamp.nanosec, 0);

		    for (int i = 0; i < POINT_COUNT; ++i) {
			    EXPECT_EQ(reinterpret_cast<const float*>(msg->data.data())[i], input.getFieldValue<DISTANCE_F32>(i));
		    }
		    ++messageCount;
	    });

	// Run
	for (int i = 0; i < MESSAGE_REPEATS; ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_run(inputNode));
	}

	// Wait for messages
	{
		auto start = std::chrono::steady_clock::now();
		do {
			rclcpp::spin_some(node);
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		} while (messageCount != MESSAGE_REPEATS &&
		         std::chrono::steady_clock::now() - start < std::chrono::seconds(WAIT_TIME_SECS));
		ASSERT_EQ(messageCount, MESSAGE_REPEATS);
	}
}
