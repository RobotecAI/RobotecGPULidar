#include <helpers/commonHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/graphHelpers.hpp>
#include <helpers/testPointCloud.hpp>

#include <graph/NodesRos2.hpp>

#include <rclcpp/rclcpp.hpp>

// TODO: replace with API calls
#include <api/apiCommon.hpp>

TEST(Ros2PublishRadarScanNodeTest, should_receive_sent_data)
{
	const auto POINT_COUNT = 5;
	const auto TOPIC_NAME = "rgl_test_radar_scan";
	const auto FRAME_ID = "rgl_test_frame_id";
	const auto NODE_NAME = "rgl_test_node";
	const auto WAIT_TIME_SECS = 1;
	const auto MESSAGE_REPEATS = 3;
	TestPointCloud input({DISTANCE_F32, AZIMUTH_F32, ELEVATION_F32, RADIAL_SPEED_F32}, POINT_COUNT);

	// Create nodes
	rgl_node_t inputNode = input.createUsePointsNode(), radarScanNode = nullptr;
	ASSERT_RGL_SUCCESS(
	    rgl_node_publish_ros2_radarscan(&radarScanNode, TOPIC_NAME, FRAME_ID, QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
	                                    QOS_POLICY_DURABILITY_SYSTEM_DEFAULT, QOS_POLICY_HISTORY_SYSTEM_DEFAULT, 0));

	// Connect nodes
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(inputNode, radarScanNode));

	// Synchronization primitives
	std::atomic<int> messageCount = 0;

	// Create ROS2 subscriber to receive RadarScan messages on topic "radar_scan"
	auto node = std::make_shared<rclcpp::Node>(NODE_NAME, rclcpp::NodeOptions{});
	auto qos = rclcpp::QoS(10);
	qos.reliability(static_cast<rmw_qos_reliability_policy_t>(QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));
	qos.durability(static_cast<rmw_qos_durability_policy_t>(QOS_POLICY_DURABILITY_SYSTEM_DEFAULT));
	qos.history(static_cast<rmw_qos_history_policy_t>(QOS_POLICY_HISTORY_SYSTEM_DEFAULT));
	auto subscriber = node->create_subscription<radar_msgs::msg::RadarScan>(
	    TOPIC_NAME, qos, [&](const radar_msgs::msg::RadarScan::SharedPtr msg) {
		    EXPECT_EQ(msg->returns.size(), POINT_COUNT);
		    EXPECT_EQ(msg->header.frame_id, FRAME_ID);
		    EXPECT_NE(msg->header.stamp.sec + msg->header.stamp.nanosec, 0);

		    for (int i = 0; i < POINT_COUNT; ++i) {
			    EXPECT_EQ(msg->returns[i].range, input.getFieldValue<DISTANCE_F32>(i));
			    EXPECT_EQ(msg->returns[i].azimuth, input.getFieldValue<AZIMUTH_F32>(i));
			    EXPECT_EQ(msg->returns[i].elevation, input.getFieldValue<ELEVATION_F32>(i));
			    EXPECT_EQ(msg->returns[i].doppler_velocity, input.getFieldValue<RADIAL_SPEED_F32>(i));
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

	rclcpp::shutdown();
}