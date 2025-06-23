#include <helpers/commonHelpers.hpp>
#include <helpers/graphHelpers.hpp>
#include <helpers/testPointCloud.hpp>

#include <rgl/api/extensions/ros2.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Ros2PublishLaserScanNodeTest : public RGLTest
{};

TEST_F(Ros2PublishLaserScanNodeTest, should_throw_invalid_pipeline_when_ros2_shutdown_laserscan)
{
	std::vector<rgl_field_t> fields{AZIMUTH_F32, DISTANCE_F32, TIME_STAMP_F64, INTENSITY_F32};
	TestPointCloud pointCloud(fields, 10);
	rgl_node_t points = pointCloud.createUsePointsNode();

	rgl_node_t yield = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yield, fields.data(), fields.size()));

	rgl_node_t ros2pub = nullptr;

	EXPECT_RGL_SUCCESS(rgl_node_publish_ros2_laserscan(&ros2pub, "laserscan", "rglFrame"));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(points, yield));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(yield, ros2pub));

	EXPECT_RGL_SUCCESS(rgl_graph_run(points));

	rclcpp::shutdown();
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(points), "Unable to execute Ros2Node because ROS2 has been shut down.");
}

TEST_F(Ros2PublishLaserScanNodeTest, should_receive_sent_data)
{
	const auto POINT_COUNT = 5;
	const auto TOPIC_NAME = "rgl_test_laser_scan";
	const auto FRAME_ID = "rgl_test_frame_id";
	const auto NODE_NAME = "rgl_test_node";
	const auto WAIT_TIME_SECS = 1;
	const auto MESSAGE_REPEATS = 3;
	const auto START_ANGLE = 0.8f;
	const auto ANGLE_INCREMENT = 0.11f;
	const auto TIME_INCREMENT = 0.22f;

	TestPointCloud input({AZIMUTH_F32, DISTANCE_F32, TIME_STAMP_F64, INTENSITY_F32}, POINT_COUNT);

	// Expected data
	const auto expectedAngles = generateFieldValues(
	    POINT_COUNT, std::function<Field<AZIMUTH_F32>::type(int)>(
	                     [START_ANGLE, ANGLE_INCREMENT](int i) { return START_ANGLE + i * ANGLE_INCREMENT; }));
	input.setFieldValues<AZIMUTH_F32>(expectedAngles);
	const auto expectedTimes = generateFieldValues(POINT_COUNT, std::function<Field<TIME_STAMP_F64>::type(int)>(
	                                                          [TIME_INCREMENT](int i) { return i * TIME_INCREMENT; }));
	input.setFieldValues<TIME_STAMP_F64>(expectedTimes);
	const auto expectedRanges = input.getFieldValues<DISTANCE_F32>();
	const auto expectedIntensities = input.getFieldValues<INTENSITY_F32>();
	const auto [minRange, maxRange] = std::minmax_element(expectedRanges.begin(), expectedRanges.end());
	const auto expectedScanTime = expectedTimes.at(expectedTimes.size() - 1) - expectedTimes.at(0);

	// Create nodes
	rgl_node_t inputNode = input.createUsePointsNode(), laserScanNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_publish_ros2_laserscan(&laserScanNode, TOPIC_NAME, FRAME_ID));

	// Connect nodes
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(inputNode, laserScanNode));

	// Synchronization primitives
	std::atomic<int> messageCount = 0;


	// Create ROS2 subscriber to receive LaserScan messages on topic "laser_scan"
	auto node = std::make_shared<rclcpp::Node>(NODE_NAME, rclcpp::NodeOptions{});
	auto qos = rclcpp::QoS(10);
	qos.reliability(static_cast<rmw_qos_reliability_policy_t>(QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));
	qos.durability(static_cast<rmw_qos_durability_policy_t>(QOS_POLICY_DURABILITY_SYSTEM_DEFAULT));
	qos.history(static_cast<rmw_qos_history_policy_t>(QOS_POLICY_HISTORY_SYSTEM_DEFAULT));
	auto subscriber = node->create_subscription<sensor_msgs::msg::LaserScan>(
	    TOPIC_NAME, qos, [&](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
		    EXPECT_EQ(msg->header.frame_id, FRAME_ID);
		    EXPECT_NE(msg->header.stamp.sec + msg->header.stamp.nanosec, 0);

			EXPECT_NEAR(msg->angle_min, START_ANGLE, EPSILON_F);
			EXPECT_NEAR(msg->angle_max, START_ANGLE + (POINT_COUNT - 1) * ANGLE_INCREMENT, EPSILON_F);
			EXPECT_NEAR(msg->range_min, *minRange, EPSILON_F);
			EXPECT_NEAR(msg->range_max, *maxRange, EPSILON_F);
			EXPECT_NEAR(msg->angle_increment, ANGLE_INCREMENT, EPSILON_F);
			EXPECT_NEAR(msg->time_increment, TIME_INCREMENT, EPSILON_F);
			EXPECT_NEAR(msg->scan_time, expectedScanTime, EPSILON_F);

		    ASSERT_EQ(msg->ranges.size(), POINT_COUNT);
		    ASSERT_EQ(msg->intensities.size(), POINT_COUNT);
		    for (int i = 0; i < POINT_COUNT; ++i) {
				EXPECT_NEAR(msg->ranges[i], expectedRanges[i], EPSILON_F);
				EXPECT_NEAR(msg->intensities[i], expectedIntensities[i], EPSILON_F);
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