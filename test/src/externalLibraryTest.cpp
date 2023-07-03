#include <gtest/gtest.h>
#include <gtest/gtest-death-test.h>
#include <utils.hpp>

#if RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#endif

class ExternalLibraryTest : public RGLTest {};

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
	ASSERT_EXIT({
		rgl_node_t ros2pub = nullptr;
		rgl_node_points_ros2_publish(&ros2pub, "rglTopic", "rglFrame");
		rgl_cleanup();
		exit(0);
	}, ::testing::ExitedWithCode(0), "")
		<< "Test for rclcpp (de)initialization failed. "
		<< "It is probably caused by a mismatched version of the spdlog library for RGL and ROS2. "
		<< "Check its compatibility.";
}
#endif
