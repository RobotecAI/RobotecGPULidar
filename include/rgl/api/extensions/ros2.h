#pragma once

#include <rgl/api/core.h>

/**
 * Available Quality of Service reliability policies.
 */
typedef enum
{
	QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0,
	QOS_POLICY_RELIABILITY_RELIABLE = 1,
	QOS_POLICY_RELIABILITY_BEST_EFFORT = 2
} rgl_qos_policy_reliability_t;

/**
 * Available Quality of Service durability policies.
 */
typedef enum
{
	QOS_POLICY_DURABILITY_SYSTEM_DEFAULT = 0,
	QOS_POLICY_DURABILITY_TRANSIENT_LOCAL = 1,
	QOS_POLICY_DURABILITY_VOLATILE = 2
} rgl_qos_policy_durability_t;

/**
 * Available Quality of Service history policies.
 */
typedef enum
{
	QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0,
	QOS_POLICY_HISTORY_KEEP_LAST = 1,
	QOS_POLICY_HISTORY_KEEP_ALL = 2
} rgl_qos_policy_history_t;

/******************************** NODES ********************************/

/**
 * Creates or modifies Ros2PublishPointsNode.
 * The node publishes PointCloud2 message to ROS2 topic.
 * Fields and their layout in the binary data blob will be automatically determined based on preceding FormatNode.
 * Graph input: FormatNode
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param topic_name Topic name to publish on.
 * @param frame_id Frame this data is associated with.
 */
RGL_API rgl_status_t
rgl_node_points_ros2_publish(rgl_node_t* node, const char* topic_name, const char* frame_id);

/**
 * Creates or modifies Ros2PublishPointsNode.
 * The node publishes PointCloud2 message to ROS2 topic with Quality of Service specified.
 * Fields and their layout in the binary data blob will be automatically determined based on preceding FormatNode.
 * Graph input: FormatNode
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param topic_name Topic name to publish on.
 * @param frame_id Frame this data is associated with.
 * @param qos_reliability QoS reliability policy.
 * @param qos_durability QoS durability policy.
 * @param qos_history QoS history policy.
 * @param qos_depth QoS history depth. If history policy is KEEP_ALL, depth is ignored but must always be non-negative.
 */
RGL_API rgl_status_t
rgl_node_points_ros2_publish_with_qos(
	rgl_node_t* node, const char* topic_name, const char* frame_id,
	rgl_qos_policy_reliability_t qos_reliability, rgl_qos_policy_durability_t qos_durability,
	rgl_qos_policy_history_t qos_history, int32_t qos_depth);
