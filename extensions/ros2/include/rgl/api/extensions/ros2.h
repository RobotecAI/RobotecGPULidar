// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rgl/api/core.h>

/******************************** CONSTANTS ********************************/

/**
 * All constants defined below are guaranteed to be equal to RCL-defined constants.
 * They are defined as a convenience to users who do not interact with ROS2 otherwise.
 */

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
 * The node publishes a PointCloud2 message to the ROS2 topic using default Quality of Service settings.
 * Fields and their layout in the binary data blob will be automatically determined based on the preceding FormatNode.
 * The message header stamp gets time from the raytraced scene. If the scene has no time, header will get the actual time.
 * Graph input: FormatNode
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param topic_name Topic name to publish on.
 * @param frame_id Frame this data is associated with.
 */
RGL_API rgl_status_t rgl_node_points_ros2_publish(rgl_node_t* node, const char* topic_name, const char* frame_id);

/**
 * Creates or modifies Ros2PublishPointsNode.
 * The node publishes a PointCloud2 message to the ROS2 topic with Quality of Service specified.
 * Fields and their layout in the binary data blob will be automatically determined based on the preceding FormatNode.
 * The message header stamp gets time from the raytraced scene. If the scene has no time, header will get the actual time.
 * Graph input: FormatNode
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param topic_name Topic name to publish on.
 * @param frame_id Frame this data is associated with.
 * @param qos_reliability QoS reliability policy.
 * @param qos_durability QoS durability policy.
 * @param qos_history QoS history policy.
 * @param qos_history_depth QoS history depth. If history policy is KEEP_ALL, depth is ignored but must always be non-negative.
 */
RGL_API rgl_status_t rgl_node_points_ros2_publish_with_qos(rgl_node_t* node, const char* topic_name, const char* frame_id,
                                                           rgl_qos_policy_reliability_t qos_reliability,
                                                           rgl_qos_policy_durability_t qos_durability,
                                                           rgl_qos_policy_history_t qos_history, int32_t qos_history_depth);

/**
 * Creates or modifies Ros2PublishRadarScanNode.
 * The node publishes a RadarScan message to the ROS2 topic using specified Quality of Service settings.
 * The message header stamp gets time from the raytraced scene. If the scene has no time, header will get the actual time.
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param topic_name Topic name to publish on.
 * @param frame_id Frame this data is associated with.
 * @param qos_reliability QoS reliability policy.
 * @param qos_durability QoS durability policy.
 * @param qos_history QoS history policy.
 * @param qos_history_depth QoS history depth. If history policy is KEEP_ALL, depth is ignored but must always be non-negative.
 */
RGL_API rgl_status_t rgl_node_publish_ros2_radarscan(rgl_node_t* node, const char* topic_name, const char* frame_id,
                                                     rgl_qos_policy_reliability_t qos_reliability,
                                                     rgl_qos_policy_durability_t qos_durability,
                                                     rgl_qos_policy_history_t qos_history, int32_t qos_history_depth);
