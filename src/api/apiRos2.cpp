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

#include <rgl/api/extensions/ros2.h>

#include <api/apiCommon.hpp>

#include <graph/NodesRos2.hpp>

extern "C" {

RGL_API rgl_status_t
rgl_node_points_ros2_publish(rgl_node_t* node, const char* topic_name, const char* frame_id)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_points_ros2_publish(node={}, topic_name={}, frame_id={})",
		          repr(node), topic_name, frame_id);
		CHECK_ARG(topic_name != nullptr);
		CHECK_ARG(topic_name[0] != '\0');
		CHECK_ARG(frame_id != nullptr);
		CHECK_ARG(frame_id[0] != '\0');

		createOrUpdateNode<Ros2PublishPointsNode>(node, topic_name, frame_id);
	});
	TAPE_HOOK(node, topic_name, frame_id);
	return status;
}

void TapePlay::tape_node_points_ros2_publish(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode[0].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_ros2_publish(&node,
		yamlNode[1].as<std::string>().c_str(),
		yamlNode[2].as<std::string>().c_str());
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_points_ros2_publish_with_qos(
	rgl_node_t* node, const char* topic_name, const char* frame_id,
	rgl_qos_policy_reliability_t qos_reliability, rgl_qos_policy_durability_t qos_durability,
	rgl_qos_policy_history_t qos_history, int32_t qos_depth)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_points_ros2_publish_with_qos(node={}, topic_name={}, frame_id={},"
		          "qos_reliability={}, qos_durability={}, qos_history={}, qos_depth={})",
		          repr(node), topic_name, frame_id,
		          qos_reliability, qos_durability, qos_history, qos_depth);
		CHECK_ARG(topic_name != nullptr);
		CHECK_ARG(topic_name[0] != '\0');
		CHECK_ARG(frame_id != nullptr);
		CHECK_ARG(frame_id[0] != '\0');
		CHECK_ARG(qos_depth >= 0);

		createOrUpdateNode<Ros2PublishPointsNode>(
			node, topic_name, frame_id, qos_reliability, qos_durability, qos_history, qos_depth);
	});
	TAPE_HOOK(node, topic_name, frame_id, qos_reliability, qos_durability, qos_history, qos_depth);
	return status;
}

void TapePlay::tape_node_points_ros2_publish_with_qos(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode[0].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_ros2_publish_with_qos(&node,
		yamlNode[1].as<std::string>().c_str(),
		yamlNode[2].as<std::string>().c_str(),
		(rgl_qos_policy_reliability_t)yamlNode[3].as<int>(),
		(rgl_qos_policy_durability_t)yamlNode[4].as<int>(),
		(rgl_qos_policy_history_t)yamlNode[5].as<int>(),
		yamlNode[6].as<int>());
	tapeNodes.insert(std::make_pair(nodeId, node));
}
}