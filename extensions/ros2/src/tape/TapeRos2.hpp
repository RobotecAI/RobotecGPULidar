// Copyright 2023 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <tape/PlaybackState.hpp>
#include <tape/TapePlayer.hpp>

class TapeRos2
{
	static void tape_node_points_ros2_publish(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_ros2_publish_with_qos(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_publish_ros2_radarscan(const YAML::Node& yamlNode, PlaybackState& state);

	// Called once in the translation unit
	static inline bool autoExtendTapeFunctions = std::invoke([]() {
		std::map<std::string, TapeFunction> tapeFunctions = {
		    TAPE_CALL_MAPPING("rgl_node_points_ros2_publish", TapeRos2::tape_node_points_ros2_publish),
		    TAPE_CALL_MAPPING("rgl_node_points_ros2_publish_with_qos", TapeRos2::tape_node_points_ros2_publish_with_qos),
		    TAPE_CALL_MAPPING("rgl_node_publish_ros2_radarscan", TapeRos2::tape_node_publish_ros2_radarscan),
		};
		TapePlayer::extendTapeFunctions(tapeFunctions);
		return true;
	});
};
