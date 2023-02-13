// Copyright 2023 Robotec.AI
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

#include <rgl/api/extensions/pcl.h>
#include <api/apiCommon.hpp>
#include <graph/NodesPcl.hpp>

extern "C" {

RGL_API rgl_status_t
rgl_node_points_downsample(rgl_node_t* node, float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_downsample(node={}, leaf=({}, {}, {}))", repr(node), leaf_size_x, leaf_size_y, leaf_size_z);

		createOrUpdateNode<DownSamplePointsNode>(node, Vec3f{leaf_size_x, leaf_size_y, leaf_size_z});
	});
	TAPE_HOOK(node, leaf_size_x, leaf_size_y, leaf_size_z);
	return status;
}

void TapePlayer::tape_node_points_downsample(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_points_downsample(&node,
		yamlNode[1].as<float>(),
		yamlNode[2].as<float>(),
		yamlNode[3].as<float>());
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_points_write_pcd_file(rgl_node_t* node, const char* file_path)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_write_pcd_file(node={}, file={})", repr(node), file_path);
		CHECK_ARG(file_path != nullptr);
		CHECK_ARG(file_path[0] != '\0');

		createOrUpdateNode<WritePCDFilePointsNode>(node, file_path);
	});
	TAPE_HOOK(node, file_path);
	return status;
}

void TapePlayer::tape_node_points_write_pcd_file(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_points_write_pcd_file(&node, yamlNode[1].as<std::string>().c_str());
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_points_visualize(rgl_node_t* node, const char* window_name, int32_t window_width, int32_t window_height, bool fullscreen)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_visualize(node={}, window_name={}, window_width={}, window_height={}, fullscreen={})",
		          repr(node), window_name, window_width, window_height, fullscreen);
		CHECK_ARG(window_name != nullptr);
		CHECK_ARG(window_name[0] != '\0');
		CHECK_ARG(window_width > 0);
		CHECK_ARG(window_height > 0);

		createOrUpdateNode<VisualizePointsNode>(node, window_name, window_width, window_height, fullscreen);
	});
	TAPE_HOOK(node, window_name, window_width, window_height, fullscreen);
	return status;
}

void TapePlayer::tape_node_points_visualize(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_points_visualize(&node,
		yamlNode[1].as<std::string>().c_str(),
		yamlNode[2].as<int32_t>(),
		yamlNode[3].as<int32_t>(),
		yamlNode[4].as<bool>());
	tapeNodes.insert(std::make_pair(nodeId, node));
}
}
