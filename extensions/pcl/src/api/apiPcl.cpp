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

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <rgl/api/extensions/pcl.h>
#include <api/apiCommon.hpp>
#include <graph/NodesCore.hpp>
#include <graph/NodesPcl.hpp>
#include <tape/TapePcl.hpp>

#include <RGLExceptions.hpp>
#include <RGLFields.hpp>

extern "C" {

RGL_API rgl_status_t rgl_graph_write_pcd_file(rgl_node_t node, const char* file_path)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_graph_write_pcd_file(node={}, file={})", repr(node), file_path);
		CHECK_ARG(file_path != nullptr);
		CHECK_ARG(file_path[0] != '\0');

		auto pointCloudNode = Node::validatePtr<IPointsNode>(node);

		if (!pointCloudNode->hasField(XYZ_VEC3_F32)) {
			throw InvalidAPIObject(
			    fmt::format("Saving PCD file {} failed - requested node does not have field XYZ.", file_path));
		}

		// We are not using format node to avoid transferring huge point cloud to GPU (risk of cuda out of memory error)
		// We are formatting manually on the CPU instead.
		pointCloudNode->waitForResults();
		Array<Field<XYZ_VEC3_F32>::type>::ConstPtr xyzTyped = pointCloudNode->getFieldDataTyped<XYZ_VEC3_F32>();
		HostArray<Field<XYZ_VEC3_F32>::type>::ConstPtr xyzTypedHost = nullptr;
		if (isHost(xyzTyped->getMemoryKind())) {
			xyzTypedHost = xyzTyped->asSubclass<HostArray>();
		} else {
			auto tmp = HostPinnedArray<Field<XYZ_VEC3_F32>::type>::create();
			tmp->copyFrom(xyzTyped);
			xyzTypedHost = tmp;
		}

		auto xyzData = xyzTypedHost->getReadPtr();

		// Convert to PCL cloud
		pcl::PointCloud<pcl::PointXYZ> pclCloud;
		pclCloud.resize(pointCloudNode->getWidth(), pointCloudNode->getHeight());
		for (int i = 0; i < xyzTypedHost->getCount(); ++i) {
			pclCloud[i] = pcl::PointXYZ(xyzData[i].x(), xyzData[i].y(), xyzData[i].z());
		}
		pclCloud.is_dense = pointCloudNode->isDense();

		// Save to PCD file
		pcl::io::savePCDFileBinary(std::string(file_path), pclCloud);
	});
	TAPE_HOOK(node, file_path);
	return status;
}

void TapePcl::tape_graph_write_pcd_file(const YAML::Node& yamlNode, PlaybackState& state)
{
	rgl_graph_write_pcd_file(state.nodes.at(yamlNode[0].as<TapeAPIObjectID>()), yamlNode[1].as<std::string>().c_str());
}

RGL_API rgl_status_t rgl_node_points_downsample(rgl_node_t* node, float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_downsample(node={}, leaf=({}, {}, {}))", repr(node), leaf_size_x, leaf_size_y,
		            leaf_size_z);

		createOrUpdateNode<DownSamplePointsNode>(node, Vec3f{leaf_size_x, leaf_size_y, leaf_size_z});
	});
	TAPE_HOOK(node, leaf_size_x, leaf_size_y, leaf_size_z);
	return status;
}

void TapePcl::tape_node_points_downsample(const YAML::Node& yamlNode, PlaybackState& state)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = state.nodes.contains(nodeId) ? state.nodes.at(nodeId) : nullptr;
	rgl_node_points_downsample(&node, yamlNode[1].as<float>(), yamlNode[2].as<float>(), yamlNode[3].as<float>());
	state.nodes.insert({nodeId, node});
}

RGL_API rgl_status_t rgl_node_points_visualize(rgl_node_t* node, const char* window_name, int32_t window_width,
                                               int32_t window_height, bool fullscreen)
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

void TapePcl::tape_node_points_visualize(const YAML::Node& yamlNode, PlaybackState& state)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = state.nodes.contains(nodeId) ? state.nodes.at(nodeId) : nullptr;
	rgl_node_points_visualize(&node, yamlNode[1].as<std::string>().c_str(), yamlNode[2].as<int32_t>(),
	                          yamlNode[3].as<int32_t>(), yamlNode[4].as<bool>());
	state.nodes.insert({nodeId, node});
}

RGL_API rgl_status_t rgl_node_points_remove_ground(rgl_node_t* node)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_remove_ground(node={})", repr(node));

		createOrUpdateNode<RemoveGroundPointsNode>(node);
	});
	// TAPE_HOOK(node); // TODO: implement tape
	return status;
}
}
