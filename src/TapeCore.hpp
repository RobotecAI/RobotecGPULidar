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

#include <Tape.hpp>

class TapeCore
{
	static void tape_get_version_info(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_get_extension_info(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_configure_logging(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_cleanup(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_mesh_create(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_mesh_destroy(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_mesh_update_vertices(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_mesh_set_texture_coords(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_texture_create(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_texture_destroy(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_entity_create(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_entity_destroy(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_entity_set_pose(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_entity_set_id(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_entity_set_intensity_texture(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_scene_set_time(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_graph_run(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_graph_destroy(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_graph_get_result_size(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_graph_get_result_data(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_graph_node_add_child(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_graph_node_remove_child(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_graph_node_set_priority(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_graph_node_get_priority(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_rays_from_mat3x4f(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_rays_set_range(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_rays_set_ring_ids(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_rays_set_time_offsets(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_rays_transform(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_points_transform(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_raytrace(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_raytrace_with_distortion(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_points_format(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_points_yield(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_points_compact(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_points_spatial_merge(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_points_temporal_merge(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_points_from_array(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_gaussian_noise_angular_ray(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_gaussian_noise_angular_hitpoint(const YAML::Node& yamlNode, TapeState& tapeState);
	static void tape_node_gaussian_noise_distance(const YAML::Node& yamlNode, TapeState& tapeState);

	// Called once in the translation unit
	static inline bool autoExtendTapeFunctions = std::invoke([]() {
		std::map<std::string, TapeFunction> tapeFunctions = {
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_get_version_info", TapeCore::tape_get_version_info),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_get_version_info", TapeCore::tape_get_version_info),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_get_extension_info", TapeCore::tape_get_extension_info),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_configure_logging", TapeCore::tape_configure_logging),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_cleanup", TapeCore::tape_cleanup),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_mesh_create", TapeCore::tape_mesh_create),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_mesh_destroy", TapeCore::tape_mesh_destroy),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_mesh_update_vertices", TapeCore::tape_mesh_update_vertices),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_mesh_set_texture_coords", TapeCore::tape_mesh_set_texture_coords),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_texture_create", TapeCore::tape_texture_create),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_texture_destroy", TapeCore::tape_texture_destroy),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_entity_create", TapeCore::tape_entity_create),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_entity_destroy", TapeCore::tape_entity_destroy),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_entity_set_pose", TapeCore::tape_entity_set_pose),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_entity_set_id", TapeCore::tape_entity_set_id),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_entity_set_intensity_texture", TapeCore::tape_entity_set_intensity_texture),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_scene_set_time", TapeCore::tape_scene_set_time),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_graph_run", TapeCore::tape_graph_run),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_graph_destroy", TapeCore::tape_graph_destroy),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_graph_get_result_size", TapeCore::tape_graph_get_result_size),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_graph_get_result_data", TapeCore::tape_graph_get_result_data),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_graph_node_add_child", TapeCore::tape_graph_node_add_child),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_graph_node_remove_child", TapeCore::tape_graph_node_remove_child),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_graph_node_set_priority", TapeCore::tape_graph_node_set_priority),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_graph_node_get_priority", TapeCore::tape_graph_node_get_priority),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_rays_from_mat3x4f", TapeCore::tape_node_rays_from_mat3x4f),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_rays_set_range", TapeCore::tape_node_rays_set_range),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_rays_set_ring_ids", TapeCore::tape_node_rays_set_ring_ids),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_rays_set_time_offsets", TapeCore::tape_node_rays_set_time_offsets),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_rays_transform", TapeCore::tape_node_rays_transform),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_points_transform", TapeCore::tape_node_points_transform),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_raytrace", TapeCore::tape_node_raytrace),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_raytrace_with_distortion", TapeCore::tape_node_raytrace_with_distortion),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_points_format", TapeCore::tape_node_points_format),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_points_yield", TapeCore::tape_node_points_yield),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_points_compact", TapeCore::tape_node_points_compact),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_points_spatial_merge", TapeCore::tape_node_points_spatial_merge),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_points_temporal_merge", TapeCore::tape_node_points_temporal_merge),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_points_from_array", TapeCore::tape_node_points_from_array),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_gaussian_noise_angular_ray", TapeCore::tape_node_gaussian_noise_angular_ray),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_gaussian_noise_angular_hitpoint",
		                               TapeCore::tape_node_gaussian_noise_angular_hitpoint),
		    MAKE_TAPE_FUNCTION_MAPPING("rgl_node_gaussian_noise_distance", TapeCore::tape_node_gaussian_noise_distance),
		};
		TapePlayer::extendTapeFunctions(tapeFunctions);
		return true;
	});
};
