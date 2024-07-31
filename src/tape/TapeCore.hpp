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

class TapeCore
{
	static void tape_get_version_info(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_get_extension_info(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_configure_logging(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_cleanup(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_mesh_create(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_mesh_destroy(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_mesh_set_texture_coords(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_mesh_set_bone_weights(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_texture_create(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_texture_destroy(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_entity_create(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_entity_destroy(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_skeleton_create(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_skeleton_destroy(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_entity_set_transform(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_entity_set_id(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_entity_set_intensity_texture(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_entity_set_laser_retro(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_entity_apply_external_animation(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_entity_set_skeleton(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_scene_set_time(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_graph_run(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_graph_destroy(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_graph_get_result_size(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_graph_get_result_data(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_graph_node_add_child(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_graph_node_remove_child(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_graph_node_set_priority(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_graph_node_get_priority(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_rays_from_mat3x4f(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_rays_set_range(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_rays_set_ring_ids(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_rays_set_time_offsets(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_rays_transform(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_transform(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_raytrace(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_raytrace_configure_velocity(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_raytrace_configure_distortion(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_raytrace_configure_non_hits(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_raytrace_configure_mask(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_raytrace_configure_beam_divergence(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_raytrace_configure_default_intensity(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_format(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_yield(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_compact_by_field(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_spatial_merge(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_temporal_merge(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_from_array(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_filter_ground(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_radar_postprocess(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_radar_track_objects(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_points_radar_set_classes(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_gaussian_noise_angular_ray(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_gaussian_noise_angular_hitpoint(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_gaussian_noise_distance(const YAML::Node& yamlNode, PlaybackState& state);
	static void tape_node_multi_return_switch(const YAML::Node& yamlNode, PlaybackState& state);

	// Called once in the translation unit
	static inline bool autoExtendTapeFunctions = std::invoke([]() {
		std::map<std::string, TapeFunction> tapeFunctions = {
		    TAPE_CALL_MAPPING("rgl_get_version_info", TapeCore::tape_get_version_info),
		    TAPE_CALL_MAPPING("rgl_get_version_info", TapeCore::tape_get_version_info),
		    TAPE_CALL_MAPPING("rgl_get_extension_info", TapeCore::tape_get_extension_info),
		    TAPE_CALL_MAPPING("rgl_configure_logging", TapeCore::tape_configure_logging),
		    TAPE_CALL_MAPPING("rgl_cleanup", TapeCore::tape_cleanup),
		    TAPE_CALL_MAPPING("rgl_mesh_create", TapeCore::tape_mesh_create),
		    TAPE_CALL_MAPPING("rgl_mesh_destroy", TapeCore::tape_mesh_destroy),
		    TAPE_CALL_MAPPING("rgl_mesh_set_texture_coords", TapeCore::tape_mesh_set_texture_coords),
		    TAPE_CALL_MAPPING("rgl_mesh_set_bone_weights", TapeCore::tape_mesh_set_bone_weights),
		    TAPE_CALL_MAPPING("rgl_texture_create", TapeCore::tape_texture_create),
		    TAPE_CALL_MAPPING("rgl_texture_destroy", TapeCore::tape_texture_destroy),
		    TAPE_CALL_MAPPING("rgl_skeleton_create", TapeCore::tape_skeleton_create),
		    TAPE_CALL_MAPPING("rgl_skeleton_destroy", TapeCore::tape_skeleton_destroy),
		    TAPE_CALL_MAPPING("rgl_entity_create", TapeCore::tape_entity_create),
		    TAPE_CALL_MAPPING("rgl_entity_destroy", TapeCore::tape_entity_destroy),
		    TAPE_CALL_MAPPING("rgl_entity_set_transform", TapeCore::tape_entity_set_transform),
		    TAPE_CALL_MAPPING("rgl_entity_set_id", TapeCore::tape_entity_set_id),
		    TAPE_CALL_MAPPING("rgl_entity_set_intensity_texture", TapeCore::tape_entity_set_intensity_texture),
		    TAPE_CALL_MAPPING("rgl_entity_set_laser_retro", TapeCore::tape_entity_set_laser_retro),
		    TAPE_CALL_MAPPING("rgl_entity_apply_external_animation", TapeCore::tape_entity_apply_external_animation),
		    TAPE_CALL_MAPPING("rgl_entity_set_skeleton", TapeCore::tape_entity_set_skeleton),
		    TAPE_CALL_MAPPING("rgl_scene_set_time", TapeCore::tape_scene_set_time),
		    TAPE_CALL_MAPPING("rgl_graph_run", TapeCore::tape_graph_run),
		    TAPE_CALL_MAPPING("rgl_graph_destroy", TapeCore::tape_graph_destroy),
		    TAPE_CALL_MAPPING("rgl_graph_get_result_size", TapeCore::tape_graph_get_result_size),
		    TAPE_CALL_MAPPING("rgl_graph_get_result_data", TapeCore::tape_graph_get_result_data),
		    TAPE_CALL_MAPPING("rgl_graph_node_add_child", TapeCore::tape_graph_node_add_child),
		    TAPE_CALL_MAPPING("rgl_graph_node_remove_child", TapeCore::tape_graph_node_remove_child),
		    TAPE_CALL_MAPPING("rgl_graph_node_set_priority", TapeCore::tape_graph_node_set_priority),
		    TAPE_CALL_MAPPING("rgl_graph_node_get_priority", TapeCore::tape_graph_node_get_priority),
		    TAPE_CALL_MAPPING("rgl_node_rays_from_mat3x4f", TapeCore::tape_node_rays_from_mat3x4f),
		    TAPE_CALL_MAPPING("rgl_node_rays_set_range", TapeCore::tape_node_rays_set_range),
		    TAPE_CALL_MAPPING("rgl_node_rays_set_ring_ids", TapeCore::tape_node_rays_set_ring_ids),
		    TAPE_CALL_MAPPING("rgl_node_rays_set_time_offsets", TapeCore::tape_node_rays_set_time_offsets),
		    TAPE_CALL_MAPPING("rgl_node_rays_transform", TapeCore::tape_node_rays_transform),
		    TAPE_CALL_MAPPING("rgl_node_points_transform", TapeCore::tape_node_points_transform),
		    TAPE_CALL_MAPPING("rgl_node_raytrace", TapeCore::tape_node_raytrace),
		    TAPE_CALL_MAPPING("rgl_node_raytrace_configure_velocity", TapeCore::tape_node_raytrace_configure_velocity),
		    TAPE_CALL_MAPPING("rgl_node_raytrace_configure_distortion", TapeCore::tape_node_raytrace_configure_distortion),
		    TAPE_CALL_MAPPING("rgl_node_raytrace_configure_non_hits", TapeCore::tape_node_raytrace_configure_non_hits),
		    TAPE_CALL_MAPPING("rgl_node_raytrace_configure_mask", TapeCore::tape_node_raytrace_configure_mask),
		    TAPE_CALL_MAPPING("rgl_node_raytrace_configure_beam_divergence",
		                      TapeCore::tape_node_raytrace_configure_beam_divergence),
		    TAPE_CALL_MAPPING("rgl_node_raytrace_configure_default_intensity",
		                      TapeCore::tape_node_raytrace_configure_default_intensity),
		    TAPE_CALL_MAPPING("rgl_node_points_format", TapeCore::tape_node_points_format),
		    TAPE_CALL_MAPPING("rgl_node_points_yield", TapeCore::tape_node_points_yield),
		    TAPE_CALL_MAPPING("rgl_node_points_compact_by_field", TapeCore::tape_node_points_compact_by_field),
		    TAPE_CALL_MAPPING("rgl_node_points_spatial_merge", TapeCore::tape_node_points_spatial_merge),
		    TAPE_CALL_MAPPING("rgl_node_points_temporal_merge", TapeCore::tape_node_points_temporal_merge),
		    TAPE_CALL_MAPPING("rgl_node_points_from_array", TapeCore::tape_node_points_from_array),
		    TAPE_CALL_MAPPING("rgl_node_points_filter_ground", TapeCore::tape_node_points_filter_ground),
		    TAPE_CALL_MAPPING("rgl_node_points_radar_postprocess", TapeCore::tape_node_points_radar_postprocess),
		    TAPE_CALL_MAPPING("rgl_node_points_radar_track_objects", TapeCore::tape_node_points_radar_track_objects),
		    TAPE_CALL_MAPPING("rgl_node_points_radar_set_classes", TapeCore::tape_node_points_radar_set_classes),
		    TAPE_CALL_MAPPING("rgl_node_gaussian_noise_angular_ray", TapeCore::tape_node_gaussian_noise_angular_ray),
		    TAPE_CALL_MAPPING("rgl_node_gaussian_noise_angular_hitpoint", TapeCore::tape_node_gaussian_noise_angular_hitpoint),
		    TAPE_CALL_MAPPING("rgl_node_gaussian_noise_distance", TapeCore::tape_node_gaussian_noise_distance),
		    TAPE_CALL_MAPPING("rgl_node_multi_return_switch", TapeCore::tape_node_multi_return_switch),
		};
		TapePlayer::extendTapeFunctions(tapeFunctions);
		return true;
	});
};
