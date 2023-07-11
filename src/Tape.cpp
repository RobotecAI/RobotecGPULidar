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

#include "Tape.hpp"

using namespace std::placeholders;
namespace fs = std::filesystem;

std::optional<TapeRecorder> tapeRecorder;

TapeRecorder::TapeRecorder(const fs::path& path)
{
	std::string pathYaml = fs::path(path).concat(YAML_EXTENSION).string();
	std::string pathBin = fs::path(path).concat(BIN_EXTENSION).string();

	fileBin = fopen(pathBin.c_str(), "wb");
	if (nullptr == fileBin) {
		throw InvalidFilePath(fmt::format("rgl_tape_record_begin: could not open binary file '{}' "
		                                  "due to the error: {}", pathBin, std::strerror(errno)));
	}
	fileYaml.open(pathYaml);
	if (fileYaml.fail()) {
		throw InvalidFilePath(fmt::format("rgl_tape_record_begin: could not open yaml file '{}' "
		                                  "due to the error: {}", pathYaml, std::strerror(errno)));
	}
	TapeRecorder::recordRGLVersion(yamlRoot);
	yamlRecording = yamlRoot["recording"];
}

TapeRecorder::~TapeRecorder()
{
	// TODO(prybicki): SIOF with Logger !!!
	fileYaml << yamlRoot;
	fileYaml.close();
	if (fileYaml.fail()) {
		RGL_WARN("rgl_tape_record_end: failed to close yaml file due to the error: {}", std::strerror(errno));
	}
	if (fclose(fileBin)) {
		RGL_WARN("rgl_tape_record_end: failed to close binary file due to the error: {}", std::strerror(errno));
	}
}

void TapeRecorder::recordRGLVersion(YAML::Node& node)
{
	YAML::Node rglVersion;
	rglVersion["major"] = RGL_VERSION_MAJOR;
	rglVersion["minor"] = RGL_VERSION_MINOR;
	rglVersion["patch"] = RGL_VERSION_PATCH;
	node[RGL_VERSION] = rglVersion;
}


TapePlayer::TapePlayer(const char* path)
{
	tapeFunctions = {
		{ "rgl_get_version_info", std::bind(&TapePlayer::tape_get_version_info, this, _1) },
		{ "rgl_get_extension_info", std::bind(&TapePlayer::tape_get_extension_info, this, _1) },
		{ "rgl_configure_logging", std::bind(&TapePlayer::tape_configure_logging, this, _1) },
		{ "rgl_cleanup", std::bind(&TapePlayer::tape_cleanup, this, _1) },
		{ "rgl_mesh_create", std::bind(&TapePlayer::tape_mesh_create, this, _1) },
		{ "rgl_mesh_destroy", std::bind(&TapePlayer::tape_mesh_destroy, this, _1) },
		{ "rgl_mesh_update_vertices", std::bind(&TapePlayer::tape_mesh_update_vertices, this, _1) },
		{ "rgl_mesh_set_texture_coords", std::bind(&TapePlayer::tape_mesh_set_texture_coords, this, _1) },
		{ "rgl_texture_create", std::bind(&TapePlayer::tape_texture_create, this, _1) },
		{ "rgl_texture_destroy", std::bind(&TapePlayer::tape_texture_destroy, this, _1) },
		{ "rgl_entity_create", std::bind(&TapePlayer::tape_entity_create, this, _1) },
		{ "rgl_entity_destroy", std::bind(&TapePlayer::tape_entity_destroy, this, _1) },
		{ "rgl_entity_set_pose", std::bind(&TapePlayer::tape_entity_set_pose, this, _1) },
		{ "rgl_entity_set_intensity_texture", std::bind(&TapePlayer::tape_entity_set_intensity_texture, this, _1) },
		{ "rgl_scene_set_time", std::bind(&TapePlayer::tape_scene_set_time, this, _1) },
		{ "rgl_graph_run", std::bind(&TapePlayer::tape_graph_run, this, _1) },
		{ "rgl_graph_destroy", std::bind(&TapePlayer::tape_graph_destroy, this, _1) },
		{ "rgl_graph_get_result_size", std::bind(&TapePlayer::tape_graph_get_result_size, this, _1) },
		{ "rgl_graph_get_result_data", std::bind(&TapePlayer::tape_graph_get_result_data, this, _1) },
		{ "rgl_graph_node_add_child", std::bind(&TapePlayer::tape_graph_node_add_child, this, _1) },
		{ "rgl_graph_node_remove_child", std::bind(&TapePlayer::tape_graph_node_remove_child, this, _1) },
		{ "rgl_node_rays_from_mat3x4f", std::bind(&TapePlayer::tape_node_rays_from_mat3x4f, this, _1) },
		{ "rgl_node_rays_set_range", std::bind(&TapePlayer::tape_node_rays_set_range, this, _1) },
		{ "rgl_node_rays_set_ring_ids", std::bind(&TapePlayer::tape_node_rays_set_ring_ids, this, _1) },
		{ "rgl_node_rays_set_time_offsets", std::bind(&TapePlayer::tape_node_rays_set_time_offsets, this, _1) },
		{ "rgl_node_rays_transform", std::bind(&TapePlayer::tape_node_rays_transform, this, _1) },
		{ "tape_node_rays_velocity_distort", std::bind(&TapePlayer::tape_node_rays_velocity_distort, this, _1) },
		{ "rgl_node_points_transform", std::bind(&TapePlayer::tape_node_points_transform, this, _1) },
		{ "rgl_node_raytrace", std::bind(&TapePlayer::tape_node_raytrace, this, _1) },
		{ "rgl_node_points_format", std::bind(&TapePlayer::tape_node_points_format, this, _1) },
		{ "rgl_node_points_yield", std::bind(&TapePlayer::tape_node_points_yield, this, _1) },
		{ "rgl_node_points_compact", std::bind(&TapePlayer::tape_node_points_compact, this, _1) },
		{ "rgl_node_points_spatial_merge", std::bind(&TapePlayer::tape_node_points_spatial_merge, this, _1) },
		{ "rgl_node_points_temporal_merge", std::bind(&TapePlayer::tape_node_points_temporal_merge, this, _1) },
		{ "rgl_node_points_from_array", std::bind(&TapePlayer::tape_node_points_from_array, this, _1) },
		{ "rgl_node_gaussian_noise_angular_ray", std::bind(&TapePlayer::tape_node_gaussian_noise_angular_ray, this, _1) },
		{ "rgl_node_gaussian_noise_angular_hitpoint", std::bind(&TapePlayer::tape_node_gaussian_noise_angular_hitpoint, this, _1) },
		{ "rgl_node_gaussian_noise_distance", std::bind(&TapePlayer::tape_node_gaussian_noise_distance, this, _1) },

		#if RGL_BUILD_PCL_EXTENSION
		{ "rgl_graph_write_pcd_file", std::bind(&TapePlayer::tape_graph_write_pcd_file, this, _1) },
		{ "rgl_node_points_downsample", std::bind(&TapePlayer::tape_node_points_downsample, this, _1) },
		{ "rgl_node_points_visualize", std::bind(&TapePlayer::tape_node_points_visualize, this, _1) },
		#endif

		#if RGL_BUILD_ROS2_EXTENSION
		{ "rgl_node_points_ros2_publish", std::bind(&TapePlayer::tape_node_points_ros2_publish, this, _1) },
		{ "rgl_node_points_ros2_publish_with_qos", std::bind(&TapePlayer::tape_node_points_ros2_publish_with_qos, this, _1) },
		#endif
	};

	std::string pathYaml = fs::path(path).concat(YAML_EXTENSION).string();
	std::string pathBin = fs::path(path).concat(BIN_EXTENSION).string();

	mmapInit(pathBin.c_str());
	yamlRoot = YAML::LoadFile(pathYaml);
	yamlRecording = yamlRoot["recording"];

	if (yamlRoot[RGL_VERSION]["major"].as<int>() != RGL_VERSION_MAJOR ||
	    yamlRoot[RGL_VERSION]["minor"].as<int>() != RGL_VERSION_MINOR) {
		throw RecordError("recording version does not match rgl version");
	}

	nextCall = yamlRecording.begin();
}

std::optional<YAML::iterator> TapePlayer::getFirstOf(std::set<std::string_view> fnNames)
{
	for (YAML::iterator it = yamlRecording.begin(); it != yamlRecording.end(); ++it) {
		const YAML::Node& node = *it;
		bool found = fnNames.contains(node["name"].as<std::string>());
		if (found) {
			return {it};
		}
	}
	return std::nullopt;
}

void TapePlayer::playUntil(std::optional<YAML::iterator> breakpoint)
{
	auto end = breakpoint.value_or(yamlRecording.end());
	for (; nextCall != end; ++nextCall) {
		playUnchecked(nextCall);
	}
}

void TapePlayer::playNext()
{
	if (nextCall == yamlRecording.end()) {
		return;
	}
	playUnchecked(nextCall++);
}

void TapePlayer::rewindTo(YAML::iterator nextCall)
{
	this->nextCall = nextCall;
}

void TapePlayer::playUnchecked(YAML::iterator call)
{
	const YAML::Node& node = *call;
	std::string functionName = node["name"].as<std::string>();

	if (!tapeFunctions.contains(functionName)) {
		throw RecordError(fmt::format("unknown function to play: {}", functionName));
	}

	tapeFunctions[functionName](node);
}

TapePlayer::~TapePlayer()
{
	if (munmap(fileMmap, mmapSize) == -1) {
		RGL_WARN("rgl_tape_play: failed to remove binary mappings due to {}", std::strerror(errno));
	}
}

void TapePlayer::mmapInit(const char* path)
{
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		throw InvalidFilePath(fmt::format("rgl_tape_play: could not open binary file: '{}' "
		                                  " due to the error: {}", path, std::strerror(errno)));
	}

	struct stat staticBuffer{};
	int err = fstat(fd, &staticBuffer);
	if (err < 0) {
		throw RecordError("rgl_tape_play: couldn't read bin file length");
	}

	fileMmap = (uint8_t*) mmap(nullptr, staticBuffer.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	mmapSize = staticBuffer.st_size;
	if (fileMmap == MAP_FAILED) {
		throw InvalidFilePath(fmt::format("rgl_tape_play: could not mmap binary file: {}", path));
	}
	if (close(fd)) {
		RGL_WARN("rgl_tape_play: failed to close binary file: '{}' "
		         "due to the error: {}", path, std::strerror(errno));
	}
}
