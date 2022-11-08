#pragma once

#include <fcntl.h>
#include <fstream>
#include <string> 
#include <optional>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_map>

#include <typeindex>
#include <typeinfo>
#include <map>

#include <spdlog/fmt/bundled/format.h>
#include <yaml-cpp/yaml.h>

#include <Logger.hpp>

#include <RGLExceptions.hpp>
#include <rgl/api/core.h>

#define BIN_EXTENSION ".bin"
#define YAML_EXTENSION ".yaml"

#define ENTITY_CREATE "rgl_entity_create"
#define ENTITY_DESTROY "rgl_entity_destroy"
#define ENTITY_SET_POSE "rgl_entity_set_pose"
#define MESH_CREATE "rgl_mesh_create"
#define MESH_DESTROY "rgl_mesh_destroy"
#define MESH_UPDATE_VERTICES "rgl_mesh_update_vertices"
#define RGL_VERSION "rgl_version"

#define TAPE_HOOK(...)                                                        \
do if (tapeRecord.has_value()) {                                              \
    tapeRecord->recordApiCall(__FUNCTION__ __VA_OPT__(,) __VA_ARGS__);        \
} while(0)

#define TAPE_ARRAY(data, count) std::make_pair(data, count)

using namespace std::placeholders;

class TapeRecord
{
	YAML::Node yamlRoot;
	YAML::Node yamlRecording;

	FILE* fileBin;
	std::ofstream fileYaml;

	size_t currentBinOffset = 0;

	static void recordRGLVersion(YAML::Node& node);

	template<typename T>
	size_t writeToBin(const T* source, size_t elemCount)
	{
		size_t elemSize = sizeof(T);
		uint8_t remainder = (elemSize * elemCount) % 16;
		uint8_t bytesToAdd = (16 - remainder) % 16;

		fwrite(source, elemSize, elemCount, fileBin);
		if (remainder != 0) {
			uint8_t zeros[16];
			fwrite(zeros, sizeof(uint8_t), bytesToAdd, fileBin);
		}

		size_t outBinOffest = currentBinOffset;
		currentBinOffset += elemSize * elemCount + bytesToAdd;
		return outBinOffest;
	}

	template<typename T, typename... Args>
	void recordApiArguments(YAML::Node& node, int currentArgIdx, T currentArg, Args... remainingArgs)
	{
		node[std::to_string(currentArgIdx)] = valueToYaml(currentArg);

		if constexpr(sizeof...(remainingArgs) > 0) {
			recordApiArguments(node, ++currentArgIdx, remainingArgs...);
			return;
		}
	}

	//// value to yaml converters
	template<typename T>
	T valueToYaml(T value) { return value; }

	intptr_t valueToYaml(rgl_node_t value) { return (intptr_t)value; }
	intptr_t valueToYaml(rgl_node_t* value) { return (intptr_t)*value; }

	intptr_t valueToYaml(rgl_entity_t value) { return (intptr_t)value; }
	intptr_t valueToYaml(rgl_entity_t* value) { return (intptr_t)*value; }

	intptr_t valueToYaml(rgl_mesh_t value) { return (intptr_t)value; }
	intptr_t valueToYaml(rgl_mesh_t* value) { return (intptr_t)*value; }

	intptr_t valueToYaml(rgl_scene_t value) { return (intptr_t)value; }
	intptr_t valueToYaml(rgl_scene_t* value) { return (intptr_t)*value; }

	intptr_t valueToYaml(void* value) { return (intptr_t)value; }

	int valueToYaml(int32_t* value) { return *value; }
	int valueToYaml(rgl_field_t value) { return (int)value; }
	int valueToYaml(rgl_log_level_t value) { return (int)value; }

	size_t valueToYaml(const rgl_mat3x4f* value) { return writeToBin(value, 1); }

	// TAPE_ARRAY
	template<typename T, typename N>
	size_t valueToYaml(std::pair<T, N> value) { return writeToBin(value.first, value.second); }

public:
	explicit TapeRecord(const char* path);

	~TapeRecord();

	template<typename... Args>
	void recordApiCall(std::string fnName, Args... args)
	{
		YAML::Node apiCallNode;
		apiCallNode["name"] = fnName;
		if constexpr(sizeof...(args) > 0) {
			recordApiArguments(apiCallNode, 0, args...);
		}
		yamlRecording.push_back(apiCallNode);
	}
};

class TapePlay
{
	YAML::Node yamlRoot;
	uint8_t* fileMmap{};
	size_t mmapSize{};

	std::unordered_map<size_t, rgl_mesh_t> tapeMeshes;
	std::unordered_map<size_t, rgl_entity_t> tapeEntities;
	std::unordered_map<size_t, rgl_node_t> tapeNodes;

	std::map<std::string, std::function<void(const YAML::Node&)>> tapeFunctions {
		{ "rgl_get_version_info", std::bind(&TapePlay::tape_get_version_info, this, _1) },
		{ "rgl_configure_logging", std::bind(&TapePlay::tape_configure_logging, this, _1) },
		{ "rgl_cleanup", std::bind(&TapePlay::tape_cleanup, this, _1) },
		{ "rgl_mesh_create", std::bind(&TapePlay::tape_mesh_create, this, _1) },
		{ "rgl_mesh_destroy", std::bind(&TapePlay::tape_mesh_destroy, this, _1) },
		{ "rgl_mesh_update_vertices", std::bind(&TapePlay::tape_mesh_update_vertices, this, _1) },
		{ "rgl_entity_create", std::bind(&TapePlay::tape_entity_create, this, _1) },
		{ "rgl_entity_destroy", std::bind(&TapePlay::tape_entity_destroy, this, _1) },
		{ "rgl_entity_set_pose", std::bind(&TapePlay::tape_entity_set_pose, this, _1) },
		{ "rgl_graph_run", std::bind(&TapePlay::tape_graph_run, this, _1) },
		{ "rgl_graph_destroy", std::bind(&TapePlay::tape_graph_destroy, this, _1) },
		{ "rgl_graph_get_result_size", std::bind(&TapePlay::tape_graph_get_result_size, this, _1) },
		{ "rgl_graph_get_result_data", std::bind(&TapePlay::tape_graph_get_result_data, this, _1) },
		{ "rgl_graph_node_set_active", std::bind(&TapePlay::tape_graph_node_set_active, this, _1) },
		{ "rgl_graph_node_add_child", std::bind(&TapePlay::tape_graph_node_add_child, this, _1) },
		{ "rgl_graph_node_remove_child", std::bind(&TapePlay::tape_graph_node_remove_child, this, _1) },
		{ "rgl_node_rays_from_mat3x4f", std::bind(&TapePlay::tape_node_rays_from_mat3x4f, this, _1) },
		{ "rgl_node_rays_set_ring_ids", std::bind(&TapePlay::tape_node_rays_set_ring_ids, this, _1) },
		{ "rgl_node_rays_transform", std::bind(&TapePlay::tape_node_rays_transform, this, _1) },
		{ "rgl_node_points_transform", std::bind(&TapePlay::tape_node_points_transform, this, _1) },
		{ "rgl_node_raytrace", std::bind(&TapePlay::tape_node_raytrace, this, _1) },
		{ "rgl_node_points_format", std::bind(&TapePlay::tape_node_points_format, this, _1) },
		{ "rgl_node_points_yield", std::bind(&TapePlay::tape_node_points_yield, this, _1) },
		{ "rgl_node_points_compact", std::bind(&TapePlay::tape_node_points_compact, this, _1) },
		{ "rgl_node_points_downsample", std::bind(&TapePlay::tape_node_points_downsample, this, _1) },
		{ "rgl_node_points_write_pcd_file", std::bind(&TapePlay::tape_node_points_write_pcd_file, this, _1) },
		{ "rgl_node_points_visualize", std::bind(&TapePlay::tape_node_points_visualize, this, _1) },
	};

	void tape_get_version_info(const YAML::Node& yamlNode);
	void tape_configure_logging(const YAML::Node& yamlNode);
	void tape_cleanup(const YAML::Node& yamlNode);
	void tape_mesh_create(const YAML::Node& yamlNode);
	void tape_mesh_destroy(const YAML::Node& yamlNode);
	void tape_mesh_update_vertices(const YAML::Node& yamlNode);
	void tape_entity_create(const YAML::Node& yamlNode);
	void tape_entity_destroy(const YAML::Node& yamlNode);
	void tape_entity_set_pose(const YAML::Node& yamlNode);
	void tape_graph_run(const YAML::Node& yamlNode);
	void tape_graph_destroy(const YAML::Node& yamlNode);
	void tape_graph_get_result_size(const YAML::Node& yamlNode);
	void tape_graph_get_result_data(const YAML::Node& yamlNode);
	void tape_graph_node_set_active(const YAML::Node& yamlNode);
	void tape_graph_node_add_child(const YAML::Node& yamlNode);
	void tape_graph_node_remove_child(const YAML::Node& yamlNode);
	void tape_node_rays_from_mat3x4f(const YAML::Node& yamlNode);
	void tape_node_rays_set_ring_ids(const YAML::Node& yamlNode);
	void tape_node_rays_transform(const YAML::Node& yamlNode);
	void tape_node_points_transform(const YAML::Node& yamlNode);
	void tape_node_raytrace(const YAML::Node& yamlNode);
	void tape_node_points_format(const YAML::Node& yamlNode);
	void tape_node_points_yield(const YAML::Node& yamlNode);
	void tape_node_points_compact(const YAML::Node& yamlNode);
	void tape_node_points_downsample(const YAML::Node& yamlNode);
	void tape_node_points_write_pcd_file(const YAML::Node& yamlNode);
	void tape_node_points_visualize(const YAML::Node& yamlNode);

	void mmapInit(const char* path);

public:

	explicit TapePlay(const char* path);
	~TapePlay();
};

extern std::optional<TapeRecord> tapeRecord;