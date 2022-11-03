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

class TapeRecord
{
	struct ToBinDataInfo
	{
		const void* data = nullptr;
		size_t elemSize;
		int elemCount;
	};

	YAML::Node yamlRoot;
	YAML::Node yamlRecording;
	size_t currentBinOffset = 0;
	FILE* fileBin;
	std::ofstream fileYaml;
	std::optional<ToBinDataInfo> valueToBin;

	void writeToBin(const void* source, size_t elemSize, size_t elemCount, FILE* file);

	static void recordRGLVersion(YAML::Node& node);

	template<typename T, typename... Args>
	void recordApiArguments(YAML::Node& node, int currentArgIdx, T currentArg, Args... remainingArgs)
	{
		node[std::to_string(currentArgIdx)] = valueToYaml(currentArg);

		if constexpr(sizeof...(remainingArgs) > 0) {
			recordApiArguments(node, ++currentArgIdx, remainingArgs...);
			return;
		}
		resolveBinQueueIfNeeded();
	}

	std::string fnPrettyToShortName(std::string prettyFn)
	{
		return prettyFn.substr(0, prettyFn.find('('));
	}

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

	int valueToYaml(rgl_log_level_t value) { return (int)value; }

	size_t valueToYaml(const rgl_vec3f* value) { addToBinQueue(value); return currentBinOffset; }
	size_t valueToYaml(const rgl_vec3i* value) { addToBinQueue(value); return currentBinOffset; }
	size_t valueToYaml(const rgl_mat3x4f* value) { addToBinQueue(value); return currentBinOffset; }
	size_t valueToYaml(const int32_t* value) { addToBinQueue(value); return currentBinOffset; }
	size_t valueToYaml(const rgl_field_t* value) { addToBinQueue(value); return currentBinOffset; }

	int32_t valueToYaml(int32_t value)
	{
		if (valueToBin.has_value())
		{
			addToBinQueue((int)value);
			resolveBinQueueIfNeeded();
		}
		return value;
	}

	template<typename T>
	void addToBinQueue(T data)
	{
		resolveBinQueueIfNeeded();
		size_t elemSize = sizeof(T);
		ToBinDataInfo a;
		a.data = reinterpret_cast<const void*>(data);
		a.elemSize = elemSize;
		a.elemCount = 1;
		valueToBin.emplace(a);
	}

	void addToBinQueue(int dataCount)
	{
		if (!valueToBin.has_value()) return;
		valueToBin.value().elemCount = dataCount;
	}

	void resolveBinQueueIfNeeded()
	{
		if (!valueToBin.has_value()) return;
		writeToBin(valueToBin.value().data, valueToBin.value().elemSize, valueToBin.value().elemCount, fileBin);
		valueToBin.reset();
	}

public:
	explicit TapeRecord(const char* path);

	~TapeRecord();

	template<typename... Args>
	void recordApiCall(std::string prettyFn, Args... args)
	{
		YAML::Node argsNode;
		if constexpr(sizeof...(args) > 0) {
			recordApiArguments(argsNode, 0, args...);
		}

		YAML::Node apiCallNode;
		apiCallNode[fnPrettyToShortName(prettyFn)] = argsNode;
		yamlRecording.push_back(apiCallNode);
	}
};

class TapePlay
{
	YAML::Node yamlRoot;
	uint8_t* fileMmap{};
	size_t mmapSize{};
	std::unordered_map<size_t, rgl_mesh_t> meshId;
	std::unordered_map<size_t, rgl_entity_t> entityId;

	void mmapInit(const char* path);

	void playMeshCreate(YAML::Node meshCreate);

	void playMeshDestroy(YAML::Node meshDestroy);

	void playMeshUpdateVertices(YAML::Node meshUpdateVertices);

	void playEntityCreate(YAML::Node entityCreate);

	void playEntityDestroy(YAML::Node entityDestroy);

	void playEntitySetPose(YAML::Node entitySetPose);

public:

	explicit TapePlay(const char* path);
};

extern std::optional<TapeRecord> tapeRecord;