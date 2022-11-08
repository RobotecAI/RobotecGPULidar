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
		YAML::Node argsNode;
		if constexpr(sizeof...(args) > 0) {
			recordApiArguments(argsNode, 0, args...);
		}

		YAML::Node apiCallNode;
		apiCallNode[fnName] = argsNode;
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