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

#pragma once

// Hack to complete compilation on Windows. In runtime, it is never used.
#ifdef _WIN32
#include <io.h>
#define PROT_READ 1
#define MAP_PRIVATE 1
#define MAP_FAILED nullptr
static int munmap(void* addr, size_t length) { return -1; }
static void* mmap(void* start, size_t length, int prot, int flags, int fd, size_t offset) { return nullptr; }
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif // _WIN32

#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <optional>
#include <unordered_map>
#include <map>

#include <spdlog/fmt/bundled/format.h>
#include <yaml-cpp/yaml.h>

#include <Logger.hpp>
#include <RGLExceptions.hpp>
#include <rgl/api/core.h>

#define BIN_EXTENSION ".bin"
#define YAML_EXTENSION ".yaml"
#define RGL_VERSION "rgl_version"

#ifdef _WIN32
#define TAPE_HOOK(...)
#else
#define TAPE_HOOK(...)                                                                                                         \
	do                                                                                                                         \
		if (tapeRecorder.has_value()) {                                                                                        \
			tapeRecorder->recordApiCall(__func__ __VA_OPT__(, ) __VA_ARGS__);                                                  \
		}                                                                                                                      \
	while (0)
#endif // _WIN32

#define TAPE_ARRAY(data, count) std::make_pair(data, count)
#define FWRITE(source, elemSize, elemCount, file)                                                                              \
	do                                                                                                                         \
		if (fwrite(source, elemSize, elemCount, file) != elemCount) {                                                          \
			throw RecordError(fmt::format("Failed to write data to binary file"));                                             \
		}                                                                                                                      \
	while (0)

// Helper macro to define tape function mapping entry
#define TAPE_CALL_MAPPING(API_CALL_STRING, TAPE_CALL)                                                                          \
	{                                                                                                                          \
		API_CALL_STRING, [](const auto& yamlNode, auto& tapeState) { TAPE_CALL(yamlNode, tapeState); }                         \
	}

// Type used as a key in TapePlayer object registry
using TapeAPIObjectID = size_t;

class TapeRecorder
{
	YAML::Node yamlRoot;      // Represents the whole yaml file
	YAML::Node yamlRecording; // The sequence of API calls

	FILE* fileBin;
	std::ofstream fileYaml;

	size_t currentBinOffset = 0;
	std::chrono::time_point<std::chrono::steady_clock> beginTimestamp;

	static void recordRGLVersion(YAML::Node& node);

	template<typename T>
	size_t writeToBin(const T* source, size_t elemCount)
	{
		size_t elemSize = sizeof(T);
		uint8_t remainder = (elemSize * elemCount) % 16;
		uint8_t bytesToAdd = (16 - remainder) % 16;

		FWRITE(source, elemSize, elemCount, fileBin);
		if (remainder != 0) {
			uint8_t zeros[16];
			FWRITE(zeros, sizeof(uint8_t), bytesToAdd, fileBin);
		}

		size_t outBinOffset = currentBinOffset;
		currentBinOffset += elemSize * elemCount + bytesToAdd;
		return outBinOffset;
	}

	template<typename T, typename... Args>
	void recordApiArguments(YAML::Node& node, int currentArgIdx, T currentArg, Args... remainingArgs)
	{
		node[currentArgIdx] = valueToYaml(currentArg);

		if constexpr (sizeof...(remainingArgs) > 0) {
			recordApiArguments(node, ++currentArgIdx, remainingArgs...);
			return;
		}
	}

	//// value to yaml converters
	// Generic converter for non-enum types
	template<typename T>
	std::enable_if_t<!std::is_enum_v<T>, T> valueToYaml(T value)
	{
		return value;
	}

	// Generic converter for enum types
	template<typename T>
	std::enable_if_t<std::is_enum_v<T>, std::underlying_type_t<T>> valueToYaml(T value)
	{
		return static_cast<std::underlying_type_t<T>>(value);
	}

	uintptr_t valueToYaml(rgl_node_t value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_node_t* value) { return (uintptr_t) *value; }

	uintptr_t valueToYaml(rgl_entity_t value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_entity_t* value) { return (uintptr_t) *value; }

	uintptr_t valueToYaml(rgl_mesh_t value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_mesh_t* value) { return (uintptr_t) *value; }

	uintptr_t valueToYaml(rgl_texture_t value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_texture_t* value) { return (uintptr_t) *value; }

	uintptr_t valueToYaml(rgl_scene_t value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_scene_t* value) { return (uintptr_t) *value; }

	uintptr_t valueToYaml(void* value) { return (uintptr_t) value; }

	int valueToYaml(int32_t* value) { return *value; }
	size_t valueToYaml(const rgl_mat3x4f* value) { return writeToBin(value, 1); }
	size_t valueToYaml(const rgl_vec3f* value) { return writeToBin(value, 1); }

	// TAPE_ARRAY
	template<typename T, typename N>
	size_t valueToYaml(std::pair<T, N> value)
	{
		return writeToBin(value.first, value.second);
	}

	template<typename N>
	size_t valueToYaml(std::pair<const void*, N> value)
	{
		return writeToBin(static_cast<const char*>(value.first), value.second);
	}

public:
	explicit TapeRecorder(const std::filesystem::path& path);

	~TapeRecorder();

	template<typename... Args>
	void recordApiCall(std::string fnName, Args... args)
	{
		YAML::Node apiCallNode;
		apiCallNode["name"] = fnName;
		apiCallNode["timestamp"] =
		    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - beginTimestamp).count();
		if constexpr (sizeof...(args) > 0) {
			recordApiArguments(apiCallNode, 0, args...);
		}
		yamlRecording.push_back(apiCallNode);
	}
};

struct PlaybackState
{
	void clear()
	{
		meshes.clear();
		entities.clear();
		textures.clear();
		nodes.clear();
	}

	template<typename T>
	T* getPtr(const YAML::Node& offsetYamlNode)
	{
		assert(fileMmap != nullptr);
		auto offset = offsetYamlNode.as<size_t>();
		if (offset > mmapSize) {
			throw std::runtime_error(fmt::format("Tape binary offset ({}) out of range ({})", offset, mmapSize));
		}
		return reinterpret_cast<T*>(fileMmap + offset);
	}

	std::unordered_map<TapeAPIObjectID, rgl_mesh_t> meshes;
	std::unordered_map<TapeAPIObjectID, rgl_entity_t> entities;
	std::unordered_map<TapeAPIObjectID, rgl_texture_t> textures;
	std::unordered_map<TapeAPIObjectID, rgl_node_t> nodes;

private:
	uint8_t* fileMmap{nullptr};
	size_t mmapSize{0};

	friend struct TapePlayer; // To access private members
};

// Signature of tape function corresponding to the API function
using TapeFunction = std::function<void(const YAML::Node&, PlaybackState&)>;

struct TapePlayer
{
	using APICallIdx = int32_t;
	explicit TapePlayer(const char* path);

	static void extendTapeFunctions(std::map<std::string, TapeFunction> map) { tapeFunctions.insert(map.begin(), map.end()); }

	std::optional<APICallIdx> findFirst(std::set<std::string_view> fnNames);
	std::optional<APICallIdx> findLast(std::set<std::string_view> fnNames);
	std::vector<APICallIdx> findAll(std::set<std::string_view> fnNames);

	void playThis(APICallIdx idx);
	void playThrough(APICallIdx last);
	void playUntil(std::optional<APICallIdx> breakpoint = std::nullopt);
	void playRealtime();

	void rewindTo(APICallIdx nextCall = 0);

	rgl_node_t getNodeHandle(TapeAPIObjectID key) { return playbackState.nodes.at(key); }

	template<typename T>
	T getCallArg(APICallIdx idx, int arg)
	{
		return yamlRecording[idx][arg].as<T>();
	}

	~TapePlayer();

private:
	YAML::Node yamlRoot{};
	YAML::Node yamlRecording{};
	APICallIdx nextCall{};
	PlaybackState playbackState;

	static inline std::map<std::string, TapeFunction> tapeFunctions = {};

private:
	void mmapInit(const char* path);
};

extern std::optional<TapeRecorder> tapeRecorder;
