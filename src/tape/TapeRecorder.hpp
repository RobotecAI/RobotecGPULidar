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

#pragma once

#include <string>
#include <optional>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include <rgl/api/core.h>
#include <RGLExceptions.hpp>
#include <Logger.hpp>

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

struct TapeRecorder
{
	explicit TapeRecorder(const std::filesystem::path& path);
	~TapeRecorder();

	/**
	 * Emits lines like: "rgl_get_version_info: {t: 61323, a: [0, 16, 2]}"
	 */
	template<typename... Args>
	void recordApiCall(std::string fnName, Args&&... args)
	{
		auto timestamp =
		    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - beginTimestamp).count();
		// clang-format off
		yamlEmitter << YAML::BeginMap << YAML::Key << fnName << YAML::Flow
		                << YAML::BeginMap
		                    << YAML::Key << "t" << YAML::Value << timestamp
		                    << YAML::Key << "a" << YAML::Value << YAML::BeginSeq << emitArgs(args...) << YAML::EndSeq
		                << YAML::EndMap
		            << YAML::EndMap;
		// clang-format on
	}

	static void recordRGLVersion();

private: // Methods
	/**
	 * Appends all arguments to the yaml file via template recursion.
	 * Returns nil-potent YAML::EMITTER_MANIP to allow chaining << operator.
	 */
	template<typename T, typename... Args>
	YAML::EMITTER_MANIP emitArgs(T&& head, Args&&... tail)
	{
		yamlEmitter << valueToYaml(head);
		return emitArgs(tail...);
	}
	YAML::EMITTER_MANIP emitArgs() { return YAML::Auto; }

	uintptr_t valueToYaml(void* value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_mesh_t value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_node_t value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_scene_t value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_node_t* value) { return (uintptr_t) *value; }
	uintptr_t valueToYaml(rgl_entity_t value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_mesh_t* value) { return (uintptr_t) *value; }
	uintptr_t valueToYaml(rgl_texture_t value) { return (uintptr_t) value; }
	uintptr_t valueToYaml(rgl_scene_t* value) { return (uintptr_t) *value; }
	uintptr_t valueToYaml(rgl_entity_t* value) { return (uintptr_t) *value; }
	uintptr_t valueToYaml(rgl_texture_t* value) { return (uintptr_t) *value; }
	size_t valueToYaml(const rgl_vec3f* value) { return writeToBin(value, 1); }
	size_t valueToYaml(const rgl_mat3x4f* value) { return writeToBin(value, 1); }

	template<typename T>
	std::enable_if_t<!std::is_enum_v<T>, T> valueToYaml(T value)
	{
		return value;
	}

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

	template<typename T>
	std::enable_if_t<std::is_enum_v<T>, std::string> valueToYaml(T value)
	{
		return std::to_string(static_cast<std::underlying_type_t<T>>(value));
	}

	int valueToYaml(int32_t* value) { return *value; }

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

private: // Fields
	std::ofstream fileYaml;
	YAML::Emitter yamlEmitter;
	FILE* fileBin;
	size_t currentBinOffset = 0;
	std::chrono::time_point<std::chrono::steady_clock> beginTimestamp;
};

extern std::optional<TapeRecorder> tapeRecorder;
