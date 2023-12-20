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

#include <optional>

#include <rgl/api/core.h>
#include <tape/PlaybackState.hpp>

// Helper macro to define tape function mapping entry
#define TAPE_CALL_MAPPING(API_CALL_STRING, TAPE_CALL)                                                                          \
	{                                                                                                                          \
		API_CALL_STRING, [](const auto& yamlNode, auto& tapeState) { TAPE_CALL(yamlNode, tapeState); }                         \
	}

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

	rgl_node_t getNodeHandle(TapeAPIObjectID key) { return playbackState->nodes.at(key); }

	template<typename T>
	T getCallArg(APICallIdx idx, int arg)
	{
		return yamlRoot[idx].begin()->second[arg + 1].as<T>(); // +1 for timestamp
	}

private:
	YAML::Node yamlRoot{};
	APICallIdx nextCallIdx{};
	std::unique_ptr<PlaybackState> playbackState;

	static inline std::map<std::string, TapeFunction> tapeFunctions = {};
};
