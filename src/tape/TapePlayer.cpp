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

#include <filesystem>
#include <fstream>
#include <cassert>

#include <tape/TapePlayer.hpp>
#include <tape/tapeDefinitions.hpp>
#include <RGLExceptions.hpp>

namespace fs = std::filesystem;

TapePlayer::TapePlayer(const char* path)
{
	std::string pathYaml = fs::path(path).concat(YAML_EXTENSION).string();
	std::string pathBin = fs::path(path).concat(BIN_EXTENSION).string();

	playbackState = std::make_unique<PlaybackState>(pathBin.c_str());

	yamlRoot = YAML::LoadFile(pathYaml);

	if (yamlRoot[0][RGL_VERSION][1].as<int>() != RGL_VERSION_MAJOR ||
	    yamlRoot[0][RGL_VERSION][2].as<int>() != RGL_VERSION_MINOR) {
		throw RecordError("recording version does not match rgl version");
	}

	nextCallIdx = 0;
}

std::optional<TapePlayer::APICallIdx> TapePlayer::findFirst(std::set<std::string_view> fnNames)
{
	for (APICallIdx idx = 0; idx < yamlRoot.size(); ++idx) {
		if (fnNames.contains(yamlRoot[idx].begin()->first.as<std::string>())) {
			return idx;
		}
	}
	return std::nullopt;
}

std::optional<TapePlayer::APICallIdx> TapePlayer::findLast(std::set<std::string_view> fnNames)
{
	for (APICallIdx idx = 0; idx < yamlRoot.size(); ++idx) {
		auto rIdx = yamlRoot.size() - 1 - idx;
		if (fnNames.contains(yamlRoot[idx].begin()->first.as<std::string>())) {
			return rIdx;
		}
	}
	return std::nullopt;
}

std::vector<TapePlayer::APICallIdx> TapePlayer::findAll(std::set<std::string_view> fnNames)
{
	std::vector<APICallIdx> result;
	for (APICallIdx idx = 0; idx < yamlRoot.size(); ++idx) {
		if (fnNames.contains((yamlRoot[idx].begin()->first.as<std::string>()))) {
			result.push_back(idx);
		}
	}
	return result;
}

void TapePlayer::playThrough(APICallIdx last)
{
	assert(last < yamlRoot.size());
	for (; nextCallIdx <= last; ++nextCallIdx) {
		playThis(nextCallIdx);
	}
}

void TapePlayer::playUntil(std::optional<APICallIdx> breakpoint)
{
	assert(!breakpoint.has_value() || nextCallIdx < breakpoint.value());
	auto end = breakpoint.value_or(yamlRoot.size());
	assert(end <= yamlRoot.size());
	for (; nextCallIdx < end; ++nextCallIdx) {
		playThis(nextCallIdx);
	}
}

void TapePlayer::rewindTo(APICallIdx nextCall) { this->nextCallIdx = nextCall; }

void TapePlayer::playThis(APICallIdx idx)
{
	const YAML::Node& node = yamlRoot[idx];
	auto functionName = node.begin()->first.as<std::string>();
	if (!tapeFunctions.contains(functionName)) {
		throw RecordError(fmt::format("unknown function to play: {}", functionName));
	}
	const YAML::Node& callParams = node.begin()->second;

	// Remove timestamp from callParams
	YAML::Node callArgs;
	for (size_t i = 1; i < callParams.size(); ++i) {
		callArgs.push_back(callParams[i]);
	}
	tapeFunctions[functionName](callArgs, *playbackState);
}

#include <thread>
void TapePlayer::playRealtime()
{
	auto beginTimestamp = std::chrono::steady_clock::now();
	YAML::Node nextCall = yamlRoot[nextCallIdx];
	for (; nextCallIdx < yamlRoot.size(); ++nextCallIdx) {
		auto nextCallNs = std::chrono::nanoseconds(nextCall.begin()->second[0].as<int64_t>());
		auto elapsed = std::chrono::steady_clock::now() - beginTimestamp;
		std::this_thread::sleep_for(nextCallNs - elapsed);
		playThis(nextCallIdx);
	}
}
