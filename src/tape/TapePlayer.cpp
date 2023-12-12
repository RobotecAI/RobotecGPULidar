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

	if (yamlRoot[RGL_VERSION][0].as<int>() != RGL_VERSION_MAJOR || yamlRoot[RGL_VERSION][1].as<int>() != RGL_VERSION_MINOR) {
		throw RecordError("recording version does not match rgl version");
	}

	nextCall = 0;
}

std::optional<TapePlayer::APICallIdx> TapePlayer::findFirst(std::set<std::string_view> fnNames)
{
	APICallIdx idx = 0;
	for (const auto& node : yamlRoot) {
		if (fnNames.contains(node.first.as<std::string>())) {
			return idx;
		}
		++idx;
	}
	return std::nullopt;
}

std::optional<TapePlayer::APICallIdx> TapePlayer::findLast(std::set<std::string_view> fnNames)
{
	auto it = yamlRoot.begin();
	for (APICallIdx idx = 0; idx < yamlRoot.size(); ++idx) {
		auto rIdx = yamlRoot.size() - 1 - idx;
		std::advance(it, rIdx);
		if (fnNames.contains(it->first.as<std::string>())) {
			return rIdx;
		}
	}
	return std::nullopt;
}

std::vector<TapePlayer::APICallIdx> TapePlayer::findAll(std::set<std::string_view> fnNames)
{
	std::vector<APICallIdx> result;
	APICallIdx idx = 0;
	for (const auto& node : yamlRoot) {
		if (fnNames.contains(node.first.as<std::string>())) {
			result.push_back(idx);
		}
		++idx;
	}
	return result;
}

void TapePlayer::playThrough(APICallIdx last)
{
	assert(last < yamlRoot.size());
	for (; nextCall <= last; ++nextCall) {
		playThis(nextCall);
	}
}

void TapePlayer::playUntil(std::optional<APICallIdx> breakpoint)
{
	assert(!breakpoint.has_value() || nextCall < breakpoint.value());
	auto end = breakpoint.value_or(yamlRoot.size());
	assert(end <= yamlRoot.size());
	for (; nextCall < end; ++nextCall) {
		playThis(nextCall);
	}
}

void TapePlayer::rewindTo(APICallIdx nextCall) { this->nextCall = nextCall; }

void TapePlayer::playThis(APICallIdx idx)
{
	auto it = yamlRoot.begin();
	std::advance(it, idx);
	auto functionName = it->first.as<std::string>();
	const YAML::Node& node = yamlRoot[functionName];
	if (!tapeFunctions.contains(functionName)) {
		throw RecordError(fmt::format("unknown function to play: {}", functionName));
	}
	tapeFunctions[functionName](node, *playbackState);
}

#include <thread>
void TapePlayer::playRealtime()
{
	auto beginTimestamp = std::chrono::steady_clock::now();
	for (const auto&& node : yamlRoot) {
		auto nextCallNs = std::chrono::nanoseconds(node.second[TIMESTAMP].as<int64_t>());
		auto elapsed = std::chrono::steady_clock::now() - beginTimestamp;
		std::this_thread::sleep_for(nextCallNs - elapsed);
		playThis(nextCall++);
	}
}
