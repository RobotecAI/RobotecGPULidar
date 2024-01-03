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
#include <tape/TapeCall.hpp>

namespace fs = std::filesystem;

TapePlayer::TapePlayer(const char* path)
{
	std::string pathYaml = fs::path(path).concat(YAML_EXTENSION).string();
	std::string pathBin = fs::path(path).concat(BIN_EXTENSION).string();

	playbackState = std::make_unique<PlaybackState>(pathBin.c_str());

	yamlRoot = YAML::LoadFile(pathYaml);
	if (yamlRoot.IsNull()) {
		throw RecordError("Invalid Tape: Empty YAML file detected");
	}
	if (yamlRoot.IsMap()) {
		throw RecordError("Unsupported Tape format: Detected outdated format");
	}

	checkTapeVersion();

	nextCallIdx = 0;
}

void TapePlayer::checkTapeVersion()
{
	auto versionCallIdx = findFirst({RGL_VERSION});
	if (!versionCallIdx.has_value()) {
		throw RecordError("Unsupported Tape format: Missing version record in the Tape");
	}
	auto versionCall = getTapeCall(versionCallIdx.value()).getArgsNode();
	int versionLinear = 1'000'000 * versionCall[0].as<int>() + 1'000 * versionCall[1].as<int>() + versionCall[2].as<int>();
	int lastTapeUpdate = 1'000'000 * RGL_TAPE_FORMAT_VERSION_MAJOR + 1'000 * RGL_TAPE_FORMAT_VERSION_MINOR +
	                     RGL_TAPE_FORMAT_VERSION_PATCH;
	if (versionLinear < lastTapeUpdate) {
		throw RecordError("Unsupported Tape Format: Tape version is too old. Required version: " +
		                  std::to_string(RGL_TAPE_FORMAT_VERSION_MAJOR) + "." + std::to_string(RGL_TAPE_FORMAT_VERSION_MINOR) +
		                  "." + std::to_string(RGL_TAPE_FORMAT_VERSION_PATCH) + ".");
	}
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
	const TapeCall& call = getTapeCall(idx);
	if (!tapeFunctions.contains(call.getFnName())) {
		throw RecordError(fmt::format("unknown function to play: {}", call.getFnName()));
	}
	tapeFunctions[call.getFnName()](call.getArgsNode(), *playbackState);
}

#include <thread>
void TapePlayer::playRealtime()
{
	auto beginTimestamp = std::chrono::steady_clock::now();
	TapeCall nextCall = getTapeCall(nextCallIdx);
	for (; nextCallIdx < yamlRoot.size(); ++nextCallIdx) {
		auto nextCallNs = std::chrono::nanoseconds(nextCall.getTimestamp().asNanoseconds());
		auto elapsed = std::chrono::steady_clock::now() - beginTimestamp;
		std::this_thread::sleep_for(nextCallNs - elapsed);
		playThis(nextCallIdx);
	}
}
