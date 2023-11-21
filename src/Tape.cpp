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
		                                  "due to the error: {}",
		                                  pathBin, std::strerror(errno)));
	}
	fileYaml.open(pathYaml);
	if (fileYaml.fail()) {
		throw InvalidFilePath(fmt::format("rgl_tape_record_begin: could not open yaml file '{}' "
		                                  "due to the error: {}",
		                                  pathYaml, std::strerror(errno)));
	}
	TapeRecorder::recordRGLVersion(yamlRoot);
	yamlRecording = yamlRoot["recording"];
	beginTimestamp = std::chrono::steady_clock::now();
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
	std::string pathYaml = fs::path(path).concat(YAML_EXTENSION).string();
	std::string pathBin = fs::path(path).concat(BIN_EXTENSION).string();

	mmapInit(pathBin.c_str());
	yamlRoot = YAML::LoadFile(pathYaml);
	yamlRecording = yamlRoot["recording"];

	if (yamlRoot[RGL_VERSION]["major"].as<int>() != RGL_VERSION_MAJOR ||
	    yamlRoot[RGL_VERSION]["minor"].as<int>() != RGL_VERSION_MINOR) {
		throw RecordError("recording version does not match rgl version");
	}

	nextCall = 0;
}

std::optional<TapePlayer::APICallIdx> TapePlayer::findFirst(std::set<std::string_view> fnNames)
{
	for (APICallIdx idx = 0; idx < yamlRecording.size(); ++idx) {
		if (fnNames.contains(yamlRecording[idx]["name"].as<std::string>())) {
			return idx;
		}
	}
	return std::nullopt;
}

std::optional<TapePlayer::APICallIdx> TapePlayer::findLast(std::set<std::string_view> fnNames)
{
	for (APICallIdx idx = 0; idx < yamlRecording.size(); ++idx) {
		auto rIdx = yamlRecording.size() - 1 - idx;
		if (fnNames.contains(yamlRecording[rIdx]["name"].as<std::string>())) {
			return rIdx;
		}
	}
	return std::nullopt;
}

std::vector<TapePlayer::APICallIdx> TapePlayer::findAll(std::set<std::string_view> fnNames)
{
	std::vector<APICallIdx> result;
	for (APICallIdx idx = 0; idx < yamlRecording.size(); ++idx) {
		if (fnNames.contains((yamlRecording[idx]["name"].as<std::string>()))) {
			result.push_back(idx);
		}
	}
	return result;
}

void TapePlayer::playThrough(APICallIdx last)
{
	assert(last < yamlRecording.size());
	for (; nextCall <= last; ++nextCall) {
		playThis(nextCall);
	}
}

void TapePlayer::playUntil(std::optional<APICallIdx> breakpoint)
{
	assert(!breakpoint.has_value() || nextCall < breakpoint.value());
	auto end = breakpoint.value_or(yamlRecording.size());
	assert(end <= yamlRecording.size());
	for (; nextCall < end; ++nextCall) {
		playThis(nextCall);
	}
}

void TapePlayer::rewindTo(APICallIdx nextCall) { this->nextCall = nextCall; }

void TapePlayer::playThis(APICallIdx idx)
{
	const YAML::Node& node = yamlRecording[idx];
	std::string functionName = node["name"].as<std::string>();

	if (!tapeFunctions.contains(functionName)) {
		throw RecordError(fmt::format("unknown function to play: {}", functionName));
	}

	tapeFunctions[functionName](node, tapeState);
}

#include <thread>
void TapePlayer::playRealtime()
{
	auto beginTimestamp = std::chrono::steady_clock::now();
	for (; nextCall < yamlRecording.size(); ++nextCall) {
		auto nextCallNs = std::chrono::nanoseconds(yamlRecording[nextCall]["timestamp"].as<int64_t>());
		auto elapsed = std::chrono::steady_clock::now() - beginTimestamp;
		std::this_thread::sleep_for(nextCallNs - elapsed);
		playThis(nextCall);
	}
}

TapePlayer::~TapePlayer()
{
	if (munmap(tapeState.fileMmap, tapeState.mmapSize) == -1) {
		RGL_WARN("rgl_tape_play: failed to remove binary mappings due to {}", std::strerror(errno));
	}
}

void TapePlayer::mmapInit(const char* path)
{
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		throw InvalidFilePath(fmt::format("rgl_tape_play: could not open binary file: '{}' "
		                                  " due to the error: {}",
		                                  path, std::strerror(errno)));
	}

	struct stat staticBuffer
	{};
	int err = fstat(fd, &staticBuffer);
	if (err < 0) {
		throw RecordError("rgl_tape_play: couldn't read bin file length");
	}

	tapeState.fileMmap = (uint8_t*) mmap(nullptr, staticBuffer.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	tapeState.mmapSize = staticBuffer.st_size;
	if (tapeState.fileMmap == MAP_FAILED) {
		throw InvalidFilePath(fmt::format("rgl_tape_play: could not mmap binary file: {}", path));
	}
	if (close(fd)) {
		RGL_WARN("rgl_tape_play: failed to close binary file: '{}' "
		         "due to the error: {}",
		         path, std::strerror(errno));
	}
}
