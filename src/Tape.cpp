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

std::optional<TapeRecord> tapeRecord;

TapeRecord::TapeRecord(const char* path)
{
	std::string pathYaml = std::string(path) + YAML_EXTENSION;
	std::string pathBin = std::string(path) + BIN_EXTENSION;
	fileBin = fopen(pathBin.c_str(), "wb");
	if (nullptr == fileBin) {
		throw InvalidFilePath(fmt::format("rgl_tape_record_begin: could not open binary file: {}", pathBin));
	}
	fileYaml.open(pathYaml);
	TapeRecord::recordRGLVersion(yamlRoot);
	yamlRecording = yamlRoot["recording"];
}

TapeRecord::~TapeRecord()
{
	fclose(fileBin);
	fileYaml << yamlRoot;
	fileYaml.close();
}

void TapeRecord::recordRGLVersion(YAML::Node& node)
{
	YAML::Node rglVersion;
	rglVersion["major"] = RGL_VERSION_MAJOR;
	rglVersion["minor"] = RGL_VERSION_MINOR;
	rglVersion["patch"] = RGL_VERSION_PATCH;
	node[RGL_VERSION] = rglVersion;
}


TapePlay::TapePlay(const char* path)
{
	std::string pathYaml = std::string(path) + YAML_EXTENSION;
	std::string pathBin = std::string(path) + BIN_EXTENSION;

	mmapInit(pathBin.c_str());
	yamlRoot = YAML::LoadFile(pathYaml);
	auto yamlRecording = yamlRoot["recording"];

	if (yamlRoot[RGL_VERSION]["major"].as<int>() != RGL_VERSION_MAJOR ||
	    yamlRoot[RGL_VERSION]["minor"].as<int>() != RGL_VERSION_MINOR) {
		throw RecordError("recording version does not match rgl version");
	}

	for (YAML::iterator it = yamlRecording.begin(); it != yamlRecording.end(); ++it) {
		const YAML::Node& node = *it;
		std::string functionName = node["name"].as<std::string>();

		if (!tapeFunctions.contains(functionName)) {
			throw RecordError(fmt::format("unknown function to play: {}", functionName));
		}

		tapeFunctions[functionName](node);
	}
}

TapePlay::~TapePlay()
{
	munmap(fileMmap, mmapSize);
}

void TapePlay::mmapInit(const char* path)
{
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		throw InvalidFilePath(fmt::format("rgl_tape_play: could not open binary file: {}", path));
	}

	struct stat staticBuffer{};
	int err = fstat(fd, &staticBuffer);
	if (err < 0) {
		throw RecordError("rgl_tape_play: couldn't read bin file length");
	}

	fileMmap = (uint8_t*) mmap(nullptr, staticBuffer.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	mmapSize = staticBuffer.st_size;
	if (fileMmap == MAP_FAILED) {
		throw InvalidFilePath(fmt::format("rgl_tape_play: could not open binary file: {}", path));
	}
	close(fd);
}
