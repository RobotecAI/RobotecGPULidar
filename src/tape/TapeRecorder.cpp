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

#include <tape/TapeRecorder.hpp>
#include <tape/tapeDefinitions.hpp>
#include <rgl/api/core.h>

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
