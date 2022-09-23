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
#include <filesystem>
#include <optional>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <rgl/api/core.h>


template <typename... Args>
void logInfo(Args&&... args) {}

template <typename... Args>
void logWarn(Args&&... args) {}

template <typename... Args>
void logError(Args&&... args) {}

struct Logger
{
	static Logger& instance();
	void configure(rgl_log_level_t logLevel, std::optional<std::filesystem::path> logFilePath, bool useStdout);
	void flush() { mainLogger->flush(); }
	spdlog::logger& getLogger() { return *mainLogger; }
private:
	Logger();
	std::shared_ptr<spdlog::logger> mainLogger;
};

#define RGL_TRACE Logger::instance().getLogger().trace
#define RGL_DEBUG Logger::instance().getLogger().debug
#define RGL_INFO Logger::instance().getLogger().info
#define RGL_WARN Logger::instance().getLogger().warn
#define RGL_ERROR Logger::instance().getLogger().error
#define RGL_CRITICAL Logger::instance().getLogger().critical