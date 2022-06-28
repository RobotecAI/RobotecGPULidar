#pragma once
#include <filesystem>
#include <optional>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <rgl/api/experimental.h>


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

#define TRACE Logger::instance().getLogger().trace
#define DEBUG Logger::instance().getLogger().debug
#define INFO Logger::instance().getLogger().info
#define WARN Logger::instance().getLogger().warn
#define ERROR Logger::instance().getLogger().error
#define CRITICAL Logger::instance().getLogger().critical
