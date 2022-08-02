#include <Logger.hpp>

Logger &Logger::instance()
{
	static Logger instance;
	return instance;
}

Logger::Logger()
{
	// These constants are defined from CLI (cmake -D...)
	configure(RGL_LOG_LEVEL, RGL_LOG_FILE, RGL_LOG_STDOUT);
}

void Logger::configure(rgl_log_level_t logLevel, std::optional <std::filesystem::path> logFilePath, bool useStdout)
{
	if (logLevel != RGL_LOG_LEVEL_OFF && !logFilePath.has_value() && !useStdout) {
		throw spdlog::spdlog_ex("logging enabled but all sinks are disabled");
	}
	std::vector<spdlog::sink_ptr> sinkList;
	if (logFilePath.has_value() && !logFilePath.value().empty()) {
		auto fileSink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logFilePath.value().string(), true);
		sinkList.push_back(fileSink);
	}
	if (useStdout) {
		auto stdoutSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
		sinkList.push_back(stdoutSink);
	}

	mainLogger = std::make_shared<spdlog::logger>("RGL", sinkList.begin(), sinkList.end());
	mainLogger->set_level(spdlog::level::info); // Print logging configuration as INFO
	mainLogger->set_pattern("[%c]: %v");
	mainLogger->info("Logging configured: level={}, file={}, stdout={}",
	                 spdlog::level::to_string_view(static_cast<spdlog::level::level_enum>(logLevel)),
	                 logFilePath.has_value() ? logFilePath.value().string() : "(disabled)",
	                 useStdout);
	// https://spdlog.docsforge.com/master/3.custom-formatting/#pattern-flags
	mainLogger->set_pattern("[%T][%6i us][%l]: %v");
	mainLogger->set_level(static_cast<spdlog::level::level_enum>(logLevel));
}
