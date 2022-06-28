#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rgl/api/experimental.h>
#include <filesystem>
#include <Logger.h>
#include <fstream>
#include <Optix.hpp>
#include "rglCheck.hpp"

using namespace ::testing;

static std::string readFile(std::filesystem::path path)
{
	std::stringstream buffer;
	buffer << std::ifstream(path).rdbuf();
	return buffer.str();
}

TEST(APIUnitTests, rgl_configure_logging)
{
	std::filesystem::path logFilePath {std::filesystem::temp_directory_path() / std::filesystem::path("RGL-log.txt")};

	ASSERT_THAT(logFilePath.c_str(), NotNull());
	RGL_CHECK(rgl_configure_logging(RGL_LOG_LEVEL_INFO, logFilePath.c_str(), true));
	ASSERT_THAT(std::filesystem::exists(logFilePath), IsTrue());
	ASSERT_THAT(readFile(logFilePath), HasSubstr("Logging configured"));

	TRACE("This is RGL trace log."); // Should be not printed
	INFO("This is RGL info log.");
	WARN("This is RGL warn log.");
	ERROR("This is RGL error log.");
	CRITICAL("This is RGL critical log.");
	Logger::instance().flush();

	ASSERT_THAT(readFile(logFilePath), Not(HasSubstr("trace")));
	ASSERT_THAT(readFile(logFilePath), HasSubstr("info"));
	ASSERT_THAT(readFile(logFilePath), HasSubstr("warn"));
	ASSERT_THAT(readFile(logFilePath), HasSubstr("error"));
	ASSERT_THAT(readFile(logFilePath), HasSubstr("critical"));

	RGL_CHECK(rgl_configure_logging(RGL_LOG_LEVEL_OFF, nullptr, false));
}
