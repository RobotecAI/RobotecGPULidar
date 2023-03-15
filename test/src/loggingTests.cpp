#include <Logger.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <utils.hpp>
#include <rgl/api/core.h>

#define FILENAME                                                                                                               \
	(strrchr(__FILE__, std::filesystem::path::preferred_separator) ?                                                           \
	     strrchr(__FILE__, std::filesystem::path::preferred_separator) + 1 :                                                   \
	     __FILE__)

class LoggingTests : public RGLAutoSetUp
{};

TEST_F(LoggingTests, rgl_configure_logging)
{
	std::filesystem::path logFilePath{std::filesystem::temp_directory_path() / std::filesystem::path(FILENAME).concat(".log")};

	// Setup logging
	ASSERT_THAT(logFilePath.c_str(), testing::NotNull());
	EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_DEBUG, logFilePath.c_str(), false));
	ASSERT_TRUE(std::filesystem::exists(logFilePath));
	Logger::getOrCreate().flush();
	ASSERT_THAT(readFileStr(logFilePath), HasSubstr("Logging configured"));

	// Write some logs
	RGL_TRACE("This is RGL trace log."); // Should be not printed
	RGL_INFO("This is RGL info log.");
	RGL_WARN("This is RGL warn log.");
	RGL_ERROR("This is RGL error log.");
	RGL_CRITICAL("This is RGL critical log.");
	Logger::getOrCreate().flush();

	// Expected log levels should be written in the file
	std::string logFile = readFileStr(logFilePath);
	EXPECT_THAT(logFile, Not(HasSubstr("[trace]: This is RGL trace log.")));
	EXPECT_THAT(logFile, HasSubstr("[info]: This is RGL info log."));
	EXPECT_THAT(logFile, HasSubstr("[warning]: This is RGL warn log."));
	EXPECT_THAT(logFile, HasSubstr("[error]: This is RGL error log."));
	EXPECT_THAT(logFile, HasSubstr("[critical]: This is RGL critical log."));
	EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_OFF, nullptr, false));

	// Logging with Tape
	ASSERT_RGL_SUCCESS(rgl_tape_record_begin("logging_call_record"));
	bool isTapeRecordActive = false;
	ASSERT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
	ASSERT_TRUE(isTapeRecordActive);

	EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_DEBUG, logFilePath.c_str(), true));

	EXPECT_RGL_SUCCESS(rgl_tape_record_end());
	EXPECT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
	EXPECT_FALSE(isTapeRecordActive);
	EXPECT_RGL_SUCCESS(rgl_tape_play("logging_call_record"));
}