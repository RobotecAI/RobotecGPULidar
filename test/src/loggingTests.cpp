#include <Logger.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rgl/api/core.h>
#include <utils.hpp>
#include <rgl/api/extensions/tape.h>

using namespace ::testing;

class LoggingTests : public RGLTest{ 
protected:
	std::filesystem::path logFilePath { std::filesystem::temp_directory_path() / std::filesystem::path("loggingTests").concat(".log") };
};

TEST_F(LoggingTests, rgl_configure_logging)
{
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
}

TEST_F(LoggingTests, rgl_record_configure_logging)
{
	std::filesystem::path loggingRecordPath { std::filesystem::temp_directory_path() / std::filesystem::path("loggingRecord") };

	// Logging with Tape
    ASSERT_RGL_SUCCESS(rgl_tape_record_begin(loggingRecordPath.c_str()));
    bool isTapeRecordActive = false;
    ASSERT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
    ASSERT_TRUE(isTapeRecordActive);

    EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_DEBUG, logFilePath.c_str(), false));

    EXPECT_RGL_SUCCESS(rgl_tape_record_end());
    EXPECT_RGL_SUCCESS(rgl_tape_record_is_active(&isTapeRecordActive));
    EXPECT_FALSE(isTapeRecordActive);
    // TODO(nebraszka): As the bin file is empty, the tape fails to play
	// EXPECT_RGL_SUCCESS(rgl_tape_play(loggingRecordPath.c_str()));
}