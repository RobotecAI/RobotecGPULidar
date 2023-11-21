#include <helpers/commonHelpers.hpp>
#include <helpers/fileHelpers.hpp>

#include <Logger.hpp>
#include <rgl/api/extensions/tape.h>

using namespace ::testing;

class GeneralCallsTest : public RGLTest
{
protected:
	std::string logFilePath{
	    (std::filesystem::temp_directory_path() / std::filesystem::path("loggingTests").concat(".log")).string()};
};

TEST_F(GeneralCallsTest, rgl_get_version_info)
{
	int major, minor, patch;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(nullptr, nullptr, nullptr), "out_major != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major, nullptr, nullptr), "out_minor != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major, &minor, nullptr), "out_patch != nullptr");

	// Valid args
	ASSERT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));
	EXPECT_EQ(major, RGL_VERSION_MAJOR);
	EXPECT_EQ(minor, RGL_VERSION_MINOR);
	EXPECT_EQ(patch, RGL_VERSION_PATCH);
}

TEST_F(GeneralCallsTest, rgl_configure_logging)
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