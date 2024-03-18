#include <helpers/commonHelpers.hpp>
#include <rgl/api/extensions/tape.h>
#include <Logger.hpp>

#include <filesystem>
#include <fstream>

class TapedTest : public RGLTest
{
protected:
	const std::string benchmarkDataDirEnvVariableName = "RGL_TAPED_TEST_DATA_DIR";
};

TEST_F(TapedTest, compare_pcd_files)
{
#if RGL_BUILD_PCL_EXTENSION

	// It is necessary to set environment variable on benchmark data directory
	if (std::getenv(benchmarkDataDirEnvVariableName.c_str()) == nullptr) {
		const std::string msg = fmt::format(
		    "Skipping Taped Test - benchmark data directory must be provided in environment variable '{}'",
		    benchmarkDataDirEnvVariableName);
		GTEST_SKIP() << msg;
	}

	const std::string benchmarkDataDir = std::getenv(benchmarkDataDirEnvVariableName.c_str());
	const std::string testTapePath{benchmarkDataDir + "/awsim-mesh2pcd"};
	const std::string expectedOutputPath{benchmarkDataDir + "/expected-output/awsim-mesh2pcd.pcd"};
	const std::string outputPath{std::filesystem::current_path().string() + "/output.pcd"};

	ASSERT_RGL_SUCCESS(rgl_tape_play(testTapePath.c_str()));

	std::ifstream expectedOutputFile(expectedOutputPath, std::ios::binary);
	std::ifstream outputFile(outputPath, std::ios::binary);

	const std::vector<char> expectedOutput((std::istreambuf_iterator<char>(expectedOutputFile)),
	                                       std::istreambuf_iterator<char>());
	const std::vector<char> output((std::istreambuf_iterator<char>(outputFile)), std::istreambuf_iterator<char>());

	EXPECT_EQ(expectedOutput, output);

#else
	const std::string msg = fmt::format("Skipping Taped Test - RGL compiled without PCL extension.");
	GTEST_SKIP() << msg;
#endif
}
