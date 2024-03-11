#include <helpers/commonHelpers.hpp>
#include <rgl/api/extensions/tape.h>
#include <Logger.hpp>

#include <filesystem>
#include <fstream>

class TapedTest : public RGLTest
{};

TEST_F(TapedTest, compare_pcd_files)
{
#if RGL_BUILD_PCL_EXTENSION
	std::string testTapePath{std::filesystem::current_path().parent_path().string() + "/data/tapes/awsim-mesh2pcd"};
	std::string expectedOutputPath{std::filesystem::current_path().parent_path().string() +
	                               "/data/tapes/expected-output/awsim-mesh2pcd.pcd"};
	std::string outputPath{std::filesystem::current_path().string() + "/awsim-mesh2pcd.pcd"};

	ASSERT_RGL_SUCCESS(rgl_tape_play(testTapePath.c_str()));

	std::ifstream expectedOutputFile(expectedOutputPath, std::ios::binary);
	std::ifstream outputFile(outputPath, std::ios::binary);

	std::vector<char> expectedOutput((std::istreambuf_iterator<char>(expectedOutputFile)), std::istreambuf_iterator<char>());
	std::vector<char> output((std::istreambuf_iterator<char>(outputFile)), std::istreambuf_iterator<char>());

	EXPECT_EQ(expectedOutput, output);
#else
	GTEST_SKIP();
	RGL_WARN("RGL compiled without PCL extension. Integration test skipped.");
#endif
}
