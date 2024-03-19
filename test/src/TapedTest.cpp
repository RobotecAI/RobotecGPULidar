#include <helpers/commonHelpers.hpp>
#include <rgl/api/extensions/tape.h>
#include <Logger.hpp>

#include <filesystem>
#include <fstream>

#if RGL_BUILD_PCL_EXTENSION
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#endif

class TapedTest : public RGLTest
{
protected:
	const std::string benchmarkDataDirEnvVariableName = "RGL_TAPED_TEST_DATA_DIR";
	const float epsilon = 1e-2f;
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
	const std::string testTapePath{
	    (std::filesystem::path(benchmarkDataDir) / std::filesystem::path("/awsim-mesh2pcd")).string()};
	const std::string expectedOutputPath{
	    (std::filesystem::path(benchmarkDataDir) / std::filesystem::path("/expected-output/awsim-mesh2pcd.pcd")).string()};
	const std::string outputPath{(std::filesystem::temp_directory_path() / std::filesystem::path("output.pcd")).string()};

	ASSERT_RGL_SUCCESS(rgl_tape_play(testTapePath.c_str()));

	pcl::PointCloud<pcl::PointXYZ>::Ptr expectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile(expectedOutputPath, *expectedCloud);
	pcl::io::loadPCDFile(outputPath, *outputCloud);

	ASSERT_TRUE(expectedCloud->size() == outputCloud->size());

	for (size_t i = 0; i < expectedCloud->size(); ++i) {
		EXPECT_NEAR(expectedCloud->points[i].x, outputCloud->points[i].x, epsilon);
		EXPECT_NEAR(expectedCloud->points[i].y, outputCloud->points[i].y, epsilon);
		EXPECT_NEAR(expectedCloud->points[i].z, outputCloud->points[i].z, epsilon);
	}

#else
	const std::string msg = fmt::format("Skipping Taped Test - RGL compiled without PCL extension.");
	GTEST_SKIP() << msg;
#endif
}
