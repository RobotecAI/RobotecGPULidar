#pragma once

#include <vector>
#include <cmath>
#include <numeric>
#include <filesystem>
#include <fstream>
#include "gtest/gtest.h"
#include <rgl/api/experimental.h>
#include <gmock/gmock-matchers.h>

#include <models.hpp>

using namespace ::testing;

#define EXPECT_RGL_SUCCESS(status) EXPECT_EQ(status, rgl_status_t::RGL_SUCCESS)
#define ASSERT_RGL_SUCCESS(status) ASSERT_EQ(status, rgl_status_t::RGL_SUCCESS)
#define EXPECT_RGL_STATUS(actual, expected, error_prefix, error_detail) \
    do                                                                  \
    {                                                                   \
        EXPECT_EQ(actual, expected);                                    \
        const char* error_string;                                       \
        rgl_get_last_error_string(&error_string);                       \
        EXPECT_THAT(error_string, HasSubstr(error_prefix));       \
        EXPECT_THAT(error_string, HasSubstr(error_detail));         \
    }                                                                   \
    while(false)

#define EXPECT_RGL_INVALID_OBJECT(status, type) EXPECT_RGL_STATUS(status, RGL_INVALID_API_OBJECT, "Object does not exist", type)
#define EXPECT_RGL_INVALID_ARGUMENT(status, error) EXPECT_RGL_STATUS(status, RGL_INVALID_ARGUMENT, "Invalid argument", error)


struct RGLAutoCleanupTest : public ::testing::Test {
protected:
	RGLAutoCleanupTest()
	{
		EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_TRACE, nullptr, true));
	}
	virtual ~RGLAutoCleanupTest() override
	{
		EXPECT_RGL_SUCCESS(rgl_cleanup());
	}
};


template <typename T>
std::vector<float> computeDistances(const T* data, int size)
{
    std::vector<float> return_data(size);
    for (int i = 0; i < size; i++) {
        return_data[i] = sqrt(data[i].value[0] * data[i].value[0] + data[i].value[1] * data[i].value[1] + data[i].value[2] * data[i].value[2]);
    }
    return return_data;
}

template <typename T>
std::vector<float> computeAngles(const T* data, int size)
{
    std::vector<float> return_data(size);
    for (int i = 0; i < size; i++) {
        return_data[i] = std::atan2(data[i].value[2], data[i].value[0]);
    }
    return return_data;
}

// static void getLidarResults(rgl_lidar_t lidar, int* hitpointCount, void* results)
// {
// 	EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(nullptr, lidar));
// 	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, hitpointCount));
// 	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));
// }

template<typename T>
std::pair<T, T> mean_and_stdev(std::vector<T> v) {
	float sum = std::accumulate(v.begin(), v.end(), 0.0);
	float mean = sum / v.size();

	std::vector<float> diff(v.size());
	std::transform(v.begin(), v.end(), diff.begin(),
	               std::bind2nd(std::minus<T>(), mean));
	float sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	float stdev = std::sqrt(sq_sum / v.size());

	return {mean, stdev};
}

template<typename T>
static std::vector<T> loadVec(std::filesystem::path path)
{
	// open the file:
	std::streampos fileSize;
	std::ifstream file(path, std::ios::binary);

	EXPECT_TRUE(file.is_open() && !file.eof());

	// get its size:
	file.seekg(0, std::ios::end);
	fileSize = file.tellg();
	file.seekg(0, std::ios::beg);

	EXPECT_TRUE(fileSize % sizeof(T) == 0);

	// read the data:
	std::vector<T> fileData(fileSize / sizeof(T));
	file.read((char*) &fileData[0], fileSize);
	return fileData;
}

static std::string readFileStr(std::filesystem::path path)
{
	std::vector<char> logFileChars = loadVec<char>(path);
	return {logFileChars.begin(), logFileChars.end()};
}


// TODO(prybicki): replace this with a proper Matrix class
static rgl_mat3x4f identity = { .value = {
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0
}};




// static rgl_lidar_t makeTrivialLidar()
// {
// 	rgl_lidar_t lidar = nullptr;
// 	EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, &identity, 1));
// 	EXPECT_THAT(lidar, NotNull());
// 	return lidar;
// }

// static rgl_lidar_t loadLidar(std::filesystem::path path)
// {
// 	rgl_lidar_t lidar = nullptr;
// 	std::vector<rgl_mat3x4f> rays = loadVec<rgl_mat3x4f>(path);
// 	EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, rays.data(), rays.size()));
// 	EXPECT_THAT(lidar, NotNull());
// 	return lidar;
// }

static rgl_mesh_t loadMesh(std::filesystem::path path)
{
	rgl_mesh_t mesh = nullptr;
	std::vector<rgl_vec3f> vs = loadVec<rgl_vec3f>(path.string() + std::string(".vertices"));
	std::vector<rgl_vec3i> is = loadVec<rgl_vec3i>(path.string() + std::string(".indices"));
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, vs.data(), vs.size(), is.data(), is.size()));
	return mesh;
}