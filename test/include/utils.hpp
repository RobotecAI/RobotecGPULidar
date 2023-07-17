#pragma once

#include <numeric>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include <rgl/api/core.h>
#include <graph/Node.hpp>
#include <math/Mat3x4f.hpp>
#include <RGLFields.hpp>
#include <models.hpp>

// TODO(msz-rai): Left this namespace for cleaner tests - fix namespace for rgl (Field)
// using namespace ::testing;

#define EXPECT_RGL_SUCCESS(status) EXPECT_EQ(status, rgl_status_t::RGL_SUCCESS)
#define ASSERT_RGL_SUCCESS(status) ASSERT_EQ(status, rgl_status_t::RGL_SUCCESS)
#define EXPECT_RGL_STATUS(actual, expected, ...)                   \
    do {                                                           \
        EXPECT_EQ(actual, expected);                               \
        const char* error_string;                                  \
        rgl_get_last_error_string(&error_string);                  \
        std::vector<std::string> errMsgBits = { __VA_ARGS__ };     \
        for (auto&& substr : errMsgBits) {                         \
            EXPECT_THAT(error_string, testing::HasSubstr(substr)); \
        }                                                          \
    } while (false)

#define EXPECT_RGL_INVALID_OBJECT(status, type) EXPECT_RGL_STATUS(status, RGL_INVALID_API_OBJECT, "Object does not exist", type)
#define EXPECT_RGL_INVALID_ARGUMENT(status, error) EXPECT_RGL_STATUS(status, RGL_INVALID_ARGUMENT, "Invalid argument", error)
#define EXPECT_RGL_INVALID_PIPELINE(status, error) EXPECT_RGL_STATUS(status, RGL_INVALID_PIPELINE, error)

constexpr float EPSILON_F = 1e-6f;
constexpr int maxGPUCoresTestCount = 20000;

static rgl_mat3x4f identityTestTransform = Mat3x4f::identity().toRGL();
static rgl_mat3x4f translationTestTransform = Mat3x4f::translation(2, 3, 4).toRGL();
static rgl_mat3x4f rotationTestTransform = Mat3x4f::rotation(10, 30, 45).toRGL();
static rgl_mat3x4f scalingTestTransform = Mat3x4f::scale(1, 2, 3).toRGL();
static rgl_mat3x4f complexTestTransform = Mat3x4f::TRS(Vec3f(1, 2, 3), Vec3f(10, 30, 45)).toRGL();

struct RGLTest : public ::testing::Test {
protected:
	RGLTest()
	{
		rgl_configure_logging(RGL_LOG_LEVEL_OFF, nullptr, false);
	}

    virtual ~RGLTest() override
    {
        EXPECT_RGL_SUCCESS(rgl_cleanup());
    }
};

template <typename T>
struct RGLTestWithParam : public RGLTest, public ::testing::WithParamInterface<T> { };

template <typename T>
std::vector<float> computeAngles(const T* data, int size)
{
    std::vector<float> outAngles(size);
    for (int i = 0; i < size; i++) {
        outAngles[i] = std::atan2(data[i].z(), data[i].x());
    }
    return outAngles;
}

template<typename T>
std::pair<T, T> calcMeanAndStdev(std::vector<T> v) {
	float sum = std::accumulate(v.begin(), v.end(), 0.0);
	float mean = sum / v.size();

	std::vector<float> diff(v.size());
	std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) { return x - mean; });
	float sqSum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	float stdev = std::sqrt(sqSum / v.size());

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

static rgl_mesh_t loadMesh(std::filesystem::path path)
{
	rgl_mesh_t mesh = nullptr;
	std::vector<rgl_vec3f> vs = loadVec<rgl_vec3f>(path.string() + std::string(".vertices"));
	std::vector<rgl_vec3i> is = loadVec<rgl_vec3i>(path.string() + std::string(".indices"));
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, vs.data(), vs.size(), is.data(), is.size()));
	return mesh;
}

template<typename T>
static std::vector<T>  generateStaticColorTexture(int width, int height, T value)
{
	auto texels = std::vector<T>(width * height);

	for (int i = 0; i < width * height; ++i)
	{
		texels[i] = (T)value;
	}
	return texels;
}

template<typename T>
static std::vector<T> generateCheckerboardTexture(int width, int height)
{
	// Generate a sample texture with a grid pattern 16x16.
	int xGridSize = ceil(width / 16.0f);
	int yGridSize = ceil(height / 16.0f);
	int xStep = 0;
	int yStep = 0;

	auto texels =  std::vector<T>(width * height);

	for (int i = 0; i < width; ++i)
	{
		for (int j = 0; j < height; ++j)
		{
			texels[i * height + j] = (T)yStep * 0.5f + (T)xStep * 0.5f;
			if (j % yGridSize == 0)
			{
				yStep += yGridSize;
			}
		}
		yStep = 0;
		if (i % xGridSize == 0)
		{
			xStep += xGridSize;
		}
	}

	return texels;
}