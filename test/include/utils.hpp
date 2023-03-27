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
        for (auto&& substr : { __VA_ARGS__ }) {                    \
            EXPECT_THAT(error_string, testing::HasSubstr(substr)); \
        }                                                          \
    } while (false)

#define EXPECT_RGL_INVALID_OBJECT(status, type) EXPECT_RGL_STATUS(status, RGL_INVALID_API_OBJECT, "Object does not exist", type)
#define EXPECT_RGL_INVALID_ARGUMENT(status, error) EXPECT_RGL_STATUS(status, RGL_INVALID_ARGUMENT, "Invalid argument", error)

constexpr float EPSILON_F = 1e-6f;
constexpr int maxGPUCoresTestCount = 20000;

static rgl_mat3x4f identityTestTransform = Mat3x4f::identity().toRGL();
static rgl_mat3x4f translationTestTransform = Mat3x4f::translation(1, 2, 3).toRGL();
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

struct RGLPointTestHelper {

protected:
    std::vector<rgl_field_t> pointFields = {
        XYZ_F32,
        IS_HIT_I32,
        INTENSITY_F32
    };

    struct TestPointStruct {
        Field<XYZ_F32>::type xyz;
        Field<IS_HIT_I32>::type isHit;
        Field<INTENSITY_F32>::type intensity;
    };

    rgl_node_t usePointsNode = nullptr;
    std::vector<TestPointStruct> inPoints;

    std::vector<TestPointStruct> GenerateTestPointsArray(int count, rgl_mat3x4f transform = identityTestTransform)
    {
        std::vector<TestPointStruct> points;
        for (int i = 0; i < count; ++i) {
            auto currentPoint = TestPointStruct { .xyz = { i, i + 1, i + 2 }, .isHit = i % 2, .intensity = 100 };
            currentPoint.xyz = Mat3x4f::fromRGL(transform) * currentPoint.xyz;
            points.emplace_back(currentPoint);
        }
        return points;
    }

    void CreateTestUsePointsNode(int pointsCount)
    {
        inPoints = GenerateTestPointsArray(pointsCount);

        EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
        ASSERT_THAT(usePointsNode, testing::NotNull());
    }
};

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
