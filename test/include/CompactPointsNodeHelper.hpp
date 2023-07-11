#pragma once

#include <RGLFields.hpp>
#include <utils.hpp>
#include <random>

constexpr int MULTIPLE_COMPACTIONS_COUNT = 15;
constexpr int RAYS_COUNT = 15;

static std::random_device randomDevice;
static auto randomSeed = randomDevice();
static std::mt19937 randomGenerator{randomSeed};

struct IsHitPoint
{
	Field<XYZ_F32>::type xyz;
	Field<IS_HIT_I32>::type isHit{};

	static std::vector<rgl_field_t> getPointFields() { return {XYZ_F32, IS_HIT_I32}; }
	bool operator==(const IsHitPoint& other) const
	{
		return xyz.x() == other.xyz.x() && xyz.y() == other.xyz.y() && xyz.z() == other.xyz.z() && isHit == other.isHit;
	}
};

struct Point
{
	Field<XYZ_F32>::type xyz;

	static std::vector<rgl_field_t> getPointFields() { return {XYZ_F32}; }
	bool operator==(const IsHitPoint& other) const
	{
		return xyz.x() == other.xyz.x() && xyz.y() == other.xyz.y() && xyz.z() == other.xyz.z();
	}
};

enum class HitPointDensity
{
	HALF_HIT = 0,
	ALL_NON_HIT,
	ALL_HIT,
	RANDOM,
	NONE
};

struct CompactPointsNodeHelper
{
	rgl_node_t usePointsNode = nullptr;
	std::vector<rgl_node_t> compactNodes;
	std::vector<rgl_node_t> pointsTransformNodes;

	std::vector<IsHitPoint> inPoints;
	std::vector<IsHitPoint> outPoints;

	CompactPointsNodeHelper()
	  : compactNodes(MULTIPLE_COMPACTIONS_COUNT, nullptr), pointsTransformNodes(MULTIPLE_COMPACTIONS_COUNT, nullptr)
	{}

	// TODO(nebraszka) In the future, it will be replaced by a more general point generator, which will also be used for other tests
	void generatePoints(int pointCount, HitPointDensity hitPointDensity, rgl_mat3x4f transform = identityTestTransform)
	{
		inPoints.reserve(pointCount);
		for (int i = 0; i < pointCount; ++i) {
			auto currentPoint = IsHitPoint{
			    .xyz = {i, i + 1, i + 2},
                  .isHit = determineIsHit(i, hitPointDensity)
            };
			currentPoint.xyz = Mat3x4f::fromRGL(transform) * currentPoint.xyz;
			inPoints.push_back(currentPoint);
		}
	}

	void getResults(rgl_node_t node)
	{
		int32_t outCount = 0, outSizeOf = 0;

		for (auto field : IsHitPoint::getPointFields()) {
			EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(node, field, &outCount, &outSizeOf));
			EXPECT_EQ(outSizeOf, getFieldSize(field));
			outPoints.resize(outCount);

			switch (field) {
				case XYZ_F32: {
					std::vector<::Field<XYZ_F32>::type> outData(outCount);
					if (outCount != 0) {
						EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(node, field, outData.data()));
						outData.resize(outCount);
						for (int i = 0; i < outCount; ++i) {
							outPoints[i].xyz = outData[i];
						}
					}
					break;
				}
				case IS_HIT_I32: {
					std::vector<::Field<IS_HIT_I32>::type> outData(outCount);
					if (outCount != 0) {
						EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(node, field, outData.data()));
						for (int i = 0; i < outCount; ++i) {
							outPoints[i].isHit = outData[i];
						}
					}
					break;
				}
			}
		}
	}

	void prepareAndRunUsePointsCompactGraph()
	{
		ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(),
		                                              IsHitPoint::getPointFields().data(),
		                                              IsHitPoint::getPointFields().size()));
		ASSERT_THAT(usePointsNode, testing::NotNull());
		ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
		ASSERT_THAT(compactNodes.at(0), testing::NotNull());
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNodes.at(0)));
		ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));
	}

private:
	static int32_t determineIsHit(int index, HitPointDensity hitPointDensity)
	{
		switch (hitPointDensity) {
			case HitPointDensity::HALF_HIT: return index % 2;
			case HitPointDensity::ALL_NON_HIT: return 0;
			case HitPointDensity::ALL_HIT: return 1;
			case HitPointDensity::RANDOM: return static_cast<int>(randomGenerator() % 2);
			default: return 0;
		}
	}
};
