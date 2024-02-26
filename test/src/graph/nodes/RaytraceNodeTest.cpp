#include <helpers/commonHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/testPointCloud.hpp>
#include <helpers/lidarHelpers.hpp>

#include <ranges>
#include <cmath>

#include <math/Mat3x4f.hpp>

class RaytraceNodeTest : public RGLTest
{
protected:
	rgl_node_t raytraceNode;

	RaytraceNodeTest() { raytraceNode = nullptr; }
};

TEST_F(RaytraceNodeTest, invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(nullptr, nullptr), "node != nullptr");
}

TEST_F(RaytraceNodeTest, valid_arguments)
{
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	ASSERT_THAT(raytraceNode, testing::NotNull());

	// If (*raytraceNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
}

// Configuration tests
TEST_F(RaytraceNodeTest, config_velocity_invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_velocity(raytraceNode, nullptr, nullptr), "node != nullptr");
}

TEST_F(RaytraceNodeTest, config_velocity_invalid_argument_linear_velocity)
{
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_velocity(raytraceNode, nullptr, nullptr),
	                            "linear_velocity != nullptr");
}

TEST_F(RaytraceNodeTest, config_velocity_invalid_argument_angular_velocity)
{
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	const rgl_vec3f linearVelocity = {1.0f, 2.0f, 3.0f};
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_velocity(raytraceNode, &linearVelocity, nullptr),
	                            "angular_velocity != nullptr");
}

TEST_F(RaytraceNodeTest, config_velocity_invalid_node_object)
{
	const rgl_vec3f linearVelocity = {1.0f, 2.0f, 3.0f};
	const rgl_vec3f angularVelocity = {4.0f, 5.0f, 6.0f};
	EXPECT_RGL_INVALID_OBJECT(rgl_node_raytrace_configure_velocity((rgl_node_t) 0x1234, &linearVelocity, &angularVelocity),
	                          "Object does not exist: Node 0x1234");
}

TEST_F(RaytraceNodeTest, config_velocity_valid_arguments)
{
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	const rgl_vec3f linearVelocity = {1.0f, 2.0f, 3.0f};
	const rgl_vec3f angularVelocity = {4.0f, 5.0f, 6.0f};
	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_velocity(raytraceNode, &linearVelocity, &angularVelocity));
}

TEST_F(RaytraceNodeTest, config_distortion_invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_distortion(raytraceNode, false), "node != nullptr");
}

TEST_F(RaytraceNodeTest, config_distortion_invalid_node_object)
{
	EXPECT_RGL_INVALID_OBJECT(rgl_node_raytrace_configure_distortion((rgl_node_t) 0x1234, false),
	                          "Object does not exist: Node 0x1234");
}

TEST_F(RaytraceNodeTest, config_distortion_valid_arguments)
{
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_distortion(raytraceNode, true));
}

TEST_F(RaytraceNodeTest, config_non_hit_distance_invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_non_hit_distance_values(raytraceNode, 0.0f, 0.0f),
	                            "node != nullptr");
}

TEST_F(RaytraceNodeTest, config_non_hit_distance_invalid_node_object)
{
	EXPECT_RGL_INVALID_OBJECT(rgl_node_raytrace_configure_non_hit_distance_values((rgl_node_t) 0x1234, 0.0f, 0.0f),
	                          "Object does not exist: Node 0x1234");
}

TEST_F(RaytraceNodeTest, config_non_hit_distance_valid_arguments)
{
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_non_hit_distance_values(raytraceNode, 0.0f, 0.0f));
}

TEST_F(RaytraceNodeTest, config_non_hit_near_distance_should_correctly_change_output)
{
	/**
	 * Test spawns six cubes. They are arranged around the lidar's origin (0,0,0).
	 * The cubes are placed in such a way that they completely surround the lidar without leaving any gaps.
	 *
	 * The centers of the cubes are positioned at:
	 * 1. (2, 0, 0),
	 * 2. (-2, 0, 0),
     * 3. (0, 2, 0),
     * 4. (0, -2, 0),
	 * 5. (0, 0, 2),
	 * 6. (0, 0, -2)
	 */
	float cubeCenterDistance = 2.0f;

	const std::vector<Vec3f> cubePositions = {Vec3f(cubeCenterDistance, 0.0f, 0.0f), Vec3f(-cubeCenterDistance, 0.0f, 0.0f),
	                                          Vec3f(0.0f, cubeCenterDistance, 0.0f), Vec3f(0.0f, -cubeCenterDistance, 0.0f),
	                                          Vec3f(0.0f, 0.0f, cubeCenterDistance), Vec3f(0.0f, 0.0f, -cubeCenterDistance)};

	for (const auto& position : cubePositions) {
		spawnCubeOnScene(Mat3x4f::translation(position.x(), position.y(), position.z()));
	}

	float fovX = 360.0f;
	float fovY = 180.0f;
	float resolution = 30.0f;
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(fovX, fovY, resolution, resolution);
	float nearNonHitDistance = 12.34f;

	std::vector<rgl_vec2f> ranges{
	    rgl_vec2f{cubeCenterDistance, cubeCenterDistance + CUBE_HALF_EDGE}
    };
	std::vector<rgl_field_t> outFields{XYZ_VEC3_F32, IS_HIT_I32, DISTANCE_F32};

	rgl_node_t raysNode = nullptr;
	rgl_node_t rangeRaysNode = nullptr;
	rgl_node_t yieldNode = nullptr;

	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysNode, rays.data(), rays.size()));
	ASSERT_RGL_SUCCESS(rgl_node_rays_set_range(&rangeRaysNode, ranges.data(), ranges.size()));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace_configure_non_hit_distance_values(raytraceNode, nearNonHitDistance, 0.0f));
	ASSERT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, outFields.data(), outFields.size()));

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raysNode, rangeRaysNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rangeRaysNode, raytraceNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldNode));

	ASSERT_RGL_SUCCESS(rgl_graph_run(raysNode));

	TestPointCloud outPointCloud = TestPointCloud::createFromNode(yieldNode, outFields);

	std::vector<Field<XYZ_VEC3_F32>::type> outCoords = outPointCloud.getFieldValues<XYZ_VEC3_F32>();
	std::vector<Field<DISTANCE_F32>::type> outDistances = outPointCloud.getFieldValues<DISTANCE_F32>();
	std::vector<Field<IS_HIT_I32>::type> outIsHits = outPointCloud.getFieldValues<IS_HIT_I32>();

	std::vector<Vec3f> expectedCoords;
	for (const auto& ray : rays) {
		Vec3f origin = Mat3x4f::fromRGL(ray) * Vec3f{0, 0, 0};
		Vec3f dir = Mat3x4f::fromRGL(ray) * Vec3f{0, 0, 1} - origin;
		Vec3f expectedPoint = origin + dir.normalized() * nearNonHitDistance;
		expectedCoords.emplace_back(expectedPoint);
	}

	for (size_t i = 0; i < outCoords.size(); ++i) {
		EXPECT_NEAR(outCoords.at(i).x(), expectedCoords.at(i).x(), EPSILON_F);
		EXPECT_NEAR(outCoords.at(i).y(), expectedCoords.at(i).y(), EPSILON_F);
		EXPECT_NEAR(outCoords.at(i).z(), expectedCoords.at(i).z(), EPSILON_F);
		EXPECT_EQ(outDistances.at(i), nearNonHitDistance);
		EXPECT_EQ(outIsHits.at(i), 0);
	}
}

TEST_F(RaytraceNodeTest, config_non_hit_far_distance_should_correctly_change_output)
{
	float fovX = 360.0f;
	float fovY = 180.0f;
	float resolution = 30.0f;
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(fovX, fovY, resolution, resolution);
	float farNonHitDistance = 13.24f;

	std::vector<rgl_field_t> outFields{XYZ_VEC3_F32, IS_HIT_I32, DISTANCE_F32};

	rgl_node_t raysNode = nullptr;
	rgl_node_t yieldNode = nullptr;

	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysNode, rays.data(), rays.size()));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace_configure_non_hit_distance_values(raytraceNode, 0.0f, farNonHitDistance));
	ASSERT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, outFields.data(), outFields.size()));

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raysNode, raytraceNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldNode));

	ASSERT_RGL_SUCCESS(rgl_graph_run(raysNode));

	TestPointCloud outPointCloud = TestPointCloud::createFromNode(yieldNode, outFields);

	std::vector<Field<XYZ_VEC3_F32>::type> outCoords = outPointCloud.getFieldValues<XYZ_VEC3_F32>();
	std::vector<Field<DISTANCE_F32>::type> outDistances = outPointCloud.getFieldValues<DISTANCE_F32>();
	std::vector<Field<IS_HIT_I32>::type> outIsHits = outPointCloud.getFieldValues<IS_HIT_I32>();

	std::vector<Vec3f> expectedCoords;
	for (const auto& ray : rays) {
		Vec3f origin = Mat3x4f::fromRGL(ray) * Vec3f{0, 0, 0};
		Vec3f dir = Mat3x4f::fromRGL(ray) * Vec3f{0, 0, 1} - origin;
		Vec3f expectedPoint = origin + dir.normalized() * farNonHitDistance;
		expectedCoords.emplace_back(expectedPoint);
	}

	for (size_t i = 0; i < outCoords.size(); ++i) {
		EXPECT_NEAR(outCoords.at(i).x(), expectedCoords.at(i).x(), EPSILON_F);
		EXPECT_NEAR(outCoords.at(i).y(), expectedCoords.at(i).y(), EPSILON_F);
		EXPECT_NEAR(outCoords.at(i).z(), expectedCoords.at(i).z(), EPSILON_F);
		EXPECT_EQ(outDistances.at(i), farNonHitDistance);
		EXPECT_EQ(outIsHits.at(i), 0);
	}
}
