#include <helpers/geometryData.hpp>
#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/textureHelpers.hpp>
#include <helpers/testPointCloud.hpp>

#include <RGLFields.hpp>

/**
 * This test instantiates a large thin wall (100x100x0.1) at Z=10 and shoots parallel rays from different sensor positions and orientations.
 * The wall is expected to be hit by all rays, and the incident angle is expected to be combination of sensor and box orientations.
 * Slight variations of the sensor and wall position are expected to have no effect on the incident angle.
 */

struct IncidentAngleTest : public RGLTestWithParam<std::tuple<Vec3f, Vec3f, Vec3f, Vec3f>>
{
	static const inline auto WALL_POS = Vec3f(0, 0, 10.0f);
	static const inline auto WALL_DIMS = Vec3f(200, 200, 0.1);
};

// clang-format off
INSTANTIATE_TEST_SUITE_P(Parametrized, IncidentAngleTest, testing::Combine(
	/* SensorPos     */ testing::Values(Vec3f(0, 0, 0), Vec3f(1, 1, 1)),
	/* WallPosOffset */ testing::Values(Vec3f(0, 0, 0), Vec3f(1, 1, 1)),
	/* SensorRotDeg  */ testing::Values(Vec3f(5, 0, 0), Vec3f(0, -15, 0), Vec3f(-25, 25, 0)),
	/* WallRotDeg    */ testing::Values(Vec3f(-10, 0, 0), Vec3f(0, 20, 0), Vec3f(30, -40, 0))
));
// clang-format on


TEST_P(IncidentAngleTest, wall)
{
	// Parameters
	auto&& [sensorPos, wallPosOffset, sensorRotDeg, wallRotDeg] = GetParam();
	Vec3f sensorRotRad = (std::numbers::pi_v<float> / 180.0f) * sensorRotDeg;
	Vec3f wallRotRad = wallRotDeg * Vec3f(std::numbers::pi_v<float> / 180.0f);

	// Expectations
	Vec3f expectedNormal = Mat3x4f::rotationRad(wallRotRad) * Vec3f{0, 0, -1};
	Vec3f rayDir = Mat3x4f::rotationRad(sensorRotRad) * Vec3f{0, 0, 1};
	float expectedIncidentAngleRad = acos(-expectedNormal.dot(rayDir));

	// Scene
	rgl_entity_t wall = makeEntity(makeCubeMesh());
	rgl_mat3x4f wallPose = Mat3x4f::TRS(WALL_POS + wallPosOffset, wallRotDeg, WALL_DIMS).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(wall, &wallPose));

	// Sensor
	std::vector<rgl_mat3x4f> rays = makeGridOfParallelRays(Vec2f(-1, -1), Vec2f(1, 1), Vec2i(10, 10));
	rgl_mat3x4f sensorPose = Mat3x4f::TRS(sensorPos, sensorRotDeg).toRGL();

	// Graph
	rgl_node_t useRaysNode = nullptr, transformRays = nullptr, raytraceNode = nullptr, yieldNode = nullptr;
	std::vector<rgl_field_t> yieldFields = {XYZ_VEC3_F32, INCIDENT_ANGLE_F32, NORMAL_VEC3_F32, IS_HIT_I32};
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformRays, &sensorPose));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, yieldFields.data(), yieldFields.size()));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, transformRays));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformRays, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldNode));

	// Run graph
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	// Check results
	TestPointCloud pc = TestPointCloud::createFromNode(yieldNode, yieldFields);
	EXPECT_EQ(pc.getPointCount(), rays.size()); // All rays are expected to hit the wall
	for (int i = 0; i < pc.getPointCount(); ++i) {
		EXPECT_TRUE(pc.getFieldValue<IS_HIT_I32>(i)); // All rays are expected to hit the wall

		// Normal as expected
		EXPECT_NEAR(pc.getFieldValue<NORMAL_VEC3_F32>(i).x(), expectedNormal.x(), EPSILON_F);
		EXPECT_NEAR(pc.getFieldValue<NORMAL_VEC3_F32>(i).y(), expectedNormal.y(), EPSILON_F);
		EXPECT_NEAR(pc.getFieldValue<NORMAL_VEC3_F32>(i).z(), expectedNormal.z(), EPSILON_F);

		// Incident angle as expected
		EXPECT_NEAR(pc.getFieldValue<INCIDENT_ANGLE_F32>(i), expectedIncidentAngleRad, 4 * EPSILON_F);
	}
}
