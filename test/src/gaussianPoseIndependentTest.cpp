#include <RGLFields.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>

// The aim of this test is to verify that gaussian noise is not dependent on the ray's pose.
// The test will compare point clouds captured inside cubes.
// Cubes are spawned in different locations. Raytrace will always be performed from the same pose in relation to the cube.
// Gaussian noise is configured with zero standard deviation so there is no randomization.
// The test checks that the noise is always applied correctly in ray coordinates.
// There was a bug in the past where the point cloud transform was not updated correctly in the Gaussian noise nodes.

constexpr float MEAN = 0.1f;
constexpr float EPSILON_DIFF = 1e-4f;

const std::vector<rgl_mat3x4f> lidarRays = makeLidar3dRays(360, 180, 2, 1);
const std::vector<Mat3x4f> cubePoses = {
    Mat3x4f::identity(),
    Mat3x4f::TRS({100, 200, 300}, {43, 123, 99}),
    Mat3x4f::TRS({5, 5, 5}, {45, 45, 45}),
    Mat3x4f::TRS({123, 321, 10.5f}, {90, 0, 35.6f}),
    Mat3x4f::identity(),
};

struct RaytraceResult
{
	Field<XYZ_VEC3_F32>::type xyz;
	Field<DISTANCE_F32>::type distance;

	static std::vector<rgl_field_t> getFields() { return {XYZ_VEC3_F32, DISTANCE_F32}; }
};

struct GaussianPoseIndependentTest : public RGLTest
{
	GaussianPoseIndependentTest()
	{
		rgl_mat3x4f identity = Mat3x4f::identity().toRGL();

		// Setup scene
		cube = makeEntity(makeCubeMesh());
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube, &identity));

		// Create nodes
		EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, lidarRays.data(), lidarRays.size()));
		EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &identity));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
		EXPECT_RGL_SUCCESS(rgl_node_points_transform(&toLidarFrame, &identity));
		auto formatFields = RaytraceResult::getFields();
		EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));
	}

	void runTest()
	{
		bool isFirstRaytrace = true;

		std::vector<RaytraceResult> firstRaytraceResult;
		std::vector<RaytraceResult> lastRaytraceResult;
		firstRaytraceResult.resize(lidarRays.size());
		lastRaytraceResult.resize(lidarRays.size());

		for (auto&& cubeTf : cubePoses) {
			rgl_mat3x4f lidarTf = cubeTf.toRGL();
			rgl_mat3x4f lidarTfInv = cubeTf.inverse().toRGL();
			ASSERT_RGL_SUCCESS(rgl_entity_set_pose(cube, &lidarTf));
			ASSERT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarTf));
			ASSERT_RGL_SUCCESS(rgl_node_points_transform(&toLidarFrame, &lidarTfInv));
			ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));

			if (isFirstRaytrace) {
				ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, firstRaytraceResult.data()));
				isFirstRaytrace = false;
				continue;
			}

			ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, lastRaytraceResult.data()));
			for (int i = 0; i < lidarRays.size(); ++i) {
				ASSERT_NEAR(firstRaytraceResult[i].xyz.x(), lastRaytraceResult[i].xyz.x(), EPSILON_DIFF);
				ASSERT_NEAR(firstRaytraceResult[i].xyz.y(), lastRaytraceResult[i].xyz.y(), EPSILON_DIFF);
				ASSERT_NEAR(firstRaytraceResult[i].xyz.z(), lastRaytraceResult[i].xyz.z(), EPSILON_DIFF);
				ASSERT_NEAR(firstRaytraceResult[i].distance, lastRaytraceResult[i].distance, EPSILON_DIFF);
			}
		}
	}

	rgl_entity_t cube = nullptr;
	rgl_node_t useRays = nullptr;
	rgl_node_t lidarPose = nullptr;
	rgl_node_t raytrace = nullptr;
	rgl_node_t toLidarFrame = nullptr;
	rgl_node_t noise = nullptr;
	rgl_node_t format = nullptr;
};

TEST_F(GaussianPoseIndependentTest, GaussianNoiseDistance)
{
	ASSERT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&noise, MEAN, 0.0f, 0.0f));

	// Connect nodes into graph
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, noise));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(noise, toLidarFrame));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(toLidarFrame, format));

	runTest();
}

TEST_F(GaussianPoseIndependentTest, GaussianNoiseAngularHitpoint)
{
	ASSERT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_hitpoint(&noise, MEAN, 0.0f, RGL_AXIS_Y));

	// Connect nodes into graph
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, noise));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(noise, toLidarFrame));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(toLidarFrame, format));

	runTest();
}

TEST_F(GaussianPoseIndependentTest, GaussianNoiseAngularRay)
{
	ASSERT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&noise, MEAN, 0.0f, RGL_AXIS_Y));

	// Connect nodes into graph
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, noise));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(noise, raytrace));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, toLidarFrame));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(toLidarFrame, format));

	runTest();
}
