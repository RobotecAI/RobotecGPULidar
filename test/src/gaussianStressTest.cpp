#include <helpers/mathHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/commonHelpers.hpp>

#include <RGLFields.hpp>
#include <math/Mat3x4f.hpp>

using namespace ::testing;

constexpr int GRAPH_EPOCHS = 256;
constexpr int LIDAR_RAYS_COUNT = 100000;
constexpr float EPSILON_MUL = 2 * 1E-6;
constexpr float EPSILON_NOISE = 0.002; 
constexpr float MEAN = 0.1;
constexpr float STD_DEV = 0.01;
constexpr float STD_DEV_PER_METER = 0.001;
constexpr rgl_axis_t ANGULAR_AXIS = RGL_AXIS_Y;

const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, 5).toRGL();
const rgl_mat3x4f lidarPoseTf = Mat3x4f::translation(0, 0, 1).toRGL();
const rgl_mat3x4f lidarPoseInvTf = Mat3x4f::fromRGL(lidarPoseTf).inverse().toRGL();
const rgl_mat3x4f lidarRayTf = Mat3x4f::identity().toRGL();
const std::vector<rgl_mat3x4f> lidarRays(LIDAR_RAYS_COUNT, lidarRayTf);
const Vec3f noiselessHitpointInLidarFrame = {0.0f, 0.0f, 3.0f};

struct GaussianStressTest : public RGLTest {
	GaussianStressTest()
	{
		// Setup scene
		rgl_entity_t cube = makeEntity(makeCubeMesh());
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube, &cubePoseTf));

		// Create nodes
		EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, lidarRays.data(), lidarRays.size()));
		EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
		EXPECT_RGL_SUCCESS(rgl_node_points_transform(&toLidarFrame, &lidarPoseInvTf));
		EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yield, yieldFields.data(), yieldFields.size()));

		outPoints.resize(LIDAR_RAYS_COUNT);
		outDistances.resize(LIDAR_RAYS_COUNT);
	}

	void runAndFetchData()
	{
		EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yield, XYZ_F32, outPoints.data()));
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yield, DISTANCE_F32, outDistances.data()));
	}

	void testDistance()
	{
		// Distance calculated from XYZ should be equal to DISTANCE field
		for (int pointIdx = 0; pointIdx < LIDAR_RAYS_COUNT; ++pointIdx) {
			EXPECT_THAT(outPoints.at(pointIdx).length(), FloatNear(outDistances.at(pointIdx), EPSILON_MUL));
		}
	}

	rgl_node_t useRays = nullptr;
	rgl_node_t lidarPose = nullptr;
	rgl_node_t raytrace = nullptr;
	rgl_node_t toLidarFrame = nullptr;
	rgl_node_t yield = nullptr;
	rgl_node_t noise = nullptr;

	std::vector<rgl_field_t> yieldFields = { XYZ_F32, DISTANCE_F32 };

	std::vector<::Field<XYZ_F32>::type> outPoints;
	std::vector<::Field<DISTANCE_F32>::type> outDistances;
};

TEST_F(GaussianStressTest, GaussianNoiseDistance)
{
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&noise, MEAN, STD_DEV, STD_DEV_PER_METER));

	// Connect nodes into graph
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, noise));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(noise, toLidarFrame));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(toLidarFrame, yield));

	for (int i = 0; i < GRAPH_EPOCHS; ++i) {
		runAndFetchData();
		testDistance();

		auto [realMean, realStdev] = calcMeanAndStdev(outDistances);
		float expectedMean = noiselessHitpointInLidarFrame.z() + MEAN;
		float expectedStdev = noiselessHitpointInLidarFrame.z() * STD_DEV_PER_METER + STD_DEV;

		EXPECT_THAT(expectedMean, FloatNear(realMean, EPSILON_NOISE));
		EXPECT_THAT(expectedStdev, FloatNear(realStdev, EPSILON_NOISE));
	}
}

TEST_F(GaussianStressTest, GaussianNoiseAngularRay)
{
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&noise, MEAN, STD_DEV, ANGULAR_AXIS));

	// Connect nodes into graph
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, noise));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(noise, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, toLidarFrame));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(toLidarFrame, yield));

	for (int i = 0; i < GRAPH_EPOCHS; ++i) {
		runAndFetchData();
		testDistance();

		auto outAngles = computeAngles(outPoints.data(), outPoints.size());
		auto [realMean, realStdev] = calcMeanAndStdev(outAngles);
		auto expectedHitpoint = (Mat3x4f::rotationRad(ANGULAR_AXIS, MEAN) * Mat3x4f::translation(noiselessHitpointInLidarFrame)).translation();
		float expectedMean = computeAngles(&expectedHitpoint, 1)[0];

		EXPECT_THAT(expectedMean, FloatNear(realMean, EPSILON_NOISE));
		EXPECT_THAT(STD_DEV, FloatNear(realStdev, EPSILON_NOISE));
	}
}

TEST_F(GaussianStressTest, GaussianNoiseAngularHitpoint)
{
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_hitpoint(&noise, MEAN, STD_DEV, ANGULAR_AXIS));

	// Connect nodes into graph
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, noise));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(noise, toLidarFrame));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(toLidarFrame, yield));

	for (int i = 0; i < GRAPH_EPOCHS; ++i) {
		runAndFetchData();
		testDistance();

		auto outAngles = computeAngles(outPoints.data(), outPoints.size());
		auto [realMean, realStdev] = calcMeanAndStdev(outAngles);
		// Not exactly expected hitpoint, but angle is the same
		auto expectedHitpoint = (Mat3x4f::rotationRad(ANGULAR_AXIS, MEAN) * Mat3x4f::translation(noiselessHitpointInLidarFrame)).translation();
		float expectedMean = computeAngles(&expectedHitpoint, 1)[0];

		EXPECT_THAT(expectedMean, FloatNear(realMean, EPSILON_NOISE));
		EXPECT_THAT(STD_DEV, FloatNear(realStdev, EPSILON_NOISE));
	}
}
