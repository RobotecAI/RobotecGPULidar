#include <helpers/geometryData.hpp>
#include <helpers/mathHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/testPointCloud.hpp>
#include <helpers/commonHelpers.hpp>

#include <RGLFields.hpp>
#include <math/Mat3x4f.hpp>

#include <random>

static constexpr float RAYTRACE_DEPTH = 1000;
static constexpr int ANGLE_ITERATIONS = 100;
static constexpr int LIDAR_RAYS_COUNT = 100000;
static constexpr float STD_DEV = 0.01;
static constexpr float STD_DEV_PER_METER = 0.001;
static constexpr float MEAN = 0.1;
static constexpr float EPSILON_MUL = 2 * 1E-4;
static constexpr float EPSILON_NOISE = 0.002;

class DistanceFieldTest : public RGLTestWithParam<float>
{
protected:
	rgl_node_t useRaysNode = nullptr;
	rgl_node_t raytraceNode = nullptr;
	rgl_node_t gaussianNoiseNode = nullptr;
	rgl_node_t yieldPointsNode = nullptr;
	rgl_node_t compactPointsNode = nullptr;
	rgl_node_t transformPointsNode = nullptr;

	std::vector<rgl_mat3x4f> rayTf;

	std::vector<rgl_field_t> fields = {DISTANCE_F32, XYZ_F32};
	std::vector<::Field<DISTANCE_F32>::type> outDistances;
	std::vector<::Field<XYZ_F32>::type> outPoints;

	std::unique_ptr<TestPointCloud> pointCloud;

	void connectNodes(bool withGaussianNoiseNode)
	{
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactPointsNode));
		if (withGaussianNoiseNode) {
			ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactPointsNode, gaussianNoiseNode));
			ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(gaussianNoiseNode, yieldPointsNode));
		} else {
			ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactPointsNode, yieldPointsNode));
		}
	}

	void disconnectNodes(bool withGaussianNoiseNode)
	{
		ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(useRaysNode, raytraceNode));
		ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(raytraceNode, compactPointsNode));
		if (withGaussianNoiseNode) {
			ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(compactPointsNode, gaussianNoiseNode));
			ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(gaussianNoiseNode, yieldPointsNode));
		} else {
			ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(compactPointsNode, yieldPointsNode));
		}
	}

	void prepareNodes()
	{
		ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
		ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));
		ASSERT_RGL_SUCCESS(rgl_node_points_yield(&yieldPointsNode, fields.data(), fields.size()));
		ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactPointsNode));
	}

	void getResults(rgl_node_t node = nullptr)
	{
		ASSERT_RGL_SUCCESS(rgl_graph_run(useRaysNode));

		if (node == nullptr) {
			node = yieldPointsNode;
		}
		pointCloud = std::make_unique<TestPointCloud>(TestPointCloud::createFromNode(node, fields));
		ASSERT_EQ(pointCloud->getPointCount(), rayTf.size());
		outDistances = pointCloud->getFieldValues<DISTANCE_F32>();
		outPoints = pointCloud->getFieldValues<XYZ_F32>();
	}
};

INSTANTIATE_TEST_CASE_P(DistanceFieldTests, DistanceFieldTest, ::testing::Values(23.1753849f, 119.9884538f, 333.3394629f));

TEST_P(DistanceFieldTest, should_compute_correct_distance_for_single_object_on_ray_path)
{
	float cubeZDistance = GetParam();
	const Mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance);
	spawnCubeOnScene(nullptr, cubePoseTf);

	rayTf.emplace_back(Mat3x4f::identity().toRGL());

	prepareNodes();
	connectNodes(false);

	getResults();

	EXPECT_EQ(outDistances.at(0), outPoints.at(0).length());
	EXPECT_EQ(outDistances.at(0), cubeZDistance - CUBE_HALF_EDGE);
}

TEST_F(DistanceFieldTest, should_compute_correct_distance_when_ray_origin_on_cube_face)
{
	float cubeZDistance = 1.0;
	const Mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance);
	spawnCubeOnScene(nullptr, cubePoseTf);

	rayTf.emplace_back(Mat3x4f::identity().toRGL());

	prepareNodes();
	connectNodes(false);

	getResults();

	EXPECT_EQ(outDistances.at(0), outPoints.at(0).length());
	EXPECT_EQ(outDistances.at(0), 2 * CUBE_HALF_EDGE);
}

TEST_F(DistanceFieldTest, should_compute_correct_distance_when_ray_origin_inside_cube)
{
	float cubeZDistance = -0.001f;
	const Mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance);
	spawnCubeOnScene(nullptr, cubePoseTf);

	rayTf.emplace_back(Mat3x4f::identity().toRGL());

	prepareNodes();
	connectNodes(false);

	getResults();

	EXPECT_EQ(outDistances.at(0), outPoints.at(0).length());
	EXPECT_EQ(outDistances.at(0), CUBE_HALF_EDGE + cubeZDistance);
}

TEST_F(DistanceFieldTest, should_compute_infinite_distance_for_object_off_ray_path)
{
	float cubeYAxisTranslation = 55.5f;
	const Mat3x4f cubePoseTf = Mat3x4f::translation(0, cubeYAxisTranslation, 0);
	spawnCubeOnScene(nullptr, cubePoseTf);

	rayTf.emplace_back(Mat3x4f::identity().toRGL());
	// After sending a ray in the direction of negative infinity, the distance should still be positive.
	rayTf.emplace_back(Mat3x4f::rotationRad(RGL_AXIS_X, M_PI).toRGL());

	prepareNodes();

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldPointsNode));

	getResults();

	EXPECT_EQ(outDistances.at(0), NON_HIT_VALUE);
	EXPECT_EQ(outDistances.at(1), NON_HIT_VALUE);
}

TEST_P(DistanceFieldTest, should_compute_correct_distances_for_various_ray_angles)
{
	float cubeZDistance = GetParam();
	const Mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance);
	spawnCubeOnScene(nullptr, cubePoseTf);

	float maxAngle = atan(CUBE_HALF_EDGE / (cubeZDistance - CUBE_HALF_EDGE));

	// Generate angle and print the seed for reproducibility
	std::random_device rd;
	auto seed = rd();
	std::mt19937 gen(seed);
	std::uniform_real_distribution<> dis(-maxAngle, maxAngle);
	fmt::print(stderr, "DistanceFieldTest random seed: {}\n", seed);

	std::vector<float> expectedDistances;

	for (int i = 0; i < ANGLE_ITERATIONS; ++i) {
		float randomAngle = dis(gen);
		rayTf.emplace_back(Mat3x4f::rotationRad(RGL_AXIS_X, randomAngle).toRGL());

		float expectedDistance = (cubeZDistance - CUBE_HALF_EDGE) / (cos(randomAngle));
		expectedDistances.emplace_back(expectedDistance);
	}

	prepareNodes();
	connectNodes(false);

	getResults();

	for (int i = 0; i < pointCloud->getPointCount(); ++i) {
		EXPECT_THAT(outPoints.at(i).length(), testing::FloatNear(outDistances.at(i), EPSILON_MUL));
		EXPECT_THAT(expectedDistances.at(i), testing::FloatNear(outDistances.at(i), EPSILON_MUL));
	}
}

TEST_F(DistanceFieldTest, should_compute_distance_from_ray_beginning)
{
	float cubeZDistance = 10.0f;
	const Mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance);
	spawnCubeOnScene(nullptr, cubePoseTf);

	rayTf.emplace_back(Mat3x4f::identity().toRGL());

	prepareNodes();
	connectNodes(false);

	getResults();

	float distance = outDistances.at(0);
	ASSERT_EQ(distance, outPoints.at(0).length());
	ASSERT_EQ(distance, cubeZDistance - CUBE_HALF_EDGE);

	disconnectNodes(false);
	rayTf.clear();

	// Translate ray position and compute the distance again
	float rayZAxisTranslation = 5.0f;
	rayTf.emplace_back(Mat3x4f::translation(0.0f, 0.0f, rayZAxisTranslation).toRGL());

	prepareNodes();
	connectNodes(false);

	getResults();

	// The distance should change if it is computed from the beginning of the ray
	// and not from the beginning of its coordinate system
	EXPECT_EQ(outDistances.at(0), outPoints.at(0).length() - rayZAxisTranslation);
	EXPECT_EQ(distance - outDistances.at(0), rayZAxisTranslation);
}

TEST_F(DistanceFieldTest, should_change_distance_when_gaussian_distance_noise_considered_mean)
{
	float cubeZDistance = 10.0f;
	const Mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance);
	spawnCubeOnScene(nullptr, cubePoseTf);

	rayTf.emplace_back(Mat3x4f::identity().toRGL());

	prepareNodes();
	connectNodes(false);

	getResults();

	float distance = outDistances.at(0);
	ASSERT_EQ(distance, outPoints.at(0).length());
	ASSERT_EQ(distance, cubeZDistance - CUBE_HALF_EDGE);

	disconnectNodes(false);

	// Add gaussian noise to the distance
	ASSERT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, MEAN, 0.0f, 0.0f));
	connectNodes(true);

	getResults();

	EXPECT_EQ(outDistances.at(0), outPoints.at(0).length());
	EXPECT_THAT(outDistances.at(0) - distance, testing::FloatNear(MEAN, EPSILON_NOISE));
}

TEST_F(DistanceFieldTest, should_change_distance_when_gaussian_distance_noise_considered_mean_and_std_dev)
{
	float cubeZDistance = 4.0f;
	const Mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance);
	spawnCubeOnScene(nullptr, cubePoseTf);

	rayTf.assign(LIDAR_RAYS_COUNT, Mat3x4f::identity().toRGL());

	ASSERT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, MEAN, STD_DEV, STD_DEV_PER_METER));
	prepareNodes();
	connectNodes(true);

	getResults();

	auto [realMean, realStdev] = calcMeanAndStdev(outDistances);
	float expectedMean = cubeZDistance - CUBE_HALF_EDGE + MEAN;
	float expectedStdev = (cubeZDistance - CUBE_HALF_EDGE) * STD_DEV_PER_METER + STD_DEV;

	EXPECT_THAT(expectedMean, testing::FloatNear(realMean, EPSILON_NOISE));
	EXPECT_THAT(expectedStdev, testing::FloatNear(realStdev, EPSILON_NOISE));
}

TEST_P(DistanceFieldTest, should_compute_correct_distances_when_points_transformed)
{
	float cubeZDistance = GetParam();
	const Mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance);
	spawnCubeOnScene(nullptr, cubePoseTf);

	float rayZDistance = cubeZDistance / 2;
	rgl_mat3x4f transform = Mat3x4f::translation(0, 0, rayZDistance).toRGL();
	rayTf.emplace_back(transform);

	// Before points transformation
	prepareNodes();
	connectNodes(false);

	getResults();

	float distanceBeforePointsTransform = outDistances.at(0);
	ASSERT_EQ(distanceBeforePointsTransform, outPoints.at(0).length() - rayZDistance);
	ASSERT_EQ(distanceBeforePointsTransform, cubeZDistance - rayZDistance - CUBE_HALF_EDGE);

	disconnectNodes(false);

	// After points transformation
	prepareNodes();
	connectNodes(false);
	rgl_mat3x4f inverseTransform = Mat3x4f::fromRGL(transform).inverse().toRGL();
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPointsNode, &inverseTransform));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(yieldPointsNode, transformPointsNode));

	getResults(transformPointsNode);

	EXPECT_EQ(outDistances.at(0), outPoints.at(0).length());
	EXPECT_EQ(outDistances.at(0), distanceBeforePointsTransform);
}
