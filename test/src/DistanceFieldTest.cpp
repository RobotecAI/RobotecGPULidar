#include <DistanceFieldTestHelper.hpp>
#include <RGLFields.hpp>

class DistanceFieldTest : public RGLTestWithParam<float>, public DistanceFieldTestHelper { };

INSTANTIATE_TEST_CASE_P(DistanceFieldTests,
    DistanceFieldTest,
    ::testing::Values(23.1753849f, 119.9884538f, 333.3394629f));

TEST_P(DistanceFieldTest, should_compute_correct_distance_for_single_object_on_ray_path)
{
    float cubeZDistance = GetParam();
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    rayTf.push_back(Mat3x4f::identity().toRGL());

    prepareNodes({0.0f, cubeZDistance + 10.0f});
    connectNodes(false);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    ASSERT_EQ(outPointsCount, rayTf.size());
    EXPECT_EQ(outDistances.at(0), outPoints.at(0).length());
    EXPECT_EQ(outDistances.at(0), cubeZDistance - CUBE_HALF_EDGE);
}

TEST_F(DistanceFieldTest, should_compute_correct_distance_when_ray_origin_on_cube_face)
{
    float cubeZDistance = 1.0;
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    rayTf.push_back(Mat3x4f::identity().toRGL());

    prepareNodes({0.0f, cubeZDistance + 10.0f});
    connectNodes(false);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    ASSERT_EQ(outPointsCount, rayTf.size());
    EXPECT_EQ(outDistances.at(0), outPoints.at(0).length());
    EXPECT_EQ(outDistances.at(0), 2 * CUBE_HALF_EDGE);
}

TEST_F(DistanceFieldTest, should_compute_correct_distance_when_ray_origin_inside_cube)
{
    float cubeZDistance = -0.001f;
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    rayTf.push_back(Mat3x4f::identity().toRGL());

    prepareNodes({0.0f, cubeZDistance + 10.0f});
    connectNodes(false);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    ASSERT_EQ(outPointsCount, rayTf.size());
    EXPECT_EQ(outDistances.at(0), outPoints.at(0).length());
    EXPECT_EQ(outDistances.at(0), CUBE_HALF_EDGE + cubeZDistance);
}

TEST_F(DistanceFieldTest, should_compute_infinite_distance_for_object_off_ray_path)
{
    float cubeYAxisTranslation = 55.5f;
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, cubeYAxisTranslation, 0).toRGL();
    prepareSceneWithCube(cubePoseTf);

    rayTf.push_back(Mat3x4f::identity().toRGL());
    // After sending a ray in the direction of negative infinity, the distance should still be positive.
    rayTf.push_back(Mat3x4f::rotationRad(RGL_AXIS_X, M_PI).toRGL());

    prepareNodes({0.0f, 10000.0f});
    connectNodes(false);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    EXPECT_EQ(outDistances.at(0), NON_HIT_VALUE);
    EXPECT_EQ(outDistances.at(1), NON_HIT_VALUE);
}

TEST_P(DistanceFieldTest, should_compute_infinite_distance_for_object_out_of_max_range)
{
    float cubeZDistance = GetParam();
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    rayTf.push_back(Mat3x4f::identity().toRGL());

    prepareNodes({0.0f, cubeZDistance - CUBE_HALF_EDGE - 1.0f});
    connectNodes(false);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    EXPECT_EQ(outDistances.at(0), NON_HIT_VALUE);
}

TEST_P(DistanceFieldTest, should_compute_infinite_distance_for_object_out_of_min_range)
{
    float cubeZDistance = GetParam();
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    rayTf.push_back(Mat3x4f::identity().toRGL());

    prepareNodes({cubeZDistance + CUBE_HALF_EDGE + 1.0f, cubeZDistance + CUBE_HALF_EDGE + 2.0f});
    connectNodes(false);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    EXPECT_EQ(outDistances.at(0), NON_HIT_VALUE);
}

TEST_P(DistanceFieldTest, should_compute_correct_distances_for_various_ray_angles)
{
    float cubeZDistance = GetParam();
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    float maxAngle = atan(CUBE_HALF_EDGE / (cubeZDistance - CUBE_HALF_EDGE));

    // Generate angle and print the seed
    std::random_device rd;
    auto seed = rd();
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> dis(-maxAngle, maxAngle);
    std::cerr << "DistanceFieldTest various_angle seed: " << seed << std::endl;

    std::vector<float> expectedDistances;

    for (int i = 0; i < ANGLE_ITERATIONS; ++i) {
        float randomAngle = dis(gen);
        rayTf.push_back(Mat3x4f::rotationRad(RGL_AXIS_X, randomAngle).toRGL());

        float expectedDistance = (cubeZDistance - CUBE_HALF_EDGE) / (cos(randomAngle));
        expectedDistances.push_back(expectedDistance);
    }

    prepareNodes({0.0f, cubeZDistance + 10.0f});
    connectNodes(false);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    ASSERT_EQ(outPointsCount, rayTf.size());

    for (int i = 0; i < outPointsCount; ++i) {
        EXPECT_NEAR(outPoints.at(i).length(), outDistances.at(i), EPSILON_MUL);
        EXPECT_NEAR(expectedDistances.at(i), outDistances.at(i), EPSILON_MUL);
    }
}

TEST_F(DistanceFieldTest, should_compute_distance_from_ray_beginning)
{
    float cubeZDistance = 10.0f;
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    rayTf.push_back(Mat3x4f::identity().toRGL());

    prepareNodes({0.0f, cubeZDistance + 10.0f});
    connectNodes(false);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    ASSERT_EQ(outPointsCount, rayTf.size());

    float distance = outDistances.at(0);
    ASSERT_EQ(distance, outPoints.at(0).length());
    ASSERT_EQ(distance, cubeZDistance - CUBE_HALF_EDGE);

    disconnectNodes(false);
    rayTf.clear();

    // Translate ray position and compute the distance again
    float rayZAxisTranslation = 5.0f;
    rayTf.push_back(Mat3x4f::translation(0.0f, 0.0f, rayZAxisTranslation).toRGL());

    prepareNodes({0.0f, cubeZDistance + 10.0f});
    connectNodes(false);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    ASSERT_EQ(outPointsCount, rayTf.size());

    // The distance should change if it is computed from the beginning of the ray
    // and not from the beginning of its coordinate system
    EXPECT_EQ(outDistances.at(0), outPoints.at(0).length() - rayZAxisTranslation);
    EXPECT_EQ(distance - outDistances.at(0), rayZAxisTranslation);
}

TEST_F(DistanceFieldTest, should_change_distance_when_gaussian_distance_noise_considered_mean)
{
    float cubeZDistance = 10.0f;
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    rayTf.push_back(Mat3x4f::identity().toRGL());

    prepareNodes({0.0f, cubeZDistance + 10.0f});
    connectNodes(false);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    ASSERT_EQ(outPointsCount, rayTf.size());

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
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    rayTf.assign(LIDAR_RAYS_COUNT, Mat3x4f::identity().toRGL());

    ASSERT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, MEAN, STD_DEV, STD_DEV_PER_METER));
    prepareNodes({0.0f, cubeZDistance + 10.0f});
    connectNodes(true);

    getResults();

    ASSERT_EQ(outDistancesCount, rayTf.size());
    ASSERT_EQ(outPointsCount, rayTf.size());

    auto [realMean, realStdev] = calcMeanAndStdev(outDistances);
    float expectedMean = cubeZDistance - CUBE_HALF_EDGE + MEAN;
    float expectedStdev = (cubeZDistance - CUBE_HALF_EDGE) * STD_DEV_PER_METER + STD_DEV;

    EXPECT_THAT(expectedMean, testing::FloatNear(realMean, EPSILON_NOISE));
    EXPECT_THAT(expectedStdev, testing::FloatNear(realStdev, EPSILON_NOISE));
}

TEST_P(DistanceFieldTest, should_compute_correct_distances_when_points_transformed)
{
    float cubeZDistance = GetParam();
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    float rayZDistance = cubeZDistance / 2;
    rgl_mat3x4f transform = Mat3x4f::translation(0, 0, rayZDistance).toRGL();
    rayTf.push_back(transform);

    // Before points transformation
    prepareNodes({0.0f, cubeZDistance + 10.0f});
    connectNodes(false);

    getResults();
    ASSERT_EQ(outDistancesCount, rayTf.size());
    ASSERT_EQ(outPointsCount, rayTf.size());

    float distanceBeforePointsTransform = outDistances.at(0);
    ASSERT_EQ(distanceBeforePointsTransform, outPoints.at(0).length() - rayZDistance);
    ASSERT_EQ(distanceBeforePointsTransform, cubeZDistance - rayZDistance - CUBE_HALF_EDGE);

    disconnectNodes(false);

    // After points transformation
    prepareNodes({0.0f, cubeZDistance + 10.0f});
    connectNodes(false);
    rgl_mat3x4f inverseTransform = Mat3x4f::fromRGL(transform).inverse().toRGL();
    EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPointsNode, &inverseTransform));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(yieldPointsNode, transformPointsNode));

    getResults(&transformPointsNode);

    ASSERT_EQ(outDistancesCount, rayTf.size());
    ASSERT_EQ(outPointsCount, rayTf.size());
    EXPECT_EQ(outDistances.at(0), outPoints.at(0).length());
    EXPECT_EQ(outDistances.at(0), distanceBeforePointsTransform);
}
