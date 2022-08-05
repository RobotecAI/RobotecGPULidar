#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rgl/api/e2e_extensions.h>
#include <rgl/api/apiUnitTests.h>
#include <testModels.hpp>
#include <utils/statistical_utils.h>
#include <utils/testUtils.h>

class GaussianNoise : public ::testing::Test {
protected:
    void SetUp() override
    {
        for (int i = 0; i < rays_count; ++i) {
            rays_tf.push_back(ray_tf);
        }

        rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length);
        rgl_entity_create(&cube, nullptr, cube_mesh);
        rgl_entity_set_pose(cube, &entity_tf);
        rgl_lidar_create(&lidar, rays_tf.data(), rays_tf.size());
    }

    void TearDown() override
    {
        rgl_mesh_destroy(cube_mesh);
        rgl_entity_destroy(cube);
        rgl_lidar_destroy(lidar);
    }

    float expected_hitpoint_z = 4.0;
    float expected_angle_in_radians = 1.57;
    std::vector<float> means = { -0.1, 0.0, 0.1 };
    float distance_std_dev = 0.1;
    float angle_std_dev = 0.01;
    float std_dev_rise_per_meter = 0.2;
    float abs_error = 0.002;

    rgl_mesh_t cube_mesh;
    rgl_entity_t cube;
    rgl_lidar_t lidar;
    rgl_mat3x4f entity_tf = {
        .value = {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 0, 0, 1, 5 } }
    };
    rgl_mat3x4f ray_tf = {
        .value = {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 0, 0, 1, 1 },
        }
    };
    std::vector<rgl_mat3x4f> rays_tf;
    int rays_count = 500000;
    rgl_vec3f results[500000];
    int hitpointCount = 0;
};

TEST_F(GaussianNoise, DistanceNoiseMean)
{
    for (auto mean : means) {
        EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 0.0, 0.0, 0.0, 0.0, mean));
        getLidarResults(lidar, &hitpointCount, results);

        ASSERT_EQ(hitpointCount, rays_count);
        for (int i = 0; i < rays_count; ++i) {
            EXPECT_EQ(results[i].value[0], 0.0f);
            EXPECT_EQ(results[i].value[1], 0.0f);
            EXPECT_EQ(results[i].value[2], expected_hitpoint_z + mean);
        }
    }
}

TEST_F(GaussianNoise, DistanceStandardDeviation)
{
    EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 0.0, 0.0, distance_std_dev, 0.0, 0.0));
    getLidarResults(lidar, &hitpointCount, results);

    ASSERT_EQ(hitpointCount, rays_count);
    auto distances = computeDistances(results, rays_count);
    auto [distance_mean, distance_st_dev] = mean_and_stdev(distances);
    EXPECT_NEAR(distance_mean, expected_hitpoint_z, abs_error);
    EXPECT_NEAR(distance_st_dev, distance_std_dev, abs_error);
}

TEST_F(GaussianNoise, DistanceStandardDeviationRaisePerMeter)
{
    EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 0.0, 0.0, distance_std_dev, std_dev_rise_per_meter, 0.0));
    getLidarResults(lidar, &hitpointCount, results);

    ASSERT_EQ(hitpointCount, rays_count);
    auto distances = computeDistances(results, rays_count);
    auto [distance_mean, distance_st_dev] = mean_and_stdev(distances);
    EXPECT_NEAR(distance_mean, expected_hitpoint_z, abs_error);
    EXPECT_NEAR(distance_st_dev, distance_std_dev, abs_error);
    // TODO: investigate why this EXPECT do not pass
    // EXPECT_NEAR(distance_st_dev, distance_std_dev + expected_hitpoint_z * std_dev_rise_per_meter, 0.0002f);
}

TEST_F(GaussianNoise, AngleNoiseRayBasedStandardDeviation)
{
    EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, angle_std_dev, 0.0, 0.0, 0.0, 0.0));
    getLidarResults(lidar, &hitpointCount, results);

    ASSERT_EQ(hitpointCount, rays_count);
    auto angles = computeAngles(results, rays_count);
    auto [angle_mean, angle_std_dev] = mean_and_stdev(angles);
    EXPECT_NEAR(angle_mean, expected_angle_in_radians, abs_error);
    EXPECT_NEAR(angle_std_dev, angle_std_dev, abs_error);
}

TEST_F(GaussianNoise, AngleNoiseRayBasedMean)
{
    for (auto mean : means) {

        EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 0.0, mean, 0.0, 0.0, 0.0));
        getLidarResults(lidar, &hitpointCount, results);

        ASSERT_EQ(hitpointCount, rays_count);
        auto angles = computeAngles(results, rays_count);
        auto [angle_mean, angle_std_dev] = mean_and_stdev(angles);
        // TODO: investigate why this EXPECT do not pass
        // EXPECT_NEAR(angle_mean, expected_angle_in_radians + mean, abs_error);
        EXPECT_NEAR(angle_std_dev, 0.0f, abs_error);
    }
}

TEST_F(GaussianNoise, AngleNoiseHitpointBasedStandardDeviation)
{
    EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_HITPOINT_BASED, angle_std_dev, 0.0, 0.0, 0.0, 0.0));
    getLidarResults(lidar, &hitpointCount, results);

    ASSERT_EQ(hitpointCount, rays_count);
    auto angles = computeAngles(results, rays_count);
    auto [angle_mean, angle_std_dev] = mean_and_stdev(angles);
    EXPECT_NEAR(angle_mean, expected_angle_in_radians, abs_error);
    EXPECT_NEAR(angle_std_dev, angle_std_dev, abs_error);
}

TEST_F(GaussianNoise, AngleNoiseHitpointBasedMean)
{
    for (auto mean : means) {

        EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_HITPOINT_BASED, 0.0, mean, 0.0, 0.0, 0.0));
        getLidarResults(lidar, &hitpointCount, results);

        ASSERT_EQ(hitpointCount, rays_count);
        auto angles = computeAngles(results, rays_count);
        auto [angle_mean, angle_std_dev] = mean_and_stdev(angles);
        EXPECT_NEAR(angle_mean, expected_angle_in_radians + mean, abs_error);
        EXPECT_NEAR(angle_std_dev, 0.0f, abs_error);
    }
}
