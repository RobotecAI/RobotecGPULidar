#include "gaussianNoise.h"

#include "utils/statistical_utils.h"
#include "DeviceBuffer.hpp"

#include <fmt/printf.h>

#include <gtest/gtest.h>

#include <iostream>
#include <fstream>
#include <math.h>

#define CUDA_CALL(x) do { if((x) != cudaSuccess) { \
    printf("Error at %s:%d\n",__FILE__,__LINE__); }} while(0)


template<typename T>
std::vector<float> computeDistances(const T* data, int size) {
    std::vector<float> return_data(size);
    for (int i = 0; i < size; i++) {
        return_data[i] = sqrt(data[i].x * data[i].x + data[i].y * data[i].y + data[i].z * data[i].z);
    }
    return return_data;
}

template<typename T>
std::vector<float> computeAngles(const T* data, int size) {
    std::vector<float> return_data(size);
    for (int i = 0; i < size; i++) {
        return_data[i] = std::atan2(data[i].z, data[i].x);
    }
    return return_data;
}

template<typename T>
void saveToFile(std::vector<T> data, std::string filename) {
    std::ofstream file(filename);
    for (const T& point : data) {
        file << point.x << ";" << point.y << ";" << point.z << std::endl;
    }
    file.close();
}

template<typename T>
void verifyData(const std::vector<T>& data, float distance_st_dev_gold, float angular_st_dev_gold, std::string description) {
    {
        std::vector<float> distances(data.size());
        std::transform(data.cbegin(), data.cend(), distances.begin(), [](const T& point) {
            return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        });

        auto [average_distance, distance_st_dev] = mean_and_stdev(distances);

        EXPECT_NEAR(distance_st_dev, distance_st_dev_gold, 0.01f * distance_st_dev_gold);

        fmt::print("{}: Distance: Average: {}, Std dev {}\n", description, average_distance, distance_st_dev);
    }

    {
        std::vector<float> angles(data.size());

        std::transform(data.cbegin(), data.cend(), angles.begin(), [](const T& point) {
            return std::atan2(point.z, point.x);
        });
        auto [average, st_dev] = mean_and_stdev(angles);

        EXPECT_NEAR(st_dev, angular_st_dev_gold, 0.01f * angular_st_dev_gold);

        fmt::print("{}: Angle: Average: {}, Std dev {}\n", description, average, st_dev);
    }
}

TEST(GaussianNoise, NoiseStatsVerification) {
    const int point_cloud_size = 10000;
    std::vector<PCL12> input_point_cloud(point_cloud_size);
    for (int i = 0; i < point_cloud_size; i++) {
        input_point_cloud[i].x = 2.0f;
        input_point_cloud[i].y = 0.0f;
        input_point_cloud[i].z = 2.0f;
    }
    float points_distance = std::sqrt(8.0f);

    std::vector<Point3f> input_visualization_point_cloud(point_cloud_size);
    for (int i = 0; i < point_cloud_size; i++) {
        input_visualization_point_cloud[i].x = 10.0f;
        input_visualization_point_cloud[i].y = 0.0f;
        input_visualization_point_cloud[i].z = 2.0f;
    }
    Point3f visualization_point_cloud_origin = {8.0f, 0.0f, 0.0f};

    float angular_st_dev_gold = 0.1f;
    float distance_st_dev_gold_base = 0.3f;
    float distance_st_dev_rise_per_meter_gold = 0.1f;
    float distance_st_dev_gold = distance_st_dev_gold_base + points_distance * distance_st_dev_rise_per_meter_gold;

    DeviceBuffer<curandStatePhilox4_32_10_t> dPHILOXStates;

    cudaStream_t stream;
    CUDA_CALL(cudaStreamCreate(&stream));

    dPHILOXStates.resizeToFit(point_cloud_size);

    DeviceBuffer<PCL12> dInputPointCloud;
    dInputPointCloud.resizeToFit(point_cloud_size);
    dInputPointCloud.copyFromHostAsync(input_point_cloud.data(), input_point_cloud.size(), stream);

    DeviceBuffer<Point3f> dInputVisualizationPointCloud;
    dInputVisualizationPointCloud.resizeToFit(point_cloud_size);
    dInputVisualizationPointCloud.copyFromHostAsync(input_visualization_point_cloud.data(), input_visualization_point_cloud.size(), stream);

    DeviceBuffer<PCL12> dResults;

    dResults.resizeToFit(point_cloud_size);

    LidarNoiseParams lidarNoiseParams = {
        .angularNoiseType = AngularNoiseType::HITPOINT_BASED,
        .angularNoiseStDev = angular_st_dev_gold,
        .angularNoiseMean = 1.0f,
        .distanceNoiseStDevBase = distance_st_dev_gold_base,
        .distanceNoiseStDevRisePerMeter = distance_st_dev_rise_per_meter_gold,
        .distanceNoiseMean = 0.0f
    };

    setupGaussianNoiseGenerator(1234, stream, dPHILOXStates);
    addGaussianNoise(stream, dInputPointCloud, dInputVisualizationPointCloud, visualization_point_cloud_origin,
        lidarNoiseParams, dPHILOXStates, dInputPointCloud, dInputVisualizationPointCloud);

    HostPinnedBuffer<PCL12> hResults;
    hResults.copyFromDeviceAsync(dInputPointCloud, stream);

    HostPinnedBuffer<Point3f> hVisualizationResults;
    hVisualizationResults.copyFromDeviceAsync(dInputVisualizationPointCloud, stream);

    CUDA_CALL(cudaStreamSynchronize(stream));
    CUDA_CALL(cudaStreamDestroy(stream));

    std::vector<PCL12> results(hResults.readHost(), hResults.readHost() + hResults.getElemCount());
    saveToFile(results, "pcl.csv");

    std::vector<Point3f> visualization_results(hVisualizationResults.readHost(), hVisualizationResults.readHost() + hVisualizationResults.getElemCount());
    saveToFile(visualization_results, "visualized_pcl.csv");

    verifyData(results, distance_st_dev_gold, angular_st_dev_gold, "Point cloud results");

    std::vector<Point3f> centered_points(visualization_results.size());
    std::transform(visualization_results.cbegin(), visualization_results.cend(), centered_points.begin(), [visualization_point_cloud_origin](const Point3f& point) {
        return Point3f{point.x - visualization_point_cloud_origin.x, point.y - visualization_point_cloud_origin.y, point.z - visualization_point_cloud_origin.z};
    });

    verifyData(centered_points, distance_st_dev_gold, angular_st_dev_gold, "Point cloud visualization results");
}
