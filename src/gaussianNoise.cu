#include <cuda.h>
#include <curand_kernel.h>

#include "data_types/PCLFormats.h"
#include "data_types/LidarNoiseParams.h"
#define HOSTDEVICE __device__
#include "linearGeometry.h"

template<class T>
__device__ T rotatePointAroundY(const T& point, float angle_rad) {
    return T { cos(angle_rad) * point.x - sin(angle_rad) * point.z,
               point.y,
               cos(angle_rad) * point.z + sin(angle_rad) * point.x };
}

template<class T>
__device__ float length(const T& vector) {
    return sqrt(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);
}

template<class T>
__device__ T scale(const T& point, float scale) {
    return {point.x * scale, point.y * scale, point.z * scale};
}

template<class T>
__device__ T normalized(const T& point) {
    return scale(point, 1.0f/length(point));
}

template<class T>
__device__ T addDistanceNoise(const T& point, float distance_error) {
    T point_normalized = normalized(point);

    float distance = length(point);
    float new_distance = distance + distance_error;

    return scale(point_normalized, new_distance);
}

__device__ Mat3x4f addAngularGaussianNoise(const Mat3x4f& transform_to_rotate,
                                           const LidarNoiseParams& lidar_noise_params,
                                           curandStatePhilox4_32_10_t& randomization_state) {
    float angle =
        lidar_noise_params.angularNoiseMean + curand_normal(&randomization_state) * lidar_noise_params.angularNoiseStDev;

    return multiply3x4TransformMatrices(yAxisRotation3x4Matrix(angle), transform_to_rotate);
}

__device__ void addAngularGaussianNoise(const PCL12& input_point,
                                           const Point3f& input_visualization_point,
                                           const Point3f& visualization_point_cloud_origin,
                                           const LidarNoiseParams& lidar_noise_params,
                                           curandStatePhilox4_32_10_t& randomization_state,
                                           PCL12& result_point,
                                           Point3f& result_visualization_point) {
    float angular_offset =
        lidar_noise_params.angularNoiseMean + curand_normal(&randomization_state) * lidar_noise_params.angularNoiseStDev;

    result_point = rotatePointAroundY(input_point, angular_offset);

    Point3f point = {
        input_visualization_point.x - visualization_point_cloud_origin.x,
        input_visualization_point.y - visualization_point_cloud_origin.y,
        input_visualization_point.z - visualization_point_cloud_origin.z
    };

    Point3f point_with_noise = rotatePointAroundY(point, angular_offset);

    result_visualization_point.x = visualization_point_cloud_origin.x + point_with_noise.x;
    result_visualization_point.y = visualization_point_cloud_origin.y + point_with_noise.y;
    result_visualization_point.z = visualization_point_cloud_origin.z + point_with_noise.z;
}
__device__ void addDistanceGaussianNoise(const PCL12& input_point,
                                            const Point3f& input_visualization_point,
                                            const Point3f& visualization_point_cloud_origin,
                                            const LidarNoiseParams& lidar_noise_params,
                                            curandStatePhilox4_32_10_t& randomization_state,
                                            PCL12& result_point,
                                            Point3f& result_visualization_point) {
    float distance = length(input_point);
    float distance_induced_st_dev = distance * lidar_noise_params.distanceNoiseStDevRisePerMeter;
    float total_st_dev = distance_induced_st_dev + lidar_noise_params.distanceNoiseStDevBase;

    float distance_error =
        lidar_noise_params.distanceNoiseMean + curand_normal(&randomization_state) * total_st_dev;

    result_point = addDistanceNoise(input_point, distance_error);

    Point3f point = {
        input_visualization_point.x - visualization_point_cloud_origin.x,
        input_visualization_point.y - visualization_point_cloud_origin.y,
        input_visualization_point.z - visualization_point_cloud_origin.z
    };

    Point3f point_with_noise = addDistanceNoise(point, distance_error);

    result_visualization_point.x = visualization_point_cloud_origin.x + point_with_noise.x;
    result_visualization_point.y = visualization_point_cloud_origin.y + point_with_noise.y;
    result_visualization_point.z = visualization_point_cloud_origin.z + point_with_noise.z;
}

