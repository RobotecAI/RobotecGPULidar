#pragma once

#include <cuda.h>
#include <curand_kernel.h>

#include "DeviceBuffer.hpp"
#include "data_types/PCLFormats.h"
#include "data_types/LidarNoiseParams.h"
#include "math/Mat3x4f.hpp"

void setupGaussianNoiseGenerator(const unsigned seed,
                                 cudaStream_t stream,
                                 DeviceBuffer<curandStatePhilox4_32_10_t>& dPHILOXStates);

void addGaussianNoise(cudaStream_t stream,
                      const DeviceBuffer<Mat3x4f>& dRayPoses,
                      const LidarNoiseParams& lidar_noise_params,
                      DeviceBuffer<curandStatePhilox4_32_10_t>& dPHILOXStates,
                      DeviceBuffer<Mat3x4f>& dRayPosesWithNoise);

void addGaussianNoise(cudaStream_t stream,
                      const DeviceBuffer<PCL12>& dInputPointCloud,
                      const DeviceBuffer<Point3f>& dVisualizationPointCloud,
                      const Point3f visualization_point_cloud_origin,
                      const LidarNoiseParams& lidar_noise_params,
                      DeviceBuffer<curandStatePhilox4_32_10_t>& dPHILOXStates,
                      DeviceBuffer<PCL12>& dResults,
                      DeviceBuffer<Point3f>& dVisualizationPointCloudWithNoise);
