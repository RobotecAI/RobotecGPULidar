#include "gaussianNoise.h"

#include <stdio.h>
#include <stdlib.h>

#include "gaussianNoise.cu"

// Philox algorithm chosen based on performance
// https://stackoverflow.com/questions/18506697/curand-properties-of-generators

__global__ void kSetupGaussianNoiseGenerator(unsigned int seed, int point_cloud_size, curandStatePhilox4_32_10_t *state)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    if (id >= point_cloud_size) {
        return;
    }
    /* Each thread gets same seed, a different sequence
       number, no offset */
    curand_init(seed, id, 0, &state[id]);
}


__global__ void kAddGaussianNoise(int point_cloud_size,
                                  const PCL12* input_point_cloud,
                                  const Point3f* input_visualization_point_cloud,
                                  const Point3f visualization_point_cloud_origin,
                                  const LidarNoiseParams lidar_noise_params,
                                  curandStatePhilox4_32_10_t *state,
                                  PCL12 *result_point_cloud,
                                  Point3f* result_visualization_point_cloud)
{
    const int id = threadIdx.x + blockIdx.x * blockDim.x;
    if (id >= point_cloud_size) {
        return;
    }

    if (lidar_noise_params.angularNoiseType == rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_HITPOINT_BASED) {
        addAngularGaussianNoise(input_point_cloud[id],
                                input_visualization_point_cloud[id],
                                visualization_point_cloud_origin,
                                lidar_noise_params,
                                state[id],
                                result_point_cloud[id],
                                result_visualization_point_cloud[id]);
    }

    addDistanceGaussianNoise(result_point_cloud[id],
                             result_visualization_point_cloud[id],
                             visualization_point_cloud_origin,
                             lidar_noise_params,
                             state[id],
                             result_point_cloud[id],
                             result_visualization_point_cloud[id]);
}

__global__ void kAddGaussianNoise(int point_cloud_size,
                                  const TransformMatrix* ray_poses,
                                  LidarNoiseParams lidar_noise_params,
                                  curandStatePhilox4_32_10_t* state,
                                  TransformMatrix *ray_poses_with_noise) {
    const int id = threadIdx.x + blockIdx.x * blockDim.x;
    if (id >= point_cloud_size) {
        return;
    }
    if (lidar_noise_params.angularNoiseType == rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED) {
        ray_poses_with_noise[id] = addAngularGaussianNoise(ray_poses[id], lidar_noise_params,
                                                           state[id]);
    }
}

void setupGaussianNoiseGenerator(unsigned int seed,
                                 cudaStream_t stream,
                                 DeviceBuffer<curandStatePhilox4_32_10_t>& dPHILOXStates)
{
    const int cloud_size = dPHILOXStates.getElemCount();
    const int thread_per_block_count = 256;
    const int block_count = (cloud_size + thread_per_block_count) / thread_per_block_count;

    kSetupGaussianNoiseGenerator<<<block_count, thread_per_block_count, 0, stream>>>(seed, cloud_size, dPHILOXStates.writeDevice());
}


void addGaussianNoise(cudaStream_t stream,
                      const DeviceBuffer<TransformMatrix>& dRayPoses,
                      const LidarNoiseParams& lidar_noise_params,
                      DeviceBuffer<curandStatePhilox4_32_10_t>& dPHILOXStates,
                      DeviceBuffer<TransformMatrix>& dRayPosesWithNoise) {

    const int cloud_size = dRayPoses.getElemCount();
    const int thread_per_block_count = 256;
    const int block_count = (cloud_size + thread_per_block_count) / thread_per_block_count;

    kAddGaussianNoise<<<block_count, thread_per_block_count, 0, stream>>>(
        cloud_size,
        dRayPoses.readDevice(),
        lidar_noise_params,
        dPHILOXStates.writeDevice(),
        dRayPosesWithNoise.writeDevice());
}

void addGaussianNoise(cudaStream_t stream,
                      const DeviceBuffer<PCL12>& dInputPointCloud,
                      const DeviceBuffer<Point3f>& dVisualizationPointCloud,
                      const Point3f visualization_point_cloud_origin,
                      const LidarNoiseParams& lidar_noise_params,
                      DeviceBuffer<curandStatePhilox4_32_10_t>& dPHILOXStates,
                      DeviceBuffer<PCL12>& dResults,
                      DeviceBuffer<Point3f>& dVisualizationPointCloudWithNoise) {

    const int cloud_size = dInputPointCloud.getElemCount();
    const int thread_per_block_count = 256;
    const int block_count = (cloud_size + thread_per_block_count) / thread_per_block_count;

    kAddGaussianNoise<<<block_count, thread_per_block_count, 0, stream>>>(
        cloud_size,
        dInputPointCloud.readDevice(),
        dVisualizationPointCloud.readDevice(),
        visualization_point_cloud_origin,
        lidar_noise_params,
        dPHILOXStates.writeDevice(),
        dResults.writeDevice(),
        dVisualizationPointCloudWithNoise.writeDevice());
}
