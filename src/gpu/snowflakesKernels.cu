// Copyright 2023 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cuda.h>
#include <curand_kernel.h>
#include <gpu/kernelUtils.hpp>

#include <gpu/snowflakesKernels.hpp>

__global__ void kSetupSnowflakesDisk(size_t snowflakesCount, float maxRange, float meanSnowflakeDiameter, float distributionRate, Vec3f* outSnowflakesDisk, curandStatePhilox4_32_10_t* randomStates)
{
    LIMIT(snowflakesCount);
    // Non-uniformly distributed distance [0, maxRange] from the lidar.
    // This is necessary to obtain uniform distribution on the disk.
    float snowflakeR = sqrt(curand_uniform(&randomStates[tid])) * maxRange;
    // Uniformly distributed angle [0, 2pi] (whole circle) from the lidar.
    float snowflakeAngle = curand_uniform(&randomStates[tid]) * 2 * M_PI;

    // Transform from polar coordinates to cartesian
    float x = snowflakeR * cos(snowflakeAngle);
    float y = snowflakeR * sin(snowflakeAngle);

    // Generate random values form 0 to 1 uniformly. Then transform them to exponential distribution with lambda = distributionRate.
    float diameter = curand_uniform(&randomStates[tid]);
    diameter*=meanSnowflakeDiameter;
    diameter = log(1-diameter)/(-distributionRate);

    outSnowflakesDisk[tid] = Vec3f(x, y, diameter);
    printf("x: %f, y: %f, diameter: %f\n", x, y, diameter);
}


void gpuSnowflakesSimulationCalculateDisk(cudaStream_t stream, size_t snowflakeCount,
    float maxRange, float meanSnowflakeDiameter, float distributionRate, Vec3f* outSnowflakesDisk, curandStatePhilox4_32_10_t* randomStates)
{ run(kSetupSnowflakesDisk, stream, snowflakeCount, maxRange, meanSnowflakeDiameter, distributionRate,  outSnowflakesDisk, randomStates);}
