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
#include <gpu/gaussianNoiseKernels.hpp>

// Philox algorithm chosen based on performance
// https://stackoverflow.com/questions/18506697/curand-properties-of-generators

__global__ void kSetupGaussianNoiseGenerator(size_t pointCount, unsigned int seed, curandStatePhilox4_32_10_t* states)
{
	LIMIT(pointCount);
	/* Each thread gets same seed, a different sequence number, no offset */
	curand_init(seed, tid, 0, &states[tid]);
}

__global__ void kAddGaussianNoiseAngularRay(size_t rayCount, float mean, float stDev, rgl_axis_t rotationAxis, Mat3x4f toOriginTransform,
	curandStatePhilox4_32_10_t* randomStates, const Mat3x4f* inRays, Mat3x4f* outRays)
{
	LIMIT(rayCount);

	float angularError = mean + curand_normal(&randomStates[tid]) * stDev;
	float rotX = rotationAxis == RGL_AXIS_X ? angularError : 0.0f;
	float rotY = rotationAxis == RGL_AXIS_Y ? angularError : 0.0f;
	float rotZ = rotationAxis == RGL_AXIS_Z ? angularError : 0.0f;
	outRays[tid] = toOriginTransform.inverse() * (Mat3x4f::rotationRad(rotX, rotY, rotZ) * (toOriginTransform * inRays[tid]));
}

__global__ void kAddGaussianNoiseAngularHitpoint(size_t pointCount, float mean, float stDev, rgl_axis_t rotationAxis, Mat3x4f toOriginTransform,
	curandStatePhilox4_32_10_t* randomStates, const Field<XYZ_F32>::type* inPoints, Field<XYZ_F32>::type* outPoints, Field<DISTANCE_F32>::type* outDistances)
{
	LIMIT(pointCount);

	float angularError = mean + curand_normal(&randomStates[tid]) * stDev;
	float rotX = rotationAxis == RGL_AXIS_X ? angularError : 0.0f;
	float rotY = rotationAxis == RGL_AXIS_Y ? angularError : 0.0f;
	float rotZ = rotationAxis == RGL_AXIS_Z ? angularError : 0.0f;
	outPoints[tid] = toOriginTransform.inverse() * (Mat3x4f::rotationRad(rotX, rotY, rotZ) * (toOriginTransform * inPoints[tid]));

	if (outDistances != nullptr) {
		outDistances[tid] = outPoints[tid].length();
	}
}

__global__ void kAddGaussianNoiseDistance(size_t pointCount, float mean, float stDevBase, float stDevRisePerMeter, curandStatePhilox4_32_10_t* randomStates,
	const Field<XYZ_F32>::type* inPoints, Field<XYZ_F32>::type* outPoints, Field<DISTANCE_F32>::type* outDistances)
{
	LIMIT(pointCount);

	float distance = inPoints[tid].length();
	float distanceInducedStDev = distance * stDevRisePerMeter;
	float totalStDev = distanceInducedStDev + stDevBase;

	float distanceError = mean + curand_normal(&randomStates[tid]) * totalStDev;

	if (outDistances != nullptr) {
		outDistances[tid] = distance + distanceError;
	}

	outPoints[tid] = inPoints[tid].normalize() * (distance + distanceError);
}


void gpuSetupGaussianNoiseGenerator(cudaStream_t stream, size_t pointCount, unsigned int seed, curandStatePhilox4_32_10_t* outPHILOXStates)
{ run(kSetupGaussianNoiseGenerator, stream, pointCount, seed, outPHILOXStates); }

void gpuAddGaussianNoiseAngularRay(cudaStream_t stream, size_t rayCount, float mean, float stDev, rgl_axis_t rotationAxis, Mat3x4f toOriginTransform,
	curandStatePhilox4_32_10_t* randomStates, const Mat3x4f* inRays, Mat3x4f* outRays)
{ run(kAddGaussianNoiseAngularRay, stream, rayCount, mean, stDev, rotationAxis, toOriginTransform, randomStates, inRays, outRays); }

void gpuAddGaussianNoiseAngularHitpoint(cudaStream_t stream, size_t pointCount, float mean, float stDev, rgl_axis_t rotationAxis, Mat3x4f toOriginTransform,
	curandStatePhilox4_32_10_t* randomStates, const Field<XYZ_F32>::type* inPoints, Field<XYZ_F32>::type* outPoints, Field<DISTANCE_F32>::type* outDistances)
{ run(kAddGaussianNoiseAngularHitpoint, stream, pointCount, mean, stDev, rotationAxis, toOriginTransform, randomStates, inPoints, outPoints, outDistances); }

void gpuAddGaussianNoiseDistance(cudaStream_t stream, size_t pointCount, float mean, float stDevBase, float stDevRisePerMeter, curandStatePhilox4_32_10_t* randomStates,
	const Field<XYZ_F32>::type* inPoints, Field<XYZ_F32>::type* outPoints, Field<DISTANCE_F32>::type* outDistances)
{ run(kAddGaussianNoiseDistance, stream, pointCount, mean, stDevBase, stDevRisePerMeter, randomStates, inPoints, outPoints, outDistances); }
