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
#include <gpu/helpersKernels.hpp>

// Philox algorithm chosen based on performance
// https://stackoverflow.com/questions/18506697/curand-properties-of-generators
__global__ void kSetupRandomNumberGenerator(size_t pointCount, unsigned int seed, curandStatePhilox4_32_10_t* states)
{
	LIMIT(pointCount);
	/* Each thread gets same seed, a different sequence number, no offset */
	curand_init(seed, tid, 0, &states[tid]);
}

void gpuSetupRandomNumberGenerator(cudaStream_t stream, size_t elementsCount, unsigned int seed,
                                   curandStatePhilox4_32_10_t* outPHILOXStates)
{
	run(kSetupRandomNumberGenerator, stream, elementsCount, seed, outPHILOXStates);
}

__global__ void kUpdateVertices(size_t vertexCount, Vec3f* newVerticesToVelocity, Vec3f* oldToNewVertices)
{
	LIMIT(vertexCount);
	// See Mesh::updateVertices to understand the logic here.
	Vec3f newVertex = newVerticesToVelocity[tid];
	newVerticesToVelocity[tid] -= oldToNewVertices[tid];
	oldToNewVertices[tid] = newVertex;
}

void gpuUpdateVertices(cudaStream_t stream, size_t vertexCount, Vec3f* newVerticesToVelocity, Vec3f* oldToNewVertices)
{
	run(kUpdateVertices, stream, vertexCount, newVerticesToVelocity, oldToNewVertices);
}
