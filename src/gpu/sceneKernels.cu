// Copyright 2024 Robotec.AI
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

#include <gpu/kernelUtils.hpp>
#include <gpu/sceneKernels.hpp>

__global__ void kPerformSkeletonAnimation(size_t vertexCount, const Vec3f* restposeVertices, const BoneWeights* boneWeights,
                                          const Mat3x4f* animationMatrices, Vec3f* skinnedVertices)
{
	LIMIT(vertexCount);

	skinnedVertices[tid] = (animationMatrices[boneWeights[tid].boneIndexes.x] * restposeVertices[tid]) *
	                       boneWeights[tid].weights.x;
	skinnedVertices[tid] += (animationMatrices[boneWeights[tid].boneIndexes.y] * restposeVertices[tid]) *
	                        boneWeights[tid].weights.y;
	skinnedVertices[tid] += (animationMatrices[boneWeights[tid].boneIndexes.z] * restposeVertices[tid]) *
	                        boneWeights[tid].weights.z;
	skinnedVertices[tid] += (animationMatrices[boneWeights[tid].boneIndexes.w] * restposeVertices[tid]) *
	                        boneWeights[tid].weights.w;
}

__global__ void kCalculateAnimationMatrices(size_t boneCount, const Mat3x4f* restposes, Mat3x4f* animationMatrices)
{
	LIMIT(boneCount);
	animationMatrices[tid] = animationMatrices[tid] * restposes[tid];
}

// Updates vertices and calculates their displacement.
// Input: newVertices and oldVertices
// Output: verticesDisplacement and newVertices
__global__ void kUpdateVertices(size_t vertexCount, Vec3f* newVerticesToDisplacement, Vec3f* oldToNewVertices)
{
	LIMIT(vertexCount);
	// See ExternalAnimator::animate or SkeletonAnimator::animate to understand the logic here.
	Vec3f newVertex = newVerticesToDisplacement[tid];
	newVerticesToDisplacement[tid] -= oldToNewVertices[tid];
	oldToNewVertices[tid] = newVertex;
}

void gpuPerformSkeletonAnimation(cudaStream_t stream, size_t vertexCount, size_t boneCount, const Vec3f* restposeVertices,
                                 const BoneWeights* boneWeights, const Mat3x4f* restposes, Mat3x4f* animationMatrices,
                                 Vec3f* skinnedVertices)
{
	run(kCalculateAnimationMatrices, stream, boneCount, restposes, animationMatrices);
	run(kPerformSkeletonAnimation, stream, vertexCount, restposeVertices, boneWeights, animationMatrices, skinnedVertices);
}

void gpuUpdateVerticesWithDisplacement(cudaStream_t stream, size_t vertexCount, Vec3f* newVerticesToDisplacement,
                                       Vec3f* oldToNewVertices)
{
	run(kUpdateVertices, stream, vertexCount, newVerticesToDisplacement, oldToNewVertices);
}
