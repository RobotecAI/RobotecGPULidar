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

#pragma once

#include <gpu/sceneKernels.hpp>
#include <scene/Mesh.hpp>

/*
 * Animator that performs skeleton animation based on bone weights, restposes and the current bone pose.
 * The result of the animation is new vertices of the mesh.
 * Vertex animation displacement is the result of subtraction between current and previous vertices.
 */
struct SkeletonAnimator
{
	friend struct Entity;

	explicit SkeletonAnimator(const std::shared_ptr<Mesh>& mesh);

	void animate(const Mat3x4f* pose, std::size_t bonesCount);

private:
	std::shared_ptr<Mesh> mesh;

	DeviceSyncArray<Vec3f>::Ptr dAnimatedVertices = DeviceSyncArray<Vec3f>::create();
	DeviceSyncArray<Vec3f>::Ptr dVertexAnimationDisplacement = DeviceSyncArray<Vec3f>::create();

	DeviceSyncArray<Mat3x4f>::Ptr dAnimationMatrices = DeviceSyncArray<Mat3x4f>::create();
};
