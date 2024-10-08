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

#include <gpu/sceneKernels.hpp>
#include <scene/animator/SkeletonAnimator.hpp>

SkeletonAnimator::SkeletonAnimator(const std::shared_ptr<Mesh>& mesh)
{
	if (!mesh->dBoneWeights.has_value()) {
		throw std::invalid_argument("Cannot create SkeletonAnimator because mesh has no bone weights defined.");
	}

	if (!mesh->dRestposes.has_value()) {
		throw std::invalid_argument("Cannot create SkeletonAnimator because mesh has no restposes defined.");
	}

	this->mesh = mesh;

	dAnimatedVertices->copyFrom(mesh->dVertices);
	dVertexAnimationDisplacement->resize(mesh->dVertices->getCount(), true, false);
	dAnimationMatrices->resize(mesh->dRestposes.value()->getCount(), false, false);
}

void SkeletonAnimator::animate(const Mat3x4f* pose, std::size_t bonesCount)
{
	if (mesh->dRestposes.value()->getCount() != bonesCount) {
		auto msg = fmt::format(
		    "Cannot perform skeleton animation because bones count do not match restposes count: bones={}, restposes={}",
		    bonesCount, mesh->dRestposes.value()->getCount());
		throw std::invalid_argument(msg);
	}

	dAnimationMatrices->copyFromExternal(pose, bonesCount);
	const std::size_t vertexCount = mesh->dVertices->getCount();

	// Write new vertices to dVertexAnimationDisplacement to preserve old vertices in dAnimatedVertices
	gpuPerformSkeletonAnimation(CudaStream::getNullStream()->getHandle(), mesh->dVertices->getCount(), bonesCount,
	                            mesh->dVertices->getReadPtr(), mesh->dBoneWeights->get()->getReadPtr(),
	                            mesh->dRestposes.value()->getReadPtr(), dAnimationMatrices->getWritePtr(),
	                            dVertexAnimationDisplacement->getWritePtr());

	// Update displacements and animation vertices
	// dVertexAnimationDisplacement contains new vertices; dAnimatedVertices - old vertices
	// After this operation, the array's names will match the data they contain
	gpuUpdateVerticesWithDisplacement(CudaStream::getNullStream()->getHandle(), vertexCount,
	                                  dVertexAnimationDisplacement->getWritePtr(), dAnimatedVertices->getWritePtr());
}
