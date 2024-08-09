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
#include <scene/animator/ExternalAnimator.hpp>

ExternalAnimator::ExternalAnimator(DeviceSyncArray<Vec3f>::Ptr originalVertices)
{
	dAnimatedVertices->copyFrom(originalVertices);
	dVertexAnimationDisplacement->resize(originalVertices->getCount(), false, false);
}

void ExternalAnimator::animate(const Vec3f* vertices, std::size_t vertexCount)
{
	if (dAnimatedVertices->getCount() != vertexCount) {
		auto msg = fmt::format(
		    "Invalid argument: cannot apply external animation because vertex counts do not match: old={}, new={}",
		    dAnimatedVertices->getCount(), vertexCount);
		throw std::invalid_argument(msg);
	}

	// Use dVertexSkinningDisplacement as a buffer
	dVertexAnimationDisplacement->copyFromExternal(vertices, vertexCount);

	// Update both displacements and vertices in a single kernel
	gpuUpdateVertices(CudaStream::getNullStream()->getHandle(), vertexCount, dVertexAnimationDisplacement->getWritePtr(),
	                  dAnimatedVertices->getWritePtr());
	CHECK_CUDA(cudaStreamSynchronize(CudaStream::getNullStream()->getHandle()));
}
