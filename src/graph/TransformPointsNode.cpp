// Copyright 2022 Robotec.AI
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

#include <graph/NodesCore.hpp>
#include <gpu/nodeKernels.hpp>

void TransformPointsNode::enqueueExecImpl(cudaStream_t stream)
{
	auto pointCount = input->getWidth() * input->getHeight();
	output->resize(pointCount);
	const auto inputField = input->getFieldDataTyped<XYZ_F32>(stream);
	const auto* inputPtr = inputField->getDevicePtr();
	auto* outputPtr = output->getDevicePtr();
	gpuTransformPoints(stream, pointCount, inputPtr, outputPtr, transform);
}

VArray::ConstPtr TransformPointsNode::getFieldData(rgl_field_t field, cudaStream_t stream)
{
	if (field == XYZ_F32) {
		// TODO(prybicki): check synchronize is necessary
		CHECK_CUDA(cudaStreamSynchronize(stream));
		return output->untyped();
	}
	return input->getFieldData(field, stream);
}


std::string TransformPointsNode::getArgsString() const
{
	return fmt::format("TR={}", transform.translation());
}
