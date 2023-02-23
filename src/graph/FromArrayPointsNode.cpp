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

#include <graph/NodesCore.hpp>
#include <gpu/nodeKernels.hpp>
#include <RGLFields.hpp>

void FromArrayPointsNode::setParameters(const void* points, size_t pointCount, const std::vector<rgl_field_t>& fields)
{
	// TODO(msz-rai): Optimize memory allocation. Do not clear all fields.
	fieldData.clear();
	width = pointCount;

	for (auto&& field : fields) {
		if (!fieldData.contains(field) && !isDummy(field)) {
			fieldData.insert({field, VArray::create(field, pointCount)});
		}
	}

	VArray::Ptr inputData = VArray::create<char>();
	inputData->setData(static_cast<const char*>(points), pointCount * getPointSize(fields));

	std::size_t pointSize = getPointSize(fields);
	auto gpuFields = GPUFieldDescBuilder::buildWritable(std::dynamic_pointer_cast<IPointsSourceNode>(shared_from_this()), fields, nullptr);
	const char* inputPtr = static_cast<const char*>(inputData->getReadPtr(MemLoc::Device));
	gpuFormatAosToSoa(nullptr, pointCount, pointSize, fields.size(), inputPtr, gpuFields->getDevicePtr());
	CHECK_CUDA(cudaStreamSynchronize(nullptr));  // May not be required, but it is safer
}

void FromArrayPointsNode::validate()
{
	if (!inputs.empty()) {
		auto msg = fmt::format("inputs for node {} are not allowed", getName());
		throw InvalidPipeline(msg);
	}
}
