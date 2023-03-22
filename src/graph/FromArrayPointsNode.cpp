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

void FromArrayPointsNode::setParameters(const void* points, size_t pointCount, const std::vector<rgl_field_t>& fields)
{
	if (std::find(fields.begin(), fields.end(), RGL_FIELD_DYNAMIC_FORMAT) != fields.end()) {
		throw InvalidAPIArgument("cannot create point cloud from field 'RGL_FIELD_DYNAMIC_FORMAT'");
	}

	// TODO(msz-rai): Optimize memory allocation. Do not clear all fields.
	fieldData.clear();
	width = pointCount;

	for (auto&& field : fields) {
		if (!fieldData.contains(field) && !isDummy(field)) {
			fieldData.insert({field, VArray::create(field, pointCount)});
		}
	}

	auto inputData = VArray::create<char>();
	inputData->setData(static_cast<const char*>(points), pointCount * getPointSize(fields));

	std::size_t pointSize = getPointSize(fields);
	auto gpuFields = GPUFieldDescBuilder::buildWritable(collectFieldRawData(fields));
	const char* inputPtr = static_cast<const char*>(inputData->getReadPtr(MemLoc::Device));
	gpuFormatAosToSoa(nullptr, pointCount, pointSize, fields.size(), inputPtr, gpuFields->getDevicePtr());
	// TODO(msz-rai): check sync is necessary
	CHECK_CUDA(cudaStreamSynchronize(nullptr));
}

std::vector<std::pair<rgl_field_t, void*>> FromArrayPointsNode::collectFieldRawData(const std::vector<rgl_field_t>& fields)
{
	std::vector<std::pair<rgl_field_t, void*>> outFieldsData;
	for (auto&& field : fields) {
		outFieldsData.push_back({field, isDummy(field) ? nullptr : fieldData[field]->getWritePtr(MemLoc::Device)});
	}
	return outFieldsData;
}
