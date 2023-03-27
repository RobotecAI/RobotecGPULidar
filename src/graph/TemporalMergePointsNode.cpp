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
#include <RGLFields.hpp>

void TemporalMergePointsNode::setParameters(const std::vector<rgl_field_t>& fields)
{
	if (std::any_of(fields.begin(), fields.end(), [](rgl_field_t field){ return field == RGL_FIELD_DYNAMIC_FORMAT; })) {
		throw InvalidAPIArgument("cannot perform temporal merge on field 'RGL_FIELD_DYNAMIC_FORMAT'");
	}

	mergedData.clear();
	width = 0;

	for (auto&& field : fields) {
		if (!mergedData.contains(field) && !isDummy(field)) {
			mergedData.insert({field, VArray::create(field)});

			// Make VArray reside on MemLoc::Host
			mergedData.at(field)->getWritePtr(MemLoc::Host);
		}
	}
}

void TemporalMergePointsNode::onInputChangeImpl()
{
	IPointsNodeSingleInput::onInputChangeImpl();

	// Check input pointcloud is unorganized
	if (input->getHeight() != 1) {
		auto msg = "Temporal points merge can process unorganized point clouds only";
		throw InvalidPipeline(msg);
	}
}

void TemporalMergePointsNode::executeImpl(cudaStream_t stream)
{
	// This could work lazily - merging only on demand
	for (const auto& [field, data] : mergedData) {
		size_t pointCount = input->getPointCount();
		const auto toMergeData = input->getFieldData(field, stream);
		data->insertData(toMergeData->getReadPtr(MemLoc::Device), pointCount, width);
	}
	width += input->getWidth();
}
