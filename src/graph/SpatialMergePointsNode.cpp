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

#include <graph/Nodes.hpp>
#include <RGLFields.hpp>

void SpatialMergePointsNode::setParameters(const std::vector<rgl_field_t>& fields)
{
	if (std::find(fields.begin(), fields.end(), RGL_FIELD_DYNAMIC_FORMAT) != fields.end()) {
		throw InvalidAPIArgument("cannot perform spatial merge on field 'RGL_FIELD_DYNAMIC_FORMAT'");
	}

	mergedData.clear();
	width = 0;

	for (auto&& field : fields) {
		if (!mergedData.contains(field) && !isDummy(field)) {
			mergedData.insert({field, VArray::create(field)});
		}
	}
}

void SpatialMergePointsNode::validate()
{
	pointInputs = Node::filter<IPointsNode>(this->inputs, true);

	if (pointInputs.empty()) {
		auto msg = "expected at least one IPointsNode input for SpatialMergePointsNode";
		throw InvalidPipeline(msg);
	}

	for (const auto& input : pointInputs) {
		if (input->getHeight() != 1) {
			auto msg = "Spatial points merge could be proceed on unorganized pointclouds only";
			throw InvalidPipeline(msg);
		}

		for (const auto& [requiredField, _] : mergedData) {
			if (!input->hasField(requiredField)) {
				auto msg = fmt::format("SpatialMergePointsNode input has not required field '{}'",
				                       toString(requiredField));
				throw InvalidPipeline(msg);
			}
		}
	}
}

void SpatialMergePointsNode::schedule(cudaStream_t stream)
{
	width = 0;
	for (const auto& input : pointInputs) {
		width += input->getWidth();
	}

	for (const auto& [field, data] : mergedData) {
		data->resize(width, false, false);
		size_t offset = 0;
		for (const auto& input : pointInputs) {
			size_t pointCount = input->getPointCount();
			const auto toMergeData = input->getFieldData(field, stream);
			data->insertData(toMergeData->getReadPtr(MemLoc::Device), pointCount, offset);
			offset += pointCount;
		}
	}
}

bool SpatialMergePointsNode::isDense() const
{
	auto notDense = [](IPointsNode::Ptr node) { return node->isDense() == false; };
	if (std::any_of(pointInputs.begin(), pointInputs.end(), notDense)) {
		return false;
	}
	return true;
}
