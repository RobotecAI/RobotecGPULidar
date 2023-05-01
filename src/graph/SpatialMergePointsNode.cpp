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

void SpatialMergePointsNode::setParameters(const std::vector<rgl_field_t>& fields)
{
	if (std::any_of(fields.begin(), fields.end(), [](rgl_field_t field){ return field == RGL_FIELD_DYNAMIC_FORMAT; })) {
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

void SpatialMergePointsNode::validateImpl()
{
	pointInputs = Node::getNodesOfType<IPointsNode>(this->inputs);

	if (pointInputs.size() < 2) {
		// Having just one input does not make sense, because it is equivalent to removing this Node.
		auto msg = fmt::format("{} requires >= 2 input nodes, found {}", getName(), pointInputs.size());
		throw InvalidPipeline(msg);
	}

	for (const auto& input : pointInputs) {
		// Check input pointcloud is unorganized
		if (input->getHeight() != 1) {
			auto msg = "Spatial points merge could be proceed on unorganized pointclouds only";
			throw InvalidPipeline(msg);
		}

		// Check input pointcloud has required fields
		for (const auto& requiredField : getRequiredFieldList()) {
			if (!input->hasField(requiredField)) {
				auto msg = fmt::format("{} input does not have required field '{}'", getName(), toString(requiredField));
				throw InvalidPipeline(msg);
			}
		}
	}
}

void SpatialMergePointsNode::enqueueExecImpl()
{
	width = 0;
	for (const auto& input : pointInputs) {
		width += input->getWidth();
	}

	// This could work lazily - merging only on demand
	for (const auto& [field, data] : mergedData) {
		data->resize(width, false, false);
		size_t offset = 0;
		for (const auto& input : pointInputs) {
			size_t pointCount = input->getPointCount();
			const auto toMergeData = input->getFieldData(field);
			data->insertData(toMergeData->getReadPtr(MemLoc::Device), pointCount, offset);
			offset += pointCount;
		}
	}
}

bool SpatialMergePointsNode::isDense() const
{
	return std::all_of(pointInputs.begin(), pointInputs.end(),
	                   [](IPointsNode::Ptr node) { return node->isDense(); });
}
