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

void YieldPointsNode::setParameters(const std::vector<rgl_field_t>& fields)
{
	results.clear();
	if (std::find(fields.begin(), fields.end(), RGL_FIELD_DYNAMIC_FORMAT) != fields.end()) {
		throw InvalidAPIArgument("cannot yield field 'RGL_FIELD_DYNAMIC_FORMAT'"); // TODO: Yeah, but dummies are OK?
	}
	this->fields = fields;
}

void YieldPointsNode::enqueueExecImpl()
{
	for (auto&& field : fields) {
		results[field] = input->getFieldData(field);
	}
	if (results.contains(XYZ_VEC3_F32)) {
		xyzHostCache->resize(results.at(XYZ_VEC3_F32)->getCount(), false, false);
		CHECK_CUDA(cudaMemcpyAsync(xyzHostCache->getWritePtr(), results[XYZ_VEC3_F32]->getRawReadPtr(),
		                           xyzHostCache->getCount() * xyzHostCache->getSizeOf(), cudaMemcpyDefault, getStreamHandle()));
	}
}

IAnyArray::ConstPtr YieldPointsNode::getFieldData(rgl_field_t field)
{
	if (!results.contains(field)) {
		throw std::runtime_error("Field data is not ready yet. It was requested without waiting for results.");
	}
	return results.at(field);
}
