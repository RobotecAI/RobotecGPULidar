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
#include <graph/NodesPcl.hpp>
#include <gpu/nodeKernels.hpp>

#include <pcl/impl/point_types.hpp>

using PCLPoint = pcl::PointXYZL;

void RemoveGroundPointsNode::validateImpl() { IPointsNodeSingleInput::validateImpl(); }

void RemoveGroundPointsNode::enqueueExecImpl() {}

size_t RemoveGroundPointsNode::getWidth() const { return input->getWidth(); }

IAnyArray::ConstPtr RemoveGroundPointsNode::getFieldData(rgl_field_t field) { return input->getFieldData(field); }

std::vector<rgl_field_t> RemoveGroundPointsNode::getRequiredFieldList() const
{
	// pcl::PointXYZL is aligned to 32 bytes for SSE2 ¯\_(ツ)_/¯
	return {XYZ_VEC3_F32, PADDING_32, PADDING_32, PADDING_32, PADDING_32, PADDING_32};
}
