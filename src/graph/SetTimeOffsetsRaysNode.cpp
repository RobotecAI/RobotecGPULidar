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

void SetTimeOffsetsRaysNode::setParameters(const float* raysTimeOffsetsRaw, size_t raysTimeOffsetsCount)
{
	timeOffsets->setData(raysTimeOffsetsRaw, raysTimeOffsetsCount);
}

void SetTimeOffsetsRaysNode::validate()
{
	input = getValidInput<IRaysNode>();

	if (input->getRayCount() != timeOffsets->getCount()) {
		auto msg = fmt::format("offsets don't match number of rays. "
		                       "RayCount({}) and TimeOffsetsCount({}) should be equal", input->getRayCount(), timeOffsets->getCount());
		throw InvalidPipeline(msg);
	}
}
