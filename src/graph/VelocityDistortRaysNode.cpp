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

void VelocityDistortRaysNode::validate()
{
	input = getValidInput<IRaysNode>();

	if (input->getTimeOffsets().has_value()) {
		auto msg = fmt::format("VelocityDistortRaysNOde without SetRaysTimeOffsetNode");
		throw InvalidPipeline(msg);
	}
}

void VelocityDistortRaysNode::schedule(cudaStream_t stream)
{
	auto offsets = input->getTimeOffsets();
	//TODO distort rays on kernel mrozikp
	//gpuTransformRays(stream, getRayCount(), input->getRays()->getDevicePtr(), rays->getDevicePtr(), transform);
}
