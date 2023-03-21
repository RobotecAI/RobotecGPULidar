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

void SetRingIdsRaysNode::setParameters(const int* ringIdsRaw, size_t ringIdsCount)
{
	ringIds->setData(ringIdsRaw, ringIdsCount);
}

void SetRingIdsRaysNode::onInputChange()
{
	IRaysNodeSingleInput::onInputChange();

	if (input->getRayCount() % ringIds->getCount() != 0) {
		auto msg = fmt::format("ring ids doesn't match number of rays. "
		    "RayCount({}) mod RingIdsCount({}) should be zero", input->getRayCount(), ringIds->getCount());
		throw InvalidPipeline(msg);
	}
}
