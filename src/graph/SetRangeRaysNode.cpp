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

void SetRangeRaysNode::setParameters(const Vec2f* rangesRaw, size_t rangesCount)
{
	// Validate ranges
	for (int i = 0; i < rangesCount; ++i) {
		float minRange = rangesRaw[i][0];
		float maxRange = rangesRaw[i][1];
		if (std::isnan(minRange) || std::isnan(maxRange)) {
			throw InvalidAPIArgument("range must be a number");
		}
		if (minRange > maxRange) {
			throw InvalidAPIArgument("minimum range must be lower or equal to maximum range");
		}
		if (minRange < 0.0f) {
			throw InvalidAPIArgument("minimum range must not be negative");
		}
	}

	ranges->copyFromExternal(rangesRaw, rangesCount);
}

void SetRangeRaysNode::validateImpl()
{
        IRaysNodeSingleInput::validateImpl();

	if (!(ranges->getCount() == 1 || ranges->getCount() == input->getRayCount())) {
		auto msg = fmt::format("ranges doesn't match number of rays. "
							   "RangesCount({}) should be one or equal to RayCount({})",
							   ranges->getCount(), input->getRayCount());
		throw InvalidPipeline(msg);
	}
}
