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

#pragma once

// Integral types here generally make no sense in terms of mean and std dev.
// I would force users to use floats, to eliminate possible errors.
template<typename StatType, typename = std::enable_if_t<!std::is_integral_v<StatType>>>
class RunningStats
{
public:
	static constexpr RunningStats calculateFor(const std::vector<StatType>& range)
	{
		RunningStats stats;
		for (auto&& element : range) {
			stats.addSample(element);
		}
		return stats;
	}

	void addSample(StatType sample)
	{
		++counter;
		StatType delta = sample - mean;
		mean += delta / counter;
		m2 += delta * (sample - mean);
		lastSample = sample;
	}

	StatType getLastSample() const { return lastSample; }

	StatType getMean() const { return mean; }

	StatType getVariance() const { return counter < 2 ? StatType{} : m2 / counter; }

	auto getStdDev() const
	{
		// This if-statement is only for handling floating point numbers.
		if constexpr (std::is_floating_point_v<StatType>) {
			return std::sqrt(getVariance());
		}

		// If-statement below all handle Vector types.
		const auto variance = getVariance();

		if constexpr (std::is_same_v<StatType, Vector<2, float>>) {
			return Vector<2, float>{std::sqrt(variance.x()), std::sqrt(variance.y())};
		}

		if constexpr (std::is_same_v<StatType, Vector<3, float>>) {
			return Vector<3, float>{std::sqrt(variance.x()), std::sqrt(variance.y()), std::sqrt(variance.z())};
		}

		if constexpr (std::is_same_v<StatType, Vector<4, float>>) {
			return Vector<4, float>{std::sqrt(variance[0]), std::sqrt(variance[1]), std::sqrt(variance[2]),
			                        std::sqrt(variance[3])};
		}
	}

	auto getMeanAndStdDev() const { return std::make_pair(getMean(), getStdDev()); }

private:
	size_t counter{0};
	StatType mean{0};
	StatType m2{0};
	StatType lastSample{0};
};
