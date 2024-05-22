// Copyright 2024 Robotec.AI
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

#include "Vector.hpp"

template<template<int, typename> class Base, class Checked>
struct is_specialization_of : std::false_type
{};

template<template<int, typename> class Base, int dim, typename T>
struct is_specialization_of<Base, Base<dim, T>> : std::true_type
{};

template<typename>
inline constexpr bool is_floating_point_vector_v = false;

template<template<int, typename> class Base, int dim, typename T>
inline constexpr bool is_floating_point_vector_v<Base<dim, T>> = std::is_floating_point_v<T> &&
                                                                 is_specialization_of<Base, Vector<dim, T>>::value;

// Integral types here generally make no sense in terms of mean and std dev.
// Floating point types and any Vector specializations using floating point types are acceptable.
template<typename T, typename = std::enable_if_t<std::is_floating_point_v<T> || is_floating_point_vector_v<T>>>
class RunningStats
{
public:
	using StatType = T;

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
		lastSample = std::move(sample);
		++counter;

		const auto delta = lastSample - mean;
		mean += delta / counter;
		m2 += delta * (lastSample - mean);
	}

	size_t getSamplesCount() const { return counter; }

	StatType getLastSample() const { return lastSample; }

	StatType getMean() const { return mean; }

	StatType getVariance() const { return counter < 2 ? StatType{} : m2 / counter; }

	StatType getStdDev() const { return std::sqrt(getVariance()); }

	auto getMeanAndStdDev() const { return std::make_pair(getMean(), getStdDev()); }

private:
	StatType lastSample{0};
	size_t counter{0};

	StatType mean{0};
	StatType m2{0};
};

// RunningStats specialization targeting Vector types. This is important because of the details in addSample method and additional methods provided (covariance).)
template<int dim, typename T>
class RunningStats<Vector<dim, T>>
{
public:
	using StatType = Vector<dim, T>;

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
		lastSample = std::move(sample);
		++counter;

		const auto prevDelta = lastSample - mean;
		mean += prevDelta / counter;
		const auto currentDelta = lastSample - mean;

		m2 += prevDelta * currentDelta;

		if constexpr (dim >= 2) {
			sumCov[0] = sumCov.x() + prevDelta.x() * currentDelta.y(); // covariance xy
		}

		if constexpr (dim >= 3) {
			sumCov[1] = sumCov.y() + prevDelta.y() * currentDelta.z(); // covariance yz
			sumCov[2] = sumCov.z() + prevDelta.z() * currentDelta.x(); // covariance zx
		}
	}

	size_t getSamplesCount() const { return counter; }

	const StatType& getLastSample() const { return lastSample; }

	StatType getMean() const { return mean; }

	StatType getVariance() const { return counter < 2 ? StatType{0} : m2 / counter; }

	template<int subDim = dim, typename = std::enable_if_t<subDim >= 2>>
	T getCovarianceXY() const
	{
		return counter < 2 ? T{0} : sumCov.x() / counter;
	}

	template<int subDim = dim, typename = std::enable_if_t<subDim >= 3>>
	T getCovarianceYZ() const
	{
		return counter < 2 ? T{0} : sumCov.y() / counter;
	}

	template<int subDim = dim, typename = std::enable_if_t<subDim >= 3>>
	T getCovarianceZX() const
	{
		return counter < 2 ? T{0} : sumCov.z() / counter;
	}

	StatType getStdDev() const
	{
		const auto variance = getVariance();

		if constexpr (dim == 2) {
			return {std::sqrt(variance.x()), std::sqrt(variance.y())};
		}

		if constexpr (dim == 3) {
			return {std::sqrt(variance.x()), std::sqrt(variance.y()), std::sqrt(variance.z())};
		}

		if constexpr (dim == 4) {
			return {std::sqrt(variance[0]), std::sqrt(variance[1]), std::sqrt(variance[2]), std::sqrt(variance[3])};
		}
	}

	auto getMeanAndStdDev() const { return std::make_pair(getMean(), getStdDev()); }

private:
	StatType lastSample{0};
	size_t counter{0};

	StatType mean{0};
	StatType m2{0};
	StatType sumCov{0}; //< Sum for calculating covariance.
};
