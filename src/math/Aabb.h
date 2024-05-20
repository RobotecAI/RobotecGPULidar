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

template<int dim, typename T, typename = std::enable_if_t<(dim > 0) && std::is_arithmetic_v<T>>>
class Aabb
{
public:
	Aabb() = default;
	Aabb(const Vector<dim, T>& center, const Vector<dim, T>& size) : minCorner{center - size / 2}, maxCorner{center + size / 2}
	{}

	Aabb(const Aabb& other) = default;
	Aabb(Aabb&& other) = default;

	Aabb& operator=(const Aabb& other) = default;
	Aabb& operator=(Aabb&& other) = default;

	Vector<dim, T> min() const { return min; }
	Vector<dim, T> max() const { return max; }

	void expand(const Vector<dim, T>& point)
	{
		for (int i = 0; i < dim; ++i) {
			minCorner[i] = std::min(minCorner[i], point[i]);
			maxCorner[i] = std::max(maxCorner[i], point[i]);
		}
	}

	void merge(const Aabb& other)
	{
		expand(other.min);
		expand(other.max);
	}

	void reset()
	{
		minCorner = maxCorner = {0};
	}

private:
	Vector<dim, T> minCorner{0};
	Vector<dim, T> maxCorner{0};
};

using Aabb2Df = Aabb<2, float>;
using Aabb3Df = Aabb<3, float>;
using Aabb4Df = Aabb<4, float>;

using Aabb2Di = Aabb<2, int>;
using Aabb3Di = Aabb<3, int>;
using Aabb4Di = Aabb<4, int>;

static_assert(std::is_trivially_copyable_v<Aabb2Df>);
static_assert(std::is_trivially_copyable_v<Aabb3Df>);
static_assert(std::is_trivially_copyable_v<Aabb4Df>);
static_assert(std::is_standard_layout_v<Aabb2Df>);
static_assert(std::is_standard_layout_v<Aabb3Df>);
static_assert(std::is_standard_layout_v<Aabb4Df>);

static_assert(std::is_trivially_copyable_v<Aabb2Di>);
static_assert(std::is_trivially_copyable_v<Aabb3Di>);
static_assert(std::is_trivially_copyable_v<Aabb4Di>);
static_assert(std::is_standard_layout_v<Aabb2Di>);
static_assert(std::is_standard_layout_v<Aabb3Di>);
static_assert(std::is_standard_layout_v<Aabb4Di>);
