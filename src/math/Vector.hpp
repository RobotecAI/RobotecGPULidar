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

#pragma once

#include <type_traits>
#include <numeric>
#include <iterator>
#include <array>
#include <cmath>

#include <macros/cuda.hpp>
#include <macros/iteration.hpp>

template<int dim, typename T>
struct Vector
{
	using V = Vector<dim, T>;
	// static_assert(dim >= 2); // TODO: Is dim == 1 OK?
	// *** *** *** CONSTRUCTORS *** *** *** //

	// Zero constructor
	HostDevFn Vector() : Vector(static_cast<T>(0)) {}

	// Uniform constructor
	HostDevFn Vector(T scalar) {
		for (auto&& v : row) {
			v = scalar;
		}
	}

	Vector(const std::array<T, dim>& values)
	{ std::copy(values.begin(), values.end(), row); }

	// List constructor
	template<typename... Args>
	HostDevFn Vector(Args... args) : row{static_cast<T>(args)...}
	{ static_assert(sizeof...(Args) == dim); }

	// Type cast constructor
	template<typename U>
	HostDevFn Vector(const Vector<dim, U>& other) {
		for (int i = 0; i < dim; ++i) {
			row[i] = static_cast<T>(other.row[i]);
		}
	}

	// Dimension cast constructor
	template<int lowerDim>
	HostDevFn Vector(const Vector<lowerDim, T>& other, T fillValue)
	{
		static_assert(lowerDim < dim);
		for (int i = 0; i < lowerDim; ++i) {
			row[i] = other[i];
		}
		for (int i = lowerDim; i < dim; ++i) {
			row[i] = fillValue;
		}
	}

	template<typename TT=T, typename = std::enable_if_t<std::is_same_v<TT, float> && dim == 3>>
	DevFn Vector(float3 v) : Vector(v.x, v.y, v.z) {}

	template<typename TT=T, typename = std::enable_if_t<std::is_same_v<TT, float> && dim == 3>>
	DevFn operator float3() { return float3 {row[0], row[1], row[2]}; }

        template<typename TT=T, typename = std::enable_if_t<std::is_same_v<TT, float> && dim == 2>>
        DevFn Vector(float2 v) : Vector(v.x, v.y) {}

        template<typename TT=T, typename = std::enable_if_t<std::is_same_v<TT, float> && dim == 2>>
        DevFn operator float2() { return float2 {row[0], row[1]}; }


        // *** *** *** ACCESSORS *** *** *** //

	FORWARD_ITERATION(row, HostDevFn)
	FORWARD_INDEX_OPERATOR(row, HostDevFn)

#define NAMED_GETTER(name, index) \
HostDevFn T name() { static_assert(dim > index); return row[index]; } \
HostDevFn const T name() const { static_assert(dim > index); return row[index]; }

NAMED_GETTER(x, 0)
NAMED_GETTER(y, 1)
NAMED_GETTER(z, 2)
#undef NAMED_GETTER

// *** *** *** PIECEWISE OPERATORS (VECTOR + SCALAR) *** *** *** //

#define PIECEWISE_OPERATOR(OP, OPEQ)    \
	HostDevFn V& operator OPEQ(const V& rhs) { \
		for (int i = 0; i < dim; ++i) { \
			row[i] OPEQ rhs[i];         \
		}                               \
		return *this;                   \
	}                                   \
	HostDevFn friend V operator OP(V lhs, const V& rhs) { 	lhs OPEQ rhs; return lhs; } \

	PIECEWISE_OPERATOR(+, +=)
	PIECEWISE_OPERATOR(-, -=)
	PIECEWISE_OPERATOR(*, *=)
	PIECEWISE_OPERATOR(/, /=)
#undef PIECEWISE_OPERATOR

	HostDevFn T lengthSquared() const {
		auto sum = static_cast<T>(0);
		for (int i = 0; i < dim; ++i) {
			sum += row[i] * row[i];
		}
		return sum;
	}

	HostDevFn T length() const { return std::sqrt(lengthSquared()); }

	HostDevFn V half() const { return *this / V {static_cast<T>(2)}; }

	HostDevFn V normalize() const { return *this / length(); }

	HostDevFn T min() const {
		T value = std::numeric_limits<T>::max();
		for (int i = 0; i < dim; ++i) {
			value = row[i] < value ? row[i] : value;
		}
		return value;
	}

	HostDevFn T product() const {
		T value = static_cast<T>(1);
		for (auto&& v : *this) {
			value *= v;
		}
		return value;
	}

private:
	T row[dim];
};

#ifndef __CUDACC__
#include <spdlog/fmt/fmt.h>
template<int dim, typename T>
struct fmt::formatter<Vector<dim, T>>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx) {
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(Vector<dim, T> const& vec, FormatContext& ctx) {

		fmt::format_to(ctx.out(), "(");
		for (int i = 0; i < dim; ++i) {
			fmt::format_to(ctx.out(), "{}", vec[i]);
			if (i < dim - 1) {
				fmt::format_to(ctx.out(), ", ");
			}
		}
		return fmt::format_to(ctx.out(), ")");
	}
};
#endif // __CUDACC__

using Vec2f = Vector<2, float>;
using Vec3f = Vector<3, float>;
using Vec4f = Vector<4, float>;

using Vec2i = Vector<2, int>;
using Vec3i = Vector<3, int>;
using Vec4i = Vector<4, int>;

static_assert(std::is_trivially_copyable<Vec2f>::value);
static_assert(std::is_trivially_copyable<Vec3f>::value);
static_assert(std::is_trivially_copyable<Vec4f>::value);

static_assert(std::is_trivially_copyable<Vec2i>::value);
static_assert(std::is_trivially_copyable<Vec3i>::value);
static_assert(std::is_trivially_copyable<Vec4i>::value);
