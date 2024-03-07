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

#include <macros/cuda.hpp>
#include <math/floatComparison.hpp>
#include <math/Vector.hpp>
#include <rgl/api/core.h>

struct Mat3x4f;
HostDevFn Mat3x4f operator*(const Mat3x4f& lhs, const Mat3x4f& rhs);

struct Mat3x4f
{
	static constexpr int ROWS = 3;
	static constexpr int COLS = 4;

	float rc[ROWS][COLS];

	static HostDevFn Mat3x4f identity() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}; }

	static HostDevFn inline Mat3x4f scale(float x, float y, float z) { return {x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0}; }

	static HostDevFn inline Mat3x4f scale(Vec3f s) { return Mat3x4f::scale(s.x(), s.y(), s.z()); }

	static HostDevFn inline Mat3x4f rotationRad(float x, float y, float z)
	{
		// Based on https://github.com/microsoft/DirectXMath/blob/main/Inc/DirectXMathMatrix.inl#L1697
		// Instead of creating three matrices and muling, we create subproducts of the matrix, ommiting zeros elements.
		float cr = cosf(x);
		float sr = sinf(x);
		float cp = cosf(y);
		float sp = sinf(y);
		float cy = cosf(z);
		float sy = sinf(z);

		Mat3x4f m = Mat3x4f::identity();
		m.rc[0][0] = cp * cy;
		m.rc[0][1] = sr * sp * cy - cr * sy;
		m.rc[0][2] = sr * sy + cr * sp * cy;
		m.rc[1][0] = cp * sy;
		m.rc[1][1] = cr * cy + sr * sp * sy;
		m.rc[1][2] = cr * sp * sy - sr * cy;
		m.rc[2][0] = -sp;
		m.rc[2][1] = sr * cp;
		m.rc[2][2] = cr * cp;
		return m;
	}

	static HostDevFn inline Mat3x4f rotationRad(Vec3f r) { return Mat3x4f::rotationRad(r.x(), r.y(), r.z()); }

	static HostDevFn inline Mat3x4f rotationRad(rgl_axis_t axis, float angleRad)
	{
		float rotX = axis == RGL_AXIS_X ? angleRad : 0.0f;
		float rotY = axis == RGL_AXIS_Y ? angleRad : 0.0f;
		float rotZ = axis == RGL_AXIS_Z ? angleRad : 0.0f;
		return rotationRad(rotX, rotY, rotZ);
	}

	static HostDevFn inline Mat3x4f rotationDeg(float x, float y, float z)
	{
		return rotationRad(Vec3f{x, y, z} * (static_cast<float>(M_PI) / 180.0f));
	}

	static HostDevFn inline Mat3x4f rotationDeg(Vec3f r) { return Mat3x4f::rotationDeg(r.x(), r.y(), r.z()); }

	static HostDevFn inline Mat3x4f rotationDeg(rgl_axis_t axis, float angleDeg)
	{
		return Mat3x4f::rotationRad(axis, angleDeg * (static_cast<float>(M_PI) / 180.0f));
	}

	static HostDevFn inline Mat3x4f translation(float x, float y, float z) { return {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z}; }

	static HostDevFn inline Mat3x4f translation(const Vec3f point)
	{
		return {1, 0, 0, point.x(), 0, 1, 0, point.y(), 0, 0, 1, point.z()};
	}

	static HostDevFn inline Mat3x4f TRS(Vec3f t, Vec3f r = {0, 0, 0}, Vec3f s = {1, 1, 1})
	{
		auto T = Mat3x4f::translation(t.x(), t.y(), t.z());
		auto R = Mat3x4f::rotationDeg(r.x(), r.y(), r.z());
		auto S = Mat3x4f::scale(s.x(), s.y(), s.z());
		return T * R * S;
	}

	static HostDevFn inline Mat3x4f shear(Vec2f x, Vec2f y = {0, 0}, Vec2f z = {0, 0})
	{
		return {1, y[0], z[0], 0, x[0], 1, z[1], 0, x[1], y[1], 1, 0};
	}

	static inline Mat3x4f fromRaw(const float* data)
	{
		Mat3x4f matrix{};
		memcpy(matrix.rc, data, 12 * sizeof(float));
		return matrix;
	}

	inline void toRaw(float* dst) const { memcpy(dst, rc, 12 * sizeof(float)); }

	static inline Mat3x4f fromRGL(const rgl_mat3x4f& m)
	{
		return {m.value[0][0], m.value[0][1], m.value[0][2], m.value[0][3], m.value[1][0], m.value[1][1],
		        m.value[1][2], m.value[1][3], m.value[2][0], m.value[2][1], m.value[2][2], m.value[2][3]};
	}

	inline rgl_mat3x4f toRGL() const
	{
		return {
		    rc[0][0], rc[0][1], rc[0][2], rc[0][3], rc[1][0], rc[1][1],
		    rc[1][2], rc[1][3], rc[2][0], rc[2][1], rc[2][2], rc[2][3],
		};
	}

	inline bool operator==(const Mat3x4f& other) const
	{
		for (int y = 0; y < ROWS; ++y) {
			for (int x = 0; x < COLS; ++x) {
				if (!approximatelyEqual(rc[y][x], other.rc[y][x], 1e-6f)) {
					return false;
				}
			}
		}
		return true;
	}

	HostDevFn inline Vec3f translation() const { return {rc[0][3], rc[1][3], rc[2][3]}; }

	HostDevFn inline Mat3x4f rotation() const
	{
		return {rc[0][0], rc[0][1], rc[0][2], 0.0f, rc[1][0], rc[1][1], rc[1][2], 0.0f, rc[2][0], rc[2][1], rc[2][2], 0.0f};
	}

	HostDevFn inline float toRotationXRad() const { return atan2f(rc[2][1], rc[2][2]); }

	HostDevFn inline float toRotationYRad() const { return atan2f(-rc[2][0], sqrtf(powf(rc[2][1], 2) + powf(rc[2][2], 2))); }

	HostDevFn inline float toRotationZRad() const { return atan2f(rc[1][0], rc[0][0]); }

	HostDevFn inline Vec3f toRotationRad() const { return {toRotationXRad(), toRotationYRad(), toRotationZRad()}; }

	HostDevFn inline Vec3f scaleVec() const
	{
		return {
		    sqrtf(rc[0][0] * rc[0][0] + rc[1][0] * rc[1][0] + rc[2][0] * rc[2][0]),
		    sqrtf(rc[0][1] * rc[0][1] + rc[1][1] * rc[1][1] + rc[2][1] * rc[2][1]),
		    sqrtf(rc[0][2] * rc[0][2] + rc[1][2] * rc[1][2] + rc[2][2] * rc[2][2]),
		};
	}

	// Converts to Matrix 4x4 and performs inverse operation.
	// If determinant is zero (cannot inverse) it returns Matrix filled with zeros.
	HostDevFn inline Mat3x4f inverse() const noexcept
	{
		// Convert to 4x4
		float m[4][4] = {rc[0][0], rc[0][1], rc[0][2], rc[0][3], rc[1][0], rc[1][1], rc[1][2], rc[1][3],
		                 rc[2][0], rc[2][1], rc[2][2], rc[2][3], 0,        0,        0,        1};
		// Based on https://stackoverflow.com/a/60374938
		float A2323 = m[2][2] * m[3][3] - m[2][3] * m[3][2];
		float A1323 = m[2][1] * m[3][3] - m[2][3] * m[3][1];
		float A1223 = m[2][1] * m[3][2] - m[2][2] * m[3][1];
		float A0323 = m[2][0] * m[3][3] - m[2][3] * m[3][0];
		float A0223 = m[2][0] * m[3][2] - m[2][2] * m[3][0];
		float A0123 = m[2][0] * m[3][1] - m[2][1] * m[3][0];
		float A2313 = m[1][2] * m[3][3] - m[1][3] * m[3][2];
		float A1313 = m[1][1] * m[3][3] - m[1][3] * m[3][1];
		float A1213 = m[1][1] * m[3][2] - m[1][2] * m[3][1];
		float A2312 = m[1][2] * m[2][3] - m[1][3] * m[2][2];
		float A1312 = m[1][1] * m[2][3] - m[1][3] * m[2][1];
		float A1212 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
		float A0313 = m[1][0] * m[3][3] - m[1][3] * m[3][0];
		float A0213 = m[1][0] * m[3][2] - m[1][2] * m[3][0];
		float A0312 = m[1][0] * m[2][3] - m[1][3] * m[2][0];
		float A0212 = m[1][0] * m[2][2] - m[1][2] * m[2][0];
		float A0113 = m[1][0] * m[3][1] - m[1][1] * m[3][0];
		float A0112 = m[1][0] * m[2][1] - m[1][1] * m[2][0];

		float det = m[0][0] * (m[1][1] * A2323 - m[1][2] * A1323 + m[1][3] * A1223) -
		            m[0][1] * (m[1][0] * A2323 - m[1][2] * A0323 + m[1][3] * A0223) +
		            m[0][2] * (m[1][0] * A1323 - m[1][1] * A0323 + m[1][3] * A0123) -
		            m[0][3] * (m[1][0] * A1223 - m[1][1] * A0223 + m[1][2] * A0123);

		if (det == 0.0f) {
			return {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		}
		float idet = 1.0f / det;

		float im[4][4];
		im[0][0] = idet * (m[1][1] * A2323 - m[1][2] * A1323 + m[1][3] * A1223);
		im[0][1] = idet * -(m[0][1] * A2323 - m[0][2] * A1323 + m[0][3] * A1223);
		im[0][2] = idet * (m[0][1] * A2313 - m[0][2] * A1313 + m[0][3] * A1213);
		im[0][3] = idet * -(m[0][1] * A2312 - m[0][2] * A1312 + m[0][3] * A1212);
		im[1][0] = idet * -(m[1][0] * A2323 - m[1][2] * A0323 + m[1][3] * A0223);
		im[1][1] = idet * (m[0][0] * A2323 - m[0][2] * A0323 + m[0][3] * A0223);
		im[1][2] = idet * -(m[0][0] * A2313 - m[0][2] * A0313 + m[0][3] * A0213);
		im[1][3] = idet * (m[0][0] * A2312 - m[0][2] * A0312 + m[0][3] * A0212);
		im[2][0] = idet * (m[1][0] * A1323 - m[1][1] * A0323 + m[1][3] * A0123);
		im[2][1] = idet * -(m[0][0] * A1323 - m[0][1] * A0323 + m[0][3] * A0123);
		im[2][2] = idet * (m[0][0] * A1313 - m[0][1] * A0313 + m[0][3] * A0113);
		im[2][3] = idet * -(m[0][0] * A1312 - m[0][1] * A0312 + m[0][3] * A0112);
		im[3][0] = idet * -(m[1][0] * A1223 - m[1][1] * A0223 + m[1][2] * A0123);
		im[3][1] = idet * (m[0][0] * A1223 - m[0][1] * A0223 + m[0][2] * A0123);
		im[3][2] = idet * -(m[0][0] * A1213 - m[0][1] * A0213 + m[0][2] * A0113);
		im[3][3] = idet * (m[0][0] * A1212 - m[0][1] * A0212 + m[0][2] * A0112);

		return {
		    im[0][0], im[0][1], im[0][2], im[0][3], im[1][0], im[1][1],
		    im[1][2], im[1][3], im[2][0], im[2][1], im[2][2], im[2][3],
		};
	}

	inline Mat3x4f& operator=(const Mat3x4f& other) = default;

	HostDevFn float& operator[](int i) { return rc[i / 4][i % 4]; }
	HostDevFn const float& operator[](int i) const { return rc[i / 4][i % 4]; }
};

HostDevFn inline Mat3x4f operator*(const Mat3x4f& lhs, const Mat3x4f& rhs)
{
#define MUL(y, x) ((lhs.rc[y][0] * rhs.rc[0][x]) + (lhs.rc[y][1] * rhs.rc[1][x]) + (lhs.rc[y][2] * rhs.rc[2][x]))
	return {MUL(0, 0), MUL(0, 1), MUL(0, 2), MUL(0, 3) + lhs.rc[0][3],
	        MUL(1, 0), MUL(1, 1), MUL(1, 2), MUL(1, 3) + lhs.rc[1][3],
	        MUL(2, 0), MUL(2, 1), MUL(2, 2), MUL(2, 3) + lhs.rc[2][3]};
#undef MUL
}

HostDevFn inline Vec3f operator*(const Mat3x4f& lhs, const Vec3f& rhs)
{
#define MUL(i) ((lhs.rc[i][0] * rhs[0]) + (lhs.rc[i][1] * rhs[1]) + (lhs.rc[i][2] * rhs[2]))
	return {MUL(0) + lhs.rc[0][3], MUL(1) + lhs.rc[1][3], MUL(2) + lhs.rc[2][3]};
#undef MUL
}


#ifndef __CUDACC__
#include <spdlog/fmt/fmt.h>
template<>
struct fmt::formatter<Mat3x4f>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx)
	{
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(Mat3x4f const& m, FormatContext& ctx)
	{

		for (int y = 0; y < Mat3x4f::ROWS; ++y) {
			for (int x = 0; x < Mat3x4f::COLS; ++x) {
				fmt::format_to(ctx.out(), "{:.4f} ", m.rc[y][x]);
			}
			fmt::format_to(ctx.out(), "\n");
		}
		return ctx.out();
	}
};
#endif // __CUDACC__

static_assert(std::is_trivially_copyable_v<Mat3x4f>);
static_assert(std::is_standard_layout_v<Mat3x4f>);
static_assert(sizeof(Mat3x4f) == 12 * sizeof(float));
static_assert(alignof(Mat3x4f) == 4);
