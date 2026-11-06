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

	HostDevFn inline float toRotationXOrderZXYLeftHandRad() const
	{
		// Assuming rotation has been applied in the order: z x y in left-handed coordinate system
		// Based on: https://www.geometrictools.com/Documentation/EulerAngles.pdf
		// Taking transpose of the rotation matrix, because X axis facing opposite direction in left-handed coordinate system
		if (rc[1][2] < 1) {
			if (rc[1][2] > -1) {
				return asinf(rc[1][2]);
			} else {
				return -static_cast<float>(M_PI) / 2.0f;
			}
		} else {
			return static_cast<float>(M_PI) / 2.0f;
		}
	}

	HostDevFn inline float toRotationYOrderZXYLeftHandRad() const
	{
		// Assuming rotation has been applied in the order: z x y in left-handed coordinate system
		// Based on: https://www.geometrictools.com/Documentation/EulerAngles.pdf
		if (rc[2][1] < 1) {
			if (rc[2][1] > -1) {
				return atan2f(-rc[2][0], rc[2][2]);
			} else {
				return 0.0f;
			}
		} else {
			return 0.0f;
		}
	}

	HostDevFn inline Vec3f scaleVec() const
	{
		return {
		    sqrtf(rc[0][0] * rc[0][0] + rc[1][0] * rc[1][0] + rc[2][0] * rc[2][0]),
		    sqrtf(rc[0][1] * rc[0][1] + rc[1][1] * rc[1][1] + rc[2][1] * rc[2][1]),
		    sqrtf(rc[0][2] * rc[0][2] + rc[1][2] * rc[1][2] + rc[2][2] * rc[2][2]),
		};
	}

	// Inverse of the affine transform [R | t]: [R^-1 | -R^-1 * t].
	// R^-1 is the adjugate of R divided by its determinant. If R is singular, it returns Matrix filled with zeros.
	HostDevFn inline Mat3x4f inverse() const noexcept
	{
		// Cofactor of R at (row, col); the cyclic index order gives the cofactor signs of a 3x3 matrix.
		auto cofactor = [this](int row, int col) {
			const int r1 = (row + 1) % 3, r2 = (row + 2) % 3;
			const int c1 = (col + 1) % 3, c2 = (col + 2) % 3;
			return rc[r1][c1] * rc[r2][c2] - rc[r1][c2] * rc[r2][c1];
		};

		const float det = rc[0][0] * cofactor(0, 0) + rc[0][1] * cofactor(0, 1) + rc[0][2] * cofactor(0, 2);
		if (det == 0.0f) {
			return {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		}
		const float invDet = 1.0f / det;

		Mat3x4f inv{};
		for (int row = 0; row < 3; ++row) {
			for (int col = 0; col < 3; ++col) {
				inv.rc[row][col] = cofactor(col, row) * invDet; // adjugate = transposed cofactors
			}
		}
		// The translation from the unscaled adjugate, divided once: scaling first can overflow for a nearly
		// singular R with a large t although the result is finite.
		for (int row = 0; row < 3; ++row) {
			const float adjT = cofactor(0, row) * rc[0][3] + cofactor(1, row) * rc[1][3] + cofactor(2, row) * rc[2][3];
			inv.rc[row][3] = -adjT * invDet;
		}
		return inv;
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
