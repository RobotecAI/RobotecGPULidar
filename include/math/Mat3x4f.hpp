#pragma once

#ifndef __host__
#define __host__
#endif

#ifndef __device__
#define __device__
#endif

#include <math/floatComparison.hpp>
#include <rgl/api/core.h>
#include <math/Vector.hpp>

#ifndef __CUDACC__
#include <spdlog/fmt/fmt.h>
#endif

struct Mat3x4f;
__host__ __device__ Mat3x4f operator*(const Mat3x4f& lhs, const Mat3x4f& rhs);

struct Mat3x4f
{
	static constexpr int ROWS = 3;
	static constexpr int COLS = 4;

	float rc[ROWS][COLS];

	static Mat3x4f identity()
	{
		return {.rc = {
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0
		}};
	}

	static inline Mat3x4f scale(float x, float y, float z)
	{
		return { .rc = {
			x, 0, 0, 0,
			0, y, 0, 0,
			0, 0, z, 0
		}};
	}

	static inline Mat3x4f rotationRad(float x, float y, float z)
	{
		// https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
		Mat3x4f rx = { .rc = {
			1,       0,        0, 0,
			0, cosf(x), -sinf(x), 0,
			0, sinf(x),  cosf(x), 0
		}};
		Mat3x4f ry = { .rc = {
			 cosf(y), 0, sinf(y), 0,
			       0, 1,       0, 0,
			-sinf(y), 0, cosf(y), 0
		}};
		Mat3x4f rz = { .rc = {
			cosf(z), -sinf(z), 0, 0,
			sinf(z),  cosf(z), 0, 0,
			      0,        0, 1, 0
		}};
		return rz * ry * rx;
	}

	static inline Mat3x4f rotation(float x, float y, float z)
	{
		float toRad = (M_PI / 180.0f);
		return rotationRad(x * toRad, y * toRad, z * toRad);
	}

	static inline Mat3x4f translation(float x, float y, float z)
	{
		return {.rc = {
			1, 0, 0, x,
			0, 1, 0, y,
			0, 0, 1, z
		}};
	}

	static inline Mat3x4f TRS(Vec3f t, Vec3f r={0, 0, 0}, Vec3f s={1, 1, 1})
	{
		auto T = Mat3x4f::translation(t.x(), t.y(), t.z());
		auto R = Mat3x4f::rotation(r.x(), r.y(), r.z());
		auto S = Mat3x4f::scale(s.x(), s.y(), s.z());
		return T * R * S;
	}

	static inline Mat3x4f shear(Vec2f x, Vec2f y={0, 0}, Vec2f z={0,0})
	{
		return {.rc = {
			   1, y[0], z[0], 0,
			x[0],    1, z[1], 0,
			x[1], y[1],    1, 0
		}};
	}

	static inline Mat3x4f fromRaw(const float* data)
	{
		Mat3x4f matrix {};
		memcpy(matrix.rc, data, 12 * sizeof(float));
		return matrix;
	}

	inline void toRaw(float* dst)
	{
		memcpy(dst, rc, 12 * sizeof(float));
	}

	static inline Mat3x4f fromRGL(const rgl_mat3x4f& m)
	{
		return { .rc = {
			m.value[0][0], m.value[0][1], m.value[0][2], m.value[0][3],
			m.value[1][0], m.value[1][1], m.value[1][2], m.value[1][3],
			m.value[2][0], m.value[2][1], m.value[2][2], m.value[2][3],
		}};
	}

	inline rgl_mat3x4f toRGL()
	{
		return { .value = {
			rc[0][0], rc[0][1], rc[0][2], rc[0][3],
			rc[1][0], rc[1][1], rc[1][2], rc[1][3],
			rc[2][0], rc[2][1], rc[2][2], rc[2][3],
		}};
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

	__host__ __device__ inline Vec3f translation()
	{
		return {rc[0][3], rc[1][3], rc[2][3]};
	}

	__host__ __device__ inline Mat3x4f rotation()
	{
		return { .rc = {
			rc[0][0], rc[0][1], rc[0][2], 0.0f,
			rc[1][0], rc[1][1], rc[1][2], 0.0f,
			rc[2][0], rc[2][1], rc[2][2], 0.0f,
		}};
	}

	inline Mat3x4f& operator=(const Mat3x4f& other) = default;

	__host__ __device__ float& operator[](int i) {return rc[i/4][i%4];}
	__host__ __device__ const float& operator[](int i) const {return rc[i/4][i%4];}
};

__host__ __device__ inline Mat3x4f operator*(const Mat3x4f& lhs, const Mat3x4f& rhs)
{
#define MUL(y, x) ((lhs.rc[y][0] * rhs.rc[0][x]) + (lhs.rc[y][1] * rhs.rc[1][x]) + (lhs.rc[y][2] * rhs.rc[2][x]))
	return {.rc = {
	MUL(0, 0), MUL(0, 1), MUL(0, 2), MUL(0, 3) + lhs.rc[0][3],
	MUL(1, 0), MUL(1, 1), MUL(1, 2), MUL(1, 3) + lhs.rc[1][3],
	MUL(2, 0), MUL(2, 1), MUL(2, 2), MUL(2, 3) + lhs.rc[2][3],
	}};
#undef MUL
}

__host__ __device__ inline Vec3f operator*(const Mat3x4f& lhs, const Vec3f& rhs)
{
#define MUL(i) ((lhs.rc[i][0] * rhs[0]) + (lhs.rc[i][1] * rhs[1]) + (lhs.rc[i][2] * rhs[2]))
	return {
	MUL(0) + lhs.rc[0][3],
	MUL(1) + lhs.rc[1][3],
	MUL(2) + lhs.rc[2][3]
	};
#undef MUL
}


#ifndef __CUDACC__
#include <spdlog/fmt/fmt.h>
template<>
struct fmt::formatter<Mat3x4f>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx) {
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(Mat3x4f const& m, FormatContext& ctx) {

		for (int y = 0; y < Mat3x4f::ROWS; ++y) {
			for (int x = 0; x < Mat3x4f::COLS; ++x) {
				fmt::format_to(ctx.out(), "{:4} ", m.rc[y][x]);
			}
			fmt::format_to(ctx.out(), "\n");
		}
		return ctx.out();
	}
};
#endif // __CUDACC__

static_assert(std::is_trivially_copyable<Mat3x4f>::value);
static_assert(std::is_trivially_constructible<Mat3x4f>::value);
static_assert(sizeof(Mat3x4f) == 12 * sizeof(float));
static_assert(alignof(Mat3x4f) == 4);
