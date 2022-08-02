#pragma once

#ifndef __host__
#define __host__
#endif

#ifndef __device__
#define __device__
#endif


struct Mat3x4f
{
	float rc[3][4];

	static Mat3x4f identity()
	{
		return {.rc = {
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0
		}};
	}

	static Mat3x4f rotation(float x, float y, float z)
	{
		Mat3x4f rx = {.rc = {
			1,      0,       0, 0,
			0, cos(x), -sin(x), 0,
			0, sin(x),  cos(x), 0
		}};
		Mat3x4f ry = {.rc = {
			 cos(y), 0, sin(y), 0,
			      0, 1,      0, 0,
			-sin(y), 0, cos(y), 0
		}};
		Mat3x4f rz = {.rc = {
			cos(z), -sin(z), 0, 0,
			sin(z),  cos(z), 0, 0,
			     0,       0, 1, 0
		}};
		return {};
	}

	static Mat3x4f fromRaw(const float* data)
	{
		Mat3x4f matrix = {0};
		memcpy(matrix.rc, data, 12 * sizeof(float));
		return matrix;
	}


	void toRaw(float* dst)
	{
		memcpy(dst,  rc, 12 * sizeof(float));
	}

	rgl_mat3x4f toRGL()
	{
		return { .value = {
			rc[0][0], rc[0][1], rc[0][2], rc[0][3],
			rc[1][0], rc[1][1], rc[1][2], rc[2][3],
			rc[2][0], rc[2][1], rc[2][2], rc[2][3],
		}};
	}

	Mat3x4f& operator=(const Mat3x4f& other) = default;

	__host__ __device__ float& operator[](int i) {return rc[i/4][i%4];}
	__host__ __device__ const float& operator[](int i) const {return rc[i/4][i%4];}

};

static_assert(std::is_trivially_copyable<Mat3x4f>::value);
static_assert(std::is_trivially_constructible<Mat3x4f>::value);
static_assert(sizeof(Mat3x4f) == 12 * sizeof(float));
static_assert(alignof(Mat3x4f) == 4);
