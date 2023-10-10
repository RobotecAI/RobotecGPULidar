#pragma once

#include <math/Mat3x4f.hpp>

static rgl_mat3x4f identityTestTransform = Mat3x4f::identity().toRGL();
static rgl_mat3x4f translationTestTransform = Mat3x4f::translation(2, 3, 4).toRGL();
static rgl_mat3x4f rotationTestTransform = Mat3x4f::rotation(10, 30, 45).toRGL();
static rgl_mat3x4f scalingTestTransform = Mat3x4f::scale(1, 2, 3).toRGL();
static rgl_mat3x4f complexTestTransform = Mat3x4f::TRS(Vec3f(1, 2, 3), Vec3f(10, 30, 45)).toRGL();

template<typename T>
std::pair<T, T> calcMeanAndStdev(const std::vector<T>& v)
{
	if (v.empty()) {
		return {0, 0};
	}

	T mean = 0;
	T m2 = 0; // sum of squares of differences from the current mean

	for (int i = 0; i < v.size(); ++i) {
		T delta = v[i] - mean;
		mean += delta / (i + 1);
		T delta2 = v[i] - mean;
		m2 += delta * delta2;
	}

	T variance = m2 / v.size();
	T stdev = std::sqrt(variance);
	return {mean, stdev};
}

template<typename T>
std::vector<float> computeAngles(const T* data, int size)
{
	std::vector<float> outAngles(size);
	for (int i = 0; i < size; i++) {
		outAngles[i] = std::atan2(data[i].z(), data[i].x());
	}
	return outAngles;
}
