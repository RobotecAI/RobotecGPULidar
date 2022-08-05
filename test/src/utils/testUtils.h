#pragma once

#include <rgl/api/experimental.h>
#include <vector>
#include <cmath>
#include <numeric>
#include <gtest/gtest.h>

using namespace ::testing;

#define EXPECT_RGL_SUCCESS(status) EXPECT_EQ(status, rgl_status_t::RGL_SUCCESS)
#define EXPECT_RGL_INVALID_ARGUMENT(status, error_string_msg)     \
    {                                                             \
        EXPECT_EQ(status, rgl_status_t::RGL_INVALID_ARGUMENT);    \
        const char* error_string;                                 \
        rgl_get_last_error_string(&error_string);                 \
        EXPECT_THAT(error_string, HasSubstr("Invalid argument")); \
        EXPECT_THAT(error_string, HasSubstr(error_string_msg));   \
    }

template <typename T>
std::vector<float> computeDistances(const T* data, int size)
{
    std::vector<float> return_data(size);
    for (int i = 0; i < size; i++) {
        return_data[i] = sqrt(data[i].value[0] * data[i].value[0] + data[i].value[1] * data[i].value[1] + data[i].value[2] * data[i].value[2]);
    }
    return return_data;
}

template <typename T>
std::vector<float> computeAngles(const T* data, int size)
{
    std::vector<float> return_data(size);
    for (int i = 0; i < size; i++) {
        return_data[i] = std::atan2(data[i].value[2], data[i].value[0]);
    }
    return return_data;
}

static void getLidarResults(rgl_lidar_t lidar, int* hitpointCount, void* results)
{
	EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(nullptr, lidar));
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, hitpointCount));
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));
}

template<typename T>
std::pair<T, T> mean_and_stdev(std::vector<T> v) {
	float sum = std::accumulate(v.begin(), v.end(), 0.0);
	float mean = sum / v.size();

	std::vector<float> diff(v.size());
	std::transform(v.begin(), v.end(), diff.begin(),
	               std::bind2nd(std::minus<T>(), mean));
	float sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	float stdev = std::sqrt(sq_sum / v.size());

	return {mean, stdev};
}

static rgl_vec3f cube_vertices[] = {
	{-1, -1, -1},
	{1, -1, -1},
	{1, 1, -1},
	{-1, 1, -1},
	{-1, -1, 1},
	{1, -1, 1},
	{1, 1, 1},
	{-1, 1, 1}
};

static rgl_vec3f cube_vertices_big[] = {
	{-2, -2, -2},
	{2, -2, -2},
	{2, 2, -2},
	{-2, 2, -2},
	{-2, -2, 2},
	{2, -2, 2},
	{2, 2, 2},
	{-2, 2, 2}
};

static constexpr size_t cube_vertices_length = sizeof(cube_vertices) / sizeof(cube_vertices[0]);
static constexpr size_t cube_vertices_big_length = sizeof(cube_vertices_big) / sizeof(cube_vertices_big[0]);

static rgl_vec3i cube_indices[] = {
	{0, 1, 3},
	{3, 1, 2},
	{1, 5, 2},
	{2, 5, 6},
	{5, 4, 6},
	{6, 4, 7},
	{4, 0, 7},
	{7, 0, 3},
	{3, 2, 7},
	{7, 2, 6},
	{4, 5, 0},
	{0, 5, 1},
};
static constexpr size_t cube_indices_length = sizeof(cube_indices) / sizeof(cube_indices[0]);
