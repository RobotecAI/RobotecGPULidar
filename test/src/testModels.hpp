#pragma once

#include <rgl/api/experimental.h>

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

static constexpr size_t cube_vertices_length = sizeof(cube_vertices) / sizeof(cube_vertices[0]);

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
