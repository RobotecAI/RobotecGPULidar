#pragma once

static rgl_vec3f cubeVertices[] = {
    {-1, -1, -1},
    { 1, -1, -1},
    { 1,  1, -1},
    {-1,  1, -1},
    {-1, -1,  1},
    { 1, -1,  1},
    { 1,  1,  1},
    {-1,  1,  1}
};

static rgl_vec3f cubeVerticesX2[] = {
    {-2, -2, -2},
    { 2, -2, -2},
    { 2,  2, -2},
    {-2,  2, -2},
    {-2, -2,  2},
    { 2, -2,  2},
    { 2,  2,  2},
    {-2,  2,  2}
};

static rgl_vec3i cubeIndices[] = {
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

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(*array))
