#pragma once

#include <rgl/api/core.h>

static constexpr float CUBE_HALF_EDGE = 1.0;
static constexpr float CUBE_EDGE = 2.0;

static rgl_vec3f cubeVertices[] = {
    {-1, -1, -1}, // 0
    { 1, -1, -1}, // 1
    { 1,  1, -1}, // 2
    {-1,  1, -1}, // 3
    {-1, -1,  1}, // 4
    { 1, -1,  1}, // 5
    { 1,  1,  1}, // 6
    {-1,  1,  1}  // 7
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

static rgl_vec2f cubeUVs[] = {
    {0.0f, 0.0f},
    {1.0f, 0.0f},
    {1.0f, 1.0f},
    {0.0f, 1.0f},
    {0.0f, 0.0f},
    {1.0f, 0.0f},
    {1.0f, 1.0f},
    {0.0f, 1.0f},
};

// Indices follow right-hand rule to have normal pointing away from object
static rgl_vec3i cubeIndices[] = {
    {0, 3, 1},
    {3, 2, 1},
    {1, 2, 5},
    {2, 6, 5},
    {5, 6, 4},
    {6, 7, 4},
    {4, 7, 0},
    {7, 3, 0},
    {3, 7, 2},
    {7, 6, 2},
    {4, 0, 5},
    {0, 1, 5}
};

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(*array))
