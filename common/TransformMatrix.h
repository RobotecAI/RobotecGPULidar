#pragma once

#include <iostream>

#ifndef __host__
#define __host__
#endif

#ifndef __device__
#define __device__
#endif

struct TransformMatrix {
    float matrix_flat[12];

    static TransformMatrix identity() {
        TransformMatrix v = {0};
        v[0] = 1.0;
        v[5] = 1.0;
        v[10] = 1.0;
        return v;
    }

    static TransformMatrix fromPointer(float* data) {
        TransformMatrix matrix;
        memcpy(matrix.matrix_flat, data, 12 * sizeof(float));
        return matrix;
    }

    TransformMatrix& operator=(const TransformMatrix& other) = default;

    __host__ void print() {
        for (size_t i = 0; i < sizeof(matrix_flat) / (sizeof(*matrix_flat)); i++) {
            std::cout << matrix_flat[i] << " ";
            if (i % 4 == 3) std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    __host__ __device__ float& operator[](int i) {return matrix_flat[i];}
    __host__ __device__ const float& operator[](int i) const {return matrix_flat[i];}
};

static_assert(std::is_trivially_copyable<TransformMatrix>::value);
static_assert(std::is_trivially_constructible<TransformMatrix>::value);
static_assert(sizeof(TransformMatrix) == 12 * sizeof(float));
static_assert(alignof(TransformMatrix) == 4);
