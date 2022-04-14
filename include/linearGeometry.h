#pragma once

#include <TransformMatrix.h>
#include "gdt/math/vec.h"
#include "gdt/utils/optix_macros.h"

static HOSTDEVICE TransformMatrix multiply3x4TransformMatrices(const TransformMatrix& lhs, const TransformMatrix& rhs)
{
    TransformMatrix ret;
    int M = 3;
    int R = 4;
    int N = 4;

    for ( int i = 0; i < M; ++i ) {
        for ( int j = 0; j < R; ++j ) {
            float sum = 0.0f;
            for ( int k = 0; k < N - 1; ++k ) {
                float ik = lhs[i*N+k];
                float kj = rhs[k*R+j];
                sum += ik * kj;
            }
            if (j == 3) {
                sum += lhs[i*N+3];
            }
            ret[i*R+j] = sum;
        }
    }
    return ret;
}

static HOSTDEVICE TransformMatrix yAxisRotation3x4Matrix(float angle) {
    TransformMatrix ret;
    ret[0] = cos(angle);  ret[1] = 0.0f; ret[2] = -sin(angle);  ret[3] = 0.0f;
    ret[4] = 0.0f;        ret[5] = 1.0f; ret[6] = 0.0f;        ret[7] = 0.0f;
    ret[8] = sin(angle); ret[9] = 0.0f; ret[10] = cos(angle); ret[11] = 0.0f;
    return ret;
}

static HOSTDEVICE gdt::vec3f getTranslationFrom3x4Transform(const TransformMatrix& transform) {
    return gdt::vec3f(transform[3], transform[7], transform[11]);
}

static HOSTDEVICE gdt::vec3f multiply3x4TransformByVector3(const TransformMatrix& m_lhs, const gdt::vec3f v_rhs) {
    gdt::vec3f ret (0.0f, 0.0f, 0.0f);
    int M = 3;
    int N = 4;

    for ( unsigned int i = 0; i < M; ++i ) {
        float sum = 0.0f;
        for ( unsigned int k = 0; k < N - 1; ++k ) {
            float ik = m_lhs[ i*N+k ];
            float kv = v_rhs[ k ];
            sum += ik * kv;
        }
        sum += m_lhs[ i*N+3 ];

        ret[i] = sum;
    }
    return ret;
}
