#define HOSTDEVICE

#include <gtest/gtest.h>
#include "linearGeometry.h"

#include <iostream>

const int matrix_size = 12;

TEST(LinearAlgebraTest, TransformExtractTranslation) {
    TransformMatrix transform = {{1.0f, 0.0f, 0.0f, 1.0f,
                                  0.0f, 1.0f, 0.0f, 2.0f,
                                  0.0f, 0.0f, 1.0f, 3.0f}};
    gdt::vec3f result = getTranslationFrom3x4Transform(transform);
    std::cout << result << std::endl;

    gdt::vec3f result_golden(1.0f, 2.0f, 3.0f);
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(result[i], result_golden[i], 0.01f);
    }
}
TEST(LinearAlgebraTest, TransformMultiplicationIdentity) {
    TransformMatrix lhs = {{1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f}};
    TransformMatrix rhs = {{1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f}};

    TransformMatrix result = multiply3x4TransformMatrices(lhs, rhs);
    TransformMatrix result_golden = {{1.0f, 0.0f, 0.0f, 0.0f,
                                      0.0f, 1.0f, 0.0f, 0.0f,
                                      0.0f, 0.0f, 1.0f, 0.0f}};


    for (int i = 0; i < 12; i++) {
        EXPECT_NEAR(result[i], result_golden[i], 0.01f);
    }

    result.print();
}

TEST(LinearAlgebraTest, TransformMultiplicationDense) {
    TransformMatrix lhs = {{1.0f, 2.0f, 3.0f, 4.0f,
                            5.0f, 6.0f, 7.0f, 8.0f,
                            9.0f, 10.0f, 11.0f, 12.0f}};
    TransformMatrix rhs = {{13.0f, 14.0f, 15.0f, 16.0f,
                            17.0f, 18.0f, 19.0f, 20.0f,
                            21.0f, 22.0f, 23.0f, 24.0f}};
    TransformMatrix result = multiply3x4TransformMatrices(lhs, rhs);
    TransformMatrix result_golden = {{110.0f, 116.0f, 122.0f, 132.0f,
                                      314.0f, 332.0f, 350.0f, 376.0f,
                                      518.0f, 548.0f, 578.0f, 620.0f}};
    for (int i = 0; i < 12; i++) {
        EXPECT_NEAR(result[i], result_golden[i], 0.01f);
    }

    result.print();
}

TEST(LinearAlgebraTest, GenerateRotationMatrixAroundYAxis) {
    TransformMatrix result_golden = {0.5403023,  0.0000000,  -0.8414710, 0.0f,
                                     0.0000000,  1.0000000,  0.0000000, 0.0f,
                                     0.8414710,  0.0000000,  0.5403023, 0.0f};

    TransformMatrix result = yAxisRotation3x4Matrix(1.0f);

    for (int i = 0; i < 12; i++) {
        EXPECT_NEAR(result[i], result_golden[i], 0.01f);
    }
    result.print();

    gdt::vec3f point = {1.0f, 0.0f, 0.0f};

    std::cout << "input point: " << point << std::endl;
    gdt::vec3f rotated_point = multiply3x4TransformByVector3(result, point);
    std::cout << "rotated point: " << rotated_point << std::endl;

    float angle = std::atan2(rotated_point.z, rotated_point.x);
    std::cout << "angle : " << angle << std::endl;

    EXPECT_NEAR(angle, 1.0f, 0.01f);
}

TEST(LinearAlgebraTest, TransformVectorMultiplicationIdentity) {
    TransformMatrix lhs = {{1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f}};
    gdt::vec3f rhs(0.0f, 0.0f, 0.0f);
    gdt::vec3f result = multiply3x4TransformByVector3(lhs, rhs);
    gdt::vec3f result_golden(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(result[i], result_golden[i], 0.01f);
    }
    std::cout << result << std::endl;
}

TEST(LinearAlgebraTest, TransformVectorMultiplicationRotate45x) {
    TransformMatrix lhs = {{1.0f, 0.0f, 0,       0.0f,
                            0.0f, 0.71f, -0.71f, 0.0f,
                            0.0f, 0.71f,  0.71f, 0.0f}};
    gdt::vec3f rhs(0.0f, 1.0f, 0.0f);
    gdt::vec3f result = multiply3x4TransformByVector3(lhs, rhs);
    gdt::vec3f result_golden(0.0f, 0.71f, 0.71f);
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(result[i], result_golden[i], 0.01f);
    }
    std::cout << result << std::endl;
}

TEST(LinearAlgebraTest, TransformVectorMultiplicationTranlate) {
    TransformMatrix lhs = {{0.77f, -0.18f,  0.61f, 1.0f,
                            0.56f,  0.66f, -0.5f,  2.0f,
                            -0.31f, 0.73f,  0.61f, 3.0f}};
    gdt::vec3f rhs(1.0f,   0.0f,   0.0f);
    gdt::vec3f result = multiply3x4TransformByVector3(lhs, rhs);
    gdt::vec3f result_golden(1.7700f, 2.5600f, 2.6900f);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(result[i], result_golden[i], 0.01f);
    }
    std::cout << result << std::endl;
}
