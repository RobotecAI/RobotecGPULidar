#include <gtest/gtest.h>
#include <cmath>
#include <math/Mat3x4f.hpp>

TEST(Mat3x4f, Multiplication)
{
	Mat3x4f lhs = {
	    .rc = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}
    };
	Mat3x4f rhs = {
	    .rc = {12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1}
    };
	Mat3x4f gold = {
	    .rc = {40, 34, 28, 26, 136, 118, 100, 90, 232, 202, 172, 154}
    };

	EXPECT_EQ(lhs * rhs, gold);
}

TEST(Mat3x4f, InverseUndoesTheTransform)
{
	Mat3x4f m = {
	    .rc = {0, -2, 0, 1, 1, 0, 0, -3, 0, 0, 0.5f, 7}
    };
	Mat3x4f identity = {
	    .rc = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}
    };
	Mat3x4f product = m * m.inverse();
	for (int i = 0; i < 12; ++i) {
		EXPECT_NEAR(product[i], identity[i], 1e-6f) << "element " << i;
	}
}

TEST(Mat3x4f, InverseOfSingularIsZero)
{
	Mat3x4f singular = {
	    .rc = {1, 2, 3, 4, 2, 4, 6, 8, 0, 0, 1, 0}
    };
	Mat3x4f zero = {
	    .rc = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
	EXPECT_EQ(singular.inverse(), zero);
}

TEST(Mat3x4f, InverseTranslationStaysFiniteForLargeTranslation)
{
	// Nearly singular rotation part and a large translation: the exact inverse translation is (0, -1e20, 0).
	Mat3x4f m = {
	    .rc = {1e-20f, 1, 0, 1e20f, 0, 1, 0, 1e20f, 0, 0, 1, 0}
    };
	Mat3x4f inv = m.inverse();
	EXPECT_TRUE(std::isfinite(inv[3]));
	EXPECT_FLOAT_EQ(inv[3], 0.0f);
	EXPECT_FLOAT_EQ(inv[7], -1e20f);
}
