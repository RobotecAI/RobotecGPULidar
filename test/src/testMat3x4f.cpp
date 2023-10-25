#include <gtest/gtest.h>
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
