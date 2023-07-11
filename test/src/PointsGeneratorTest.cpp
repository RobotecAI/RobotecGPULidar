#include <gtest/gtest.h>
#include <RGLFields.hpp>
#include <PointsGenerator.hpp>

class TestPointTest : public ::testing::TestWithParam<std::tuple<Field<XYZ_F32>::type, Field<INTENSITY_F32>::type, bool>> {
protected:
    TestPoint referencePoint = TestPoint(Field<XYZ_F32>::type{1, 2, 3}, 5);
};

INSTANTIATE_TEST_SUITE_P(
    TestPointTestValues,
    TestPointTest,
    ::testing::Values(
        std::make_tuple(Field<XYZ_F32>::type{1, 2, 3}, 5, true),
        std::make_tuple(Field<XYZ_F32>::type{2, 2, 3}, 5, false),
        std::make_tuple(Field<XYZ_F32>::type{1, 3, 3}, 5, false),
        std::make_tuple(Field<XYZ_F32>::type{1, 2, 4}, 5, false),
        std::make_tuple(Field<XYZ_F32>::type{1, 2, 3}, 6, false)
    )
);

TEST_P(TestPointTest, equality_operator) 
{
    TestPoint point(std::get<0>(GetParam()), std::get<1>(GetParam()));
    EXPECT_EQ((referencePoint == point), std::get<2>(GetParam()));
}


class TestPointIsHitTest : public ::testing::TestWithParam<std::tuple<Field<XYZ_F32>::type, Field<INTENSITY_F32>::type, Field<IS_HIT_I32>::type, bool>> {
protected:
    TestPointIsHit referencePoint = TestPointIsHit(Field<XYZ_F32>::type{1, 2, 3}, 5, 1);
};

TEST_P(TestPointIsHitTest, eaquality_operator) 
{
    TestPointIsHit point(std::get<0>(GetParam()), std::get<1>(GetParam()), std::get<2>(GetParam()));
    EXPECT_EQ((referencePoint == point), std::get<3>(GetParam()));
}

INSTANTIATE_TEST_SUITE_P(
    TestPointIsHitTestValues,
    TestPointIsHitTest,
    ::testing::Values(
        std::make_tuple(Field<XYZ_F32>::type{1, 2, 3}, 5, 1, true),
        std::make_tuple(Field<XYZ_F32>::type{2, 2, 3}, 5, 1, false),
        std::make_tuple(Field<XYZ_F32>::type{1, 3, 3}, 5, 1, false),
        std::make_tuple(Field<XYZ_F32>::type{1, 2, 4}, 5, 1, false),
        std::make_tuple(Field<XYZ_F32>::type{1, 2, 3}, 6, 1, false),
        std::make_tuple(Field<XYZ_F32>::type{1, 2, 3}, 5, 0, false)
    )
);
