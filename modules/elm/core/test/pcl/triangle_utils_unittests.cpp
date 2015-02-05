/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/pcl/triangle_utils.h"

#include "gtest/gtest.h"

#ifdef __WITH_PCL

#include <opencv2/core.hpp>
#include <pcl/point_types.h>

#include "elm/core/debug_utils.h"
#include "elm/ts/mat_assertions.h"

using namespace cv;
using namespace pcl;
using namespace elm;

namespace {

class TriangleEdgesTest : public ::testing::Test
{
};

TEST_F(TriangleEdgesTest, Dims)
{
    PointXYZ p0(0.f, 0.f, 0.f);
    PointXYZ p1(0.f, 0.f, 0.f);
    PointXYZ p2(0.f, 0.f, 0.f);

    Mat1f e = TriangleEdges(p0, p1, p2);

    EXPECT_MAT_DIMS_EQ(e, Size2i(3, 1)) << "Expecting row matrix with column per point.";
}

TEST_F(TriangleEdgesTest, Origin_all)
{
    PointXYZ p0(0.f, 0.f, 0.f);
    PointXYZ p1(0.f, 0.f, 0.f);
    PointXYZ p2(0.f, 0.f, 0.f);

    Mat1f e = TriangleEdges(p0, p1, p2);

    EXPECT_MAT_EQ(e, Mat1f::zeros(1, 3));
}

TEST_F(TriangleEdgesTest, Origin_first)
{
    PointXYZ p0(0.f, 0.f, 0.f);
    PointXYZ p1(1.f, 1.f, 1.f);
    PointXYZ p2(2.f, 2.f, 2.f);

    Mat1f e = TriangleEdges(p0, p1, p2);

    EXPECT_FLOAT_EQ(e(0), static_cast<float>(sqrt(3.)));
    EXPECT_FLOAT_EQ(e(1), static_cast<float>(2.*sqrt(3.)));
    EXPECT_FLOAT_EQ(e(2), e(0));
}

TEST_F(TriangleEdgesTest, ZeroDistance)
{
    PointXYZ p0(0.8f, 3.f, 5.f);
    PointXYZ p1(0.8f, 3.f, 5.f);
    PointXYZ p2(0.8f, 3.f, 5.f);

    Mat1f e = TriangleEdges(p0, p1, p2);

    EXPECT_MAT_EQ(e, Mat1f::zeros(1, 3));
}

TEST_F(TriangleEdgesTest, Values)
{
    PointXYZ p0(0.1f, 2.f, 3.f);
    PointXYZ p1(4.f, 0.5f, 6.f);
    PointXYZ p2(7.f, 8.f, 9.f);

    Mat1f e = TriangleEdges(p0, p1, p2);

    EXPECT_FLOAT_EQ(e(0), static_cast<float>(sqrt(3.9*3.9+1.5*1.5+9.f)));
    EXPECT_FLOAT_EQ(e(1), static_cast<float>(sqrt(6.9*6.9+36.+36.)));
    EXPECT_FLOAT_EQ(e(2), static_cast<float>(sqrt(9.+7.5*7.5+9.)));
}

TEST_F(TriangleEdgesTest, Order)
{
    PointXYZ p0(0.1f, 2.f, 3.f);
    PointXYZ p1(4.f, 0.5f, 6.f);
    PointXYZ p2(7.f, 8.f, 9.f);

    Mat1f e012 = TriangleEdges(p0, p1, p2);

    EXPECT_FLOAT_EQ(e012(0), static_cast<float>(sqrt(3.9*3.9+1.5*1.5+9.f)));
    EXPECT_FLOAT_EQ(e012(1), static_cast<float>(sqrt(6.9*6.9+36.+36.)));
    EXPECT_FLOAT_EQ(e012(2), static_cast<float>(sqrt(9.+7.5*7.5+9.)));

    Mat1f e102 = TriangleEdges(p1, p0, p2);

    EXPECT_FLOAT_EQ(e012(0), e102(0));
    EXPECT_FLOAT_EQ(e012(1), e102(2));
    EXPECT_FLOAT_EQ(e012(2), e102(1));

    Mat1f e210 = TriangleEdges(p2, p1, p0);

    EXPECT_FLOAT_EQ(e012(0), e210(2));
    EXPECT_FLOAT_EQ(e012(1), e210(1));
    EXPECT_FLOAT_EQ(e012(2), e210(0));
}

TEST_F(TriangleEdgesTest, Negative)
{
    PointXYZ p0(0.f, 0.f, 0.f);
    PointXYZ p1(-1.f, -1.f, -1.f);
    PointXYZ p2(2.f, 2.f, 2.f);

    Mat1f e = TriangleEdges(p0, p1, p2);

    EXPECT_FLOAT_EQ(e(0), static_cast<float>(sqrt(3.)));
    EXPECT_FLOAT_EQ(e(1), static_cast<float>(2.*sqrt(3.)));
    EXPECT_FLOAT_EQ(e(2), static_cast<float>(3.*sqrt(3.)));
}

} // annonymous namespace for tests

#endif // __WITH_PCL
