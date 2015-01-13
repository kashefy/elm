/** @file test out Mat_ visitors,
 * some fixtures may be scattered somewhat awkwardly in this file due to conditional support (e.g. PCL support)
 */
#include "core/visitors/visitormat_.h"

#include "core/exception.h"
#include "core/pcl/cloud.h"
#include "core/pcl/vertices.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

namespace {

/**
 * @brief test class around VisitorMat_f
 */
class VisitorMat_fTest : public ::testing::Test
{
protected:
    VisitorMat_f to_;    ///< test object
};

TEST_F(VisitorMat_fTest, EmptyMat)
{
    EXPECT_TRUE(to_(Mat_f()).empty());
    EXPECT_TRUE(to_(Mat_f(0, 0)).empty());
    EXPECT_TRUE(to_(Mat_f(1, 0)).empty());
    EXPECT_TRUE(to_(Mat_f(0, 1)).empty());
}

TEST_F(VisitorMat_fTest, EmptyMatSize)
{
    EXPECT_MAT_DIMS_EQ(to_(Mat_f()), Size2i(0, 0));
    EXPECT_MAT_DIMS_EQ(to_(Mat_f(0, 0)), Size2i(0, 0));
    EXPECT_MAT_DIMS_EQ(to_(Mat_f(1, 0)), Size2i(0, 1));
    EXPECT_MAT_DIMS_EQ(to_(Mat_f(0, 1)), Size2i(1, 0));
}

TEST_F(VisitorMat_fTest, FromMat_f)
{
    int N=5;

    while(N--) {

        const int MAT_ROWS = abs(randu<int>()) % 10 + 1;
        const int MAT_COLS = abs(randu<int>()) % 100 + 1;

        Mat_f m(MAT_ROWS, MAT_COLS);
        randn(m, 0.f, 100.f);

        Mat_f m2 = to_(m);

        EXPECT_MAT_EQ(m, m2) << "Matrices are not equal";
        EXPECT_EQ(m.data, m2.data) << "Mats are not pointing to the same data in memory. Expecting shared copy.";
    }
}

TEST_F(VisitorMat_fTest, Reset)
{
    EXPECT_NO_THROW(to_.Reset());

    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);

    Mat_f m2 = to_(m);
    EXPECT_NO_THROW(to_.Reset());
}

#ifndef __WITH_PCL
    #warning "Disabling unit tests that require PCL support."
#else // __WITH_PCL

TEST_F(VisitorMat_fTest, EmptyCLoud)
{
    CloudXYZPtr cld(new CloudXYZ);
    EXPECT_TRUE(to_(cld).empty());
}

TEST_F(VisitorMat_fTest, EmptyCloudSize)
{
    CloudXYZPtr cld(new CloudXYZ);
    EXPECT_MAT_DIMS_EQ(to_(cld), Size2i(0, 0));
}

TEST_F(VisitorMat_fTest, FromCloud)
{
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZPtr cloud = Mat2PointCloud(m);

    Mat_f m2 = to_(cloud);
    hconcat(m, Mat1f(m.rows, 1, 1), m);

    EXPECT_MAT_EQ(m, m2);
    EXPECT_NE(m.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
}

TEST_F(VisitorMat_fTest, Reset_with_cloud)
{
    EXPECT_NO_THROW(to_.Reset());

    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);

    Mat_f m2 = to_(m);
    EXPECT_NO_THROW(to_.Reset());

    CloudXYZPtr cloud = Mat2PointCloud(m);
    m2 = to_(cloud);
    EXPECT_NO_THROW(to_.Reset());

    m2 = to_(m);
    EXPECT_NO_THROW(to_.Reset());
}

// fixtures for VecVertices

TEST_F(VisitorMat_fTest, Empty_VecVertices)
{
    EXPECT_TRUE(to_(VecVertices()).empty());
}

TEST_F(VisitorMat_fTest, Empty_VecVertices_Size)
{
    EXPECT_MAT_DIMS_EQ(to_(VecVertices()), Size2i(0, 0));
}

TEST_F(VisitorMat_fTest, From_VecVertices)
{
    Mat1f m(4, 3);
    for(size_t i=0; i<m.total(); i++) {

        m(i) = static_cast<float>(randu<uint32_t>() % 256);
    }
    VecVertices vv = Mat2VecVertices(m.clone());

    Mat_f m2 = to_(vv);

    EXPECT_MAT_EQ(m, m2);
    EXPECT_NE(m.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
}

TEST_F(VisitorMat_fTest, DISABLED_Reset_with_VecVertices)
{
//    EXPECT_NO_THROW(to_.Reset());

//    Mat1f m(4, 3);
//    randn(m, 0.f, 100.f);

//    Mat_f m2 = to_(m);
//    EXPECT_NO_THROW(to_.Reset());

//    CloudXYZPtr cloud = Mat2PointCloud(m);
//    m2 = to_(cloud);
//    EXPECT_NO_THROW(to_.Reset());

//    m2 = to_(m);
//    EXPECT_NO_THROW(to_.Reset());
}

#endif // __WITH_PCL

} // annonymous namespace for visitors' test fixtures
