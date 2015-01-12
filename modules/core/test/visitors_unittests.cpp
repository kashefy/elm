/** @file test out visitors,
 * some fixtures may be scattered somewhat awkwardly in this file due to conditional support (e.g. PCL support)
 */
#include "core/visitors.h"

#include "core/exception.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

namespace {

/**
 * @brief Type-Parameterized tests around POD visitor class
 */
template <class T>
class VisitorPOD_Test : public ::testing::Test
{
protected:
};
TYPED_TEST_CASE_P(VisitorPOD_Test);

TYPED_TEST_P(VisitorPOD_Test, Invalid_Mat) {

    VisitorPOD_<TypeParam > to;

    // empty
    EXPECT_THROW(to(Mat_f()), ExceptionBadDims);
    EXPECT_THROW(to(Mat_f(0, 0)), ExceptionBadDims);
    EXPECT_THROW(to(Mat_f(1, 0)), ExceptionBadDims);
    EXPECT_THROW(to(Mat_f(0, 1)), ExceptionBadDims);

    // multiple elements
    EXPECT_THROW(to(Mat_f(2, 1)), ExceptionBadDims);
    EXPECT_THROW(to(Mat_f(1, 2)), ExceptionBadDims);
    EXPECT_THROW(to(Mat_f(2, 2)), ExceptionBadDims);

    EXPECT_THROW(to(Mat_f(2, 1, 1.f)), ExceptionBadDims);
    EXPECT_THROW(to(Mat_f(1, 2, 0.f)), ExceptionBadDims);
    EXPECT_THROW(to(Mat_f(2, 1, 3.f)), ExceptionBadDims);
}

TYPED_TEST_P(VisitorPOD_Test, Value)
{
    VisitorPOD_<TypeParam > to;

    float _v = 256.f; // to cover a range of values common between all of our PODs
    while(--_v >= 0.f) {

        EXPECT_EQ(static_cast<TypeParam >(_v), to(Mat_f(1, 1, _v))) << "Value mismatch.";
    }
}

#ifdef __WITH_PCL // PCL support required for these tests

TYPED_TEST_P(VisitorPOD_Test, Invalid_Cloud)
{
    VisitorPOD_<TypeParam > to;

    // empty
    CloudXYZ::Ptr cld(new CloudXYZ);
    EXPECT_THROW(to(cld), ExceptionTypeError);

    // multi-row, 3-col
    cld = Mat2PointCloud(Mat1f(4, 3, 0.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    cld = Mat2PointCloud(Mat1f(4, 3, 1.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    cld = Mat2PointCloud(Mat1f(4, 3, 2.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    // single row
    cld = Mat2PointCloud(Mat1f(1, 3, 0.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    cld = Mat2PointCloud(Mat1f(1, 3, 1.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    // 3-channel
    cld = Mat2PointCloud(Mat3f(1, 1, 0.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    cld = Mat2PointCloud(Mat3f(1, 1, 1.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);
}
#else // __WITH_PCL

TYPED_TEST_P(VisitorPOD_Test, DISABLED_Invalid_Cloud)
{
}

#endif // __WITH_PCL

/** Write additional type+value parameterized tests here.
 *  Acquaint yourself with the values passed to along with each type.
 *
 *  Register test names:
 * @todo: add define macro guard for PCL support
 */
REGISTER_TYPED_TEST_CASE_P(VisitorPOD_Test,
                           Invalid_Mat,
                           Value,
                           Invalid_Cloud
                           ); ///< register additional typed_test_p (i.e. unit test) routines here

typedef ::testing::Types<float, int, uchar> PODTypes;  ///< lists the usual suspects of plain old data types
INSTANTIATE_TYPED_TEST_CASE_P(FeatureDataVisitorPOD_Test, VisitorPOD_Test, PODTypes);



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
    CloudXYZ::Ptr cld(new CloudXYZ);
    EXPECT_TRUE(to_(cld).empty());
}

TEST_F(VisitorMat_fTest, EmptyCloudSize)
{
    CloudXYZ::Ptr cld(new CloudXYZ);
    EXPECT_MAT_DIMS_EQ(to_(cld), Size2i(0, 0));
}

TEST_F(VisitorMat_fTest, FromCloud)
{
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZ::Ptr cloud = Mat2PointCloud(m);

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

    CloudXYZ::Ptr cloud = Mat2PointCloud(m);
    m2 = to_(cloud);
    EXPECT_NO_THROW(to_.Reset());

    m2 = to_(m);
    EXPECT_NO_THROW(to_.Reset());
}

/**
 * @brief test class around VisitorCloud
 */
class VisitorCloudTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.Reset();
    }

    VisitorCloud to_;    ///< test object
};

TEST_F(VisitorCloudTest, Empty)
{
    EXPECT_TRUE(to_(Mat_f())->empty());
    EXPECT_TRUE(to_(Mat_f(0, 0))->empty());
    EXPECT_TRUE(to_(Mat_f(1, 0))->empty());
    EXPECT_TRUE(to_(Mat_f(0, 1))->empty());

    CloudXYZ::Ptr cld(new CloudXYZ);
    EXPECT_TRUE(to_(cld)->empty());
}

TEST_F(VisitorCloudTest, EmptySize)
{
    EXPECT_EQ(size_t(0), to_(Mat_f())->size());
    EXPECT_EQ(size_t(0), to_(Mat_f(0, 0))->size());
    EXPECT_EQ(size_t(0), to_(Mat_f(1, 0))->size());
    EXPECT_EQ(size_t(0), to_(Mat_f(0, 1))->size());

    CloudXYZ::Ptr cld(new CloudXYZ);
    EXPECT_EQ(size_t(0), to_(cld)->size());
}

TEST_F(VisitorCloudTest, FromMat_f)
{
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);

    CloudXYZ::Ptr cld = to_(m);
    Mat1f m2 = PointCloud2Mat(cld);
    hconcat(m, Mat1f(m.rows, 1, 1), m);

    EXPECT_MAT_EQ(m, m2);
    EXPECT_NE(m.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
}

TEST_F(VisitorCloudTest, Cloud)
{
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZ::Ptr cld = Mat2PointCloud(m);

    CloudXYZ::Ptr cld2 = to_(cld);

    EXPECT_EQ(cld->size(), cld2->size()) << "Size mismatch.";
    EXPECT_EQ(cld->width, cld2->width) << "Width mismatch.";
    EXPECT_EQ(cld->height, cld2->height) << "Height mismatch.";

    for(CloudXYZ::iterator itr1=cld->begin(), itr2=cld2->begin();
        itr1 != cld->end(); ++itr1, ++itr2) {

        pcl::PointXYZ _p1 = *itr1;
        pcl::PointXYZ _p2 = *itr2;
        EXPECT_FLOAT_EQ(_p1.x, _p2.x) << "Unexpected value for x coordinate.";
        EXPECT_FLOAT_EQ(_p1.y, _p2.y) << "Unexpected value for y coordinate.";
        EXPECT_FLOAT_EQ(_p1.z, _p2.z) << "Unexpected value for z coordinate.";
    }
}

TEST_F(VisitorCloudTest, ResetValid)
{
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);

    CloudXYZ::Ptr cld1 = to_(m);
    EXPECT_NO_THROW(to_.Reset());

    CloudXYZ::Ptr cld2 = Mat2PointCloud(m);
    CloudXYZ::Ptr cld3 = to_(cld2);
    EXPECT_NO_THROW(to_.Reset());

    cld1 = to_(m);
    EXPECT_NO_THROW(to_.Reset());
}

/**
 * @brief Thest this visitor's caching
 */
TEST_F(VisitorCloudTest, Reset)
{
    Mat1f m(4, 3, 1.f);
    Mat1f m0 = m.clone();
    hconcat(m0, Mat1f(m0.rows, 1, 1), m0);

    CloudXYZ::Ptr cld = to_(m);
    EXPECT_MAT_EQ(PointCloud2Mat(cld), m0);

    cld = to_(m); // result of first call is cached.

    const int N=2;
    ASSERT_GT(N, 1) << "This test must iterate at least twice to see effect of Reset() on cached reference.";
    for(int i=0; i<2; i++) {

        cld = to_(m+1);

        if(i == 0) {

            EXPECT_MAT_EQ(PointCloud2Mat(cld), m0) << "Not suing cahced cloud.";
        }
        else {

            Mat1f m2 = m0.clone();
            m2.colRange(0, m2.cols-1).setTo(2.f);
            EXPECT_MAT_EQ(PointCloud2Mat(cld), m2) << "Still using cached cloud";
        }
        to_.Reset();
    }
}

#endif // __WITH_PCL

} // annonymous namespace for visitors' test fixtures
