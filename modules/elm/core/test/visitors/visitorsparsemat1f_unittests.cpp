/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file test out Mat_ visitors,
 * some fixtures may be scattered somewhat awkwardly in this file due to conditional support (e.g. PCL support)
 */
#include "elm/core/visitors/visitorsparsemat1f.h"

#include "elm/core/exception.h"
#include "elm/core/pcl/cloud_.h"
#include "elm/core/pcl/vertices.h"
#include "elm/ts/pcl_point_typed_tests.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

/**
 * @brief test class around VisitorMat1f
 */
class VisitorSparseMat1fTest : public ::testing::Test
{
protected:
    VisitorSparseMat1f to_;    ///< test object
};

TEST_F(VisitorSparseMat1fTest, EmptySparseMat1f)
{
    EXPECT_EQ(0, to_(SparseMat1f()).size());
    EXPECT_THROW(to_(SparseMat1f(0, 0)), std::exception);
}

TEST_F(VisitorSparseMat1fTest, EmptyMat1f)
{
    EXPECT_EQ(0, to_(Mat1f()).size());
    EXPECT_EQ(0, to_(Mat1f(0, 0)).size());
    EXPECT_EQ(0, to_(Mat1f(1, 0)).size());
    EXPECT_EQ(0, to_(Mat1f(0, 1)).size());
}

TEST_F(VisitorSparseMat1fTest, EmptyMat)
{
    EXPECT_THROW(to_(Mat()), ExceptionBadDims);

    EXPECT_THROW(to_(Mat(0, 0, CV_32FC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 0, CV_32FC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 0, CV_32FC3)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_32FC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_32FC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_32FC3)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_32FC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_32FC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_32FC3)), ExceptionBadDims);

    EXPECT_THROW(to_(Mat(0, 0, CV_8UC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 0, CV_8UC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 0, CV_8UC3)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_8UC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_8UC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_8UC3)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_8UC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_8UC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_8UC3)), ExceptionBadDims);

    EXPECT_THROW(to_(Mat(0, 0, CV_32SC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 0, CV_32SC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 0, CV_32SC3)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_32SC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_32SC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_32SC3)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_32SC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_32SC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_32SC3)), ExceptionBadDims);

    EXPECT_THROW(to_(Mat(0, 0, CV_8UC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 0, CV_8UC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 0, CV_8UC3)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_8UC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_8UC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(0, 1, CV_8UC3)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_8UC1)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_8UC2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat(1, 0, CV_8UC3)), ExceptionBadDims);
}

TEST_F(VisitorSparseMat1fTest, FromSparseMat1f)
{
    int N=5;

    while(N--) {

        const int MAT_ROWS = abs(randu<int>()) % 10 + 1;
        const int MAT_COLS = abs(randu<int>()) % 100 + 1;

        Mat1f tmp(MAT_ROWS, MAT_COLS);
        randn(tmp, 0.f, 100.f);
        SparseMat1f m(tmp);

        Mat1f m2;
        to_(m).convertTo(m2, CV_32FC1);

        EXPECT_MAT_EQ(m, m2) << "Matrices are not equal";
    }
}

TEST_F(VisitorSparseMat1fTest, FromDenseMat1f)
{
    int N=5;

    while(N--) {

        const int MAT_ROWS = abs(randu<int>()) % 10 + 1;
        const int MAT_COLS = abs(randu<int>()) % 100 + 1;

        Mat1f m(MAT_ROWS, MAT_COLS);
        randn(m, 0.f, 100.f);

        {
            Mat1f m2;
            to_(m).convertTo(m2, CV_32FC1);

            EXPECT_MAT_EQ(m, m2) << "Matrices are not equal";
            EXPECT_NE(m.data, m2.data);
        }
        {
            Mat m1;
            m.convertTo(m1, CV_64FC1);
            Mat1f m2;
            to_(m).convertTo(m2, CV_32FC1);

            EXPECT_MAT_EQ(m, m2) << "Matrices are not equal";
            EXPECT_NE(m.data, m2.data);
        }
        {
            Mat m1;
            m.convertTo(m1, CV_8UC1);
            Mat1f m2;
            to_(m).convertTo(m2, CV_32FC1);

            EXPECT_MAT_EQ(m, m2) << "Matrices are not equal";
            EXPECT_NE(m.data, m2.data);
        }
        {
            Mat m1;
            m.convertTo(m1, CV_32SC1);
            Mat1f m2;
            to_(m).convertTo(m2, CV_32FC1);

            EXPECT_MAT_EQ(m, m2) << "Matrices are not equal";
            EXPECT_NE(m.data, m2.data);
        }
    }
}

TEST_F(VisitorSparseMat1fTest, FromVecMat1f_empty)
{
    EXPECT_EQ(0, to_(VecMat1f()).size());
    EXPECT_EQ(0, to_(VecMat1f(0)).size());\
}

TEST_F(VisitorSparseMat1fTest, FromVecMat1f_empty_elements)
{
    EXPECT_EQ(0, to_(VecMat1f(1)).size());
    EXPECT_EQ(0, to_(VecMat1f(1, Mat1f())).size());
    EXPECT_EQ(0, to_(VecMat1f(3, Mat1f())).size());
    EXPECT_EQ(0, to_(VecMat1f(3, Mat1f(1, 0))).size());
    EXPECT_EQ(0, to_(VecMat1f(3, Mat1f(0, 1))).size());
}

TEST_F(VisitorSparseMat1fTest, FromVecMat1f_row_mats)
{
    const int N=5;
    VecMat1f v;
    for(int i=0; i<N; i++) {

        v.push_back(Mat1f(1, 3, static_cast<float>(i)));

        SparseMat1f sparse = to_(v);
        Mat1f m;
        sparse.convertTo(m, CV_32FC1);

        EXPECT_EQ(3, m.cols);
        EXPECT_EQ(i+1, m.rows);

        for(int r=0; r<m.rows; r++) {

            for(int c=0; c<m.cols; c++) {

                EXPECT_FLOAT_EQ(m(r, c), v[r](c));
            }
        }
    }
}

TEST_F(VisitorSparseMat1fTest, FromVecMat1f_rows_concat)
{
    VecMat1f v;
    v.push_back(Mat1f(2, 3, 1.f));
    v.push_back(Mat1f(3, 3, 2.f));

    SparseMat1f sparse = to_(v);
    Mat1f m;
    sparse.convertTo(m, CV_32FC1);

    EXPECT_MAT_EQ(m.rowRange(0, 2), v[0]);
    EXPECT_MAT_EQ(m.rowRange(2, m.rows), v[1]);
}

TEST_F(VisitorSparseMat1fTest, FromVecMat1f_variable_size)
{
    {
        VecMat1f v;
        v.push_back(Mat1f(1, 5, 1.f));
        v.push_back(Mat1f(1, 2, 2.f));

        EXPECT_THROW(to_(v), ExceptionBadDims);
    }
    {
        VecMat1f v;
        v.push_back(Mat1f(1, 5, 1.f));
        v.push_back(Mat1f(1, 2, 0.f));

        EXPECT_THROW(to_(v), ExceptionBadDims);
    }
    {
        VecMat1f v;
        v.push_back(Mat1f(1, 2, 1.f));
        v.push_back(Mat1f(1, 5, 2.f));

        EXPECT_THROW(to_(v), ExceptionBadDims);
    }
    {
        VecMat1f v;
        v.push_back(Mat1f(1, 2, 0.f));
        v.push_back(Mat1f(1, 5, 2.f));

        EXPECT_THROW(to_(v), ExceptionBadDims);
    }
}

TEST_F(VisitorSparseMat1fTest, Reset)
{
    EXPECT_NO_THROW(to_.Reset());

    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);

    Mat1f m2;
    to_(m).convertTo(m2, CV_32FC1);
    EXPECT_NO_THROW(to_.Reset());
}

#ifdef __WITH_PCL // unit tests that require PCL support (e.g point cloud)

// fixtures for VecVertices

TEST_F(VisitorSparseMat1fTest, Empty_VecVertices)
{
    EXPECT_EQ(0, to_(VecVertices()).size());
}

TEST_F(VisitorSparseMat1fTest, Empty_VecVertices_Size)
{
    EXPECT_MAT_DIMS_EQ(to_(VecVertices()), Size2i(0, 0));
}

TEST_F(VisitorSparseMat1fTest, From_VecVertices)
{
    Mat1f m(4, 3);
    for(size_t i=0; i<m.total(); i++) {

        m(i) = static_cast<float>(randu<uint32_t>() % 256);
    }
    VecVertices vv = Mat2VecVertices(m.clone());

    Mat1f m2;
    to_(vv).convertTo(m2, CV_32FC1);

    EXPECT_MAT_EQ(m, m2);
    EXPECT_NE(m.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
}

TEST_F(VisitorSparseMat1fTest, Reset_with_VecVertices)
{
    EXPECT_NO_THROW(to_.Reset());

    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);

    SparseMat1f m2 = to_(m);
    EXPECT_NO_THROW(to_.Reset());

    CloudXYZPtr cloud = Mat2PointCloud_<pcl::PointXYZ >(m);
    m2 = to_(cloud);
    EXPECT_NO_THROW(to_.Reset());

    m2 = to_(m);
    EXPECT_NO_THROW(to_.Reset());
}

template <class TPoint>
class VisitorSparseMat1fCloud_TypedTest : public VisitorSparseMat1fTest
{
protected:
    virtual void SetUp() {

        size_t field_count_sz = elm::ts::ExpectedPointAttr_<TPoint>::field_count;
        field_count_ = static_cast<int>(field_count_sz);

        size_t nb_floats_sz = elm::ts::ExpectedPointAttr_<TPoint>::nb_floats;
        nb_floats_ = static_cast<int>(nb_floats_sz);
    }

    // members
    int field_count_;
    int nb_floats_;
};

TYPED_TEST_CASE(VisitorSparseMat1fCloud_TypedTest, PCLPointTypes);

TYPED_TEST(VisitorSparseMat1fCloud_TypedTest, EmptyCLoud) {

    typename pcl::PointCloud<TypeParam >::Ptr cld(new pcl::PointCloud<TypeParam >);
    EXPECT_EQ(0, to_(cld).size());
}

TYPED_TEST(VisitorSparseMat1fCloud_TypedTest, FromCloud) {

    Mat1f m(4, this->field_count_);
    randn(m, 0.f, 100.f);
    typename pcl::PointCloud<TypeParam >::Ptr cloud = Mat2PointCloud_<TypeParam >(m);

    Mat1f m2;
    to_(cloud).convertTo(m2, CV_32FC1);

    Mat1f m_ref = PointCloud2Mat_<TypeParam> (cloud);

    EXPECT_MAT_EQ(m_ref, m2);
    EXPECT_NE(m.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
}

TYPED_TEST(VisitorSparseMat1fCloud_TypedTest, Reset_with_cloud) {

    EXPECT_NO_THROW(this->to_.Reset());

    Mat1f m(4, this->field_count_);
    randn(m, 0.f, 100.f);

    Mat1f m2;
    this->to_(m).convertTo(m2, CV_32FC1);
    EXPECT_NO_THROW(this->to_.Reset());

    typename pcl::PointCloud<TypeParam >::Ptr cloud = Mat2PointCloud_<TypeParam >(m);
    this->to_(cloud);
    EXPECT_NO_THROW(this->to_.Reset());

    this->to_(m);
    EXPECT_NO_THROW(this->to_.Reset());
}

#endif // __WITH_PCL

template <class T>
class VisitorSparseMat1fPOD_TypedTest : public VisitorSparseMat1fTest
{
protected:
};

typedef ::testing::Types<float, int, uchar> PODTypes;
TYPED_TEST_CASE(VisitorSparseMat1fPOD_TypedTest, PODTypes);

TYPED_TEST(VisitorSparseMat1fPOD_TypedTest, FromPOD)
{
    int N=10;

    while(N--) {

        TypeParam v = randu<TypeParam>();

        SparseMat1f m = this->to_(v);
        EXPECT_EQ(2, m.dims());
        EXPECT_EQ(1, m.size(0));
        EXPECT_EQ(1, m.size(1));
        EXPECT_EQ(m.size(0), m.size(1));

        Mat1f m2;
        m.convertTo(m2, CV_32FC1);
        EXPECT_EQ(static_cast<float>(v), m2(0));
    }
}

} // annonymous namespace for visitors' test fixtures
