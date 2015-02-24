/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file test out VecMat1f visitor,
 * some fixtures may be scattered somewhat awkwardly in this file due to conditional support (e.g. PCL support)
 */
#include "elm/core/visitors/visitorvecmat1f.h"

#include "elm/core/exception.h"
#include "elm/core/pcl/cloud_.h"
#include "elm/core/pcl/point_traits.h"
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
class VisitorVecMat1fTest : public ::testing::Test
{
protected:
    VisitorVecMat1f to_;    ///< test object
};

TEST_F(VisitorVecMat1fTest, EmptyVecMat1f)
{
    EXPECT_TRUE(to_(VecMat1f()).empty());
    EXPECT_TRUE(to_(VecMat1f(0)).empty());
}

TEST_F(VisitorVecMat1fTest, EmptyVecMat1f_size)
{
    EXPECT_SIZE(size_t(0), to_(VecMat1f()));
    EXPECT_SIZE(size_t(0), to_(VecMat1f(0)));
}

TEST_F(VisitorVecMat1fTest, FromVecMat1f_size)
{
    int N=5;

    VecMat1f in;
    for(int i=0; i<N; i++) {

        Mat1f m(2, 3, static_cast<float>(N-i));

        in.push_back(m);

        VecMat1f v = to_(in);
        EXPECT_SIZE(size_t(i+1), v);

        for(size_t j=0; j<v.size(); j++) {

            Mat1f m2 = v[j];

            EXPECT_MAT_EQ(m2, Mat1f(2, 3, static_cast<float>(N-j))) << "Matrices are not equal";
            EXPECT_EQ(in[j].data, m2.data) << "Mats are not pointing to the same data in memory. Expecting shared copy.";
        }
    }
}

TEST_F(VisitorVecMat1fTest, FromMat1f)
{
    int N=5;

    while(N--) {

        const int MAT_ROWS = abs(randu<int>()) % 10 + 1;
        const int MAT_COLS = abs(randu<int>()) % 100 + 1;

        Mat1f m(MAT_ROWS, MAT_COLS);
        randn(m, 0.f, 100.f);

        VecMat1f v = to_(m);

        EXPECT_SIZE(size_t(1), v);

        Mat1f m2 = v[0];

        EXPECT_MAT_EQ(m, m2) << "Matrices are not equal";
        EXPECT_EQ(m.data, m2.data) << "Mats are not pointing to the same data in memory. Expecting shared copy.";
    }
}

TEST_F(VisitorVecMat1fTest, EmptySparseMat1f)
{
    EXPECT_SIZE(1, to_(SparseMat1f()));
    EXPECT_TRUE(to_(SparseMat1f())[0].empty());

    SparseMat m(Mat1f(1, 1, 1.f));
    EXPECT_SIZE(1, to_(SparseMat1f()));
    m.clear();  // clearing is equivalent to setting elements to zero

    EXPECT_TRUE(to_(SparseMat1f())[0].empty());
}

TEST_F(VisitorVecMat1fTest, FromSparseMat1f)
{
    int N=5;

    while(N--) {

        const int MAT_ROWS = abs(randu<int>()) % 10 + 1;
        const int MAT_COLS = abs(randu<int>()) % 100 + 1;

        Mat1f m(MAT_ROWS, MAT_COLS);
        randn(m, 0.f, 100.f);

        SparseMat1f m2 = to_(SparseMat1f(m))[0];

        EXPECT_MAT_EQ(m, m2) << "Matrices are not equal";
    }
}

TEST_F(VisitorVecMat1fTest, Reset)
{
    EXPECT_NO_THROW(to_.Reset());
    to_(Mat1f(4, 3, 1.f));
    EXPECT_NO_THROW(to_.Reset());
}

#ifdef __WITH_PCL

template <class TPoint>
class VisitorVecMat1fCloudTest : public VisitorVecMat1fTest
{
protected:
    virtual void SetUp()
    {
        typedef PCLPointTraits_<TPoint > PTraits;

        field_count = static_cast<int>(PTraits::FieldCount());
        to_.Reset();
    }

    int field_count;
};

TYPED_TEST_CASE(VisitorVecMat1fCloudTest, PCLPointTypes);

TYPED_TEST(VisitorVecMat1fCloudTest, FromCLoud_empty)
{
    typename pcl::PointCloud<TypeParam >::Ptr cld(new pcl::PointCloud<TypeParam>());
    EXPECT_TRUE(this->to_(cld).empty());
}

TYPED_TEST(VisitorVecMat1fCloudTest, FromCloud)
{
    Mat1f m(4, this->field_count);
    randn(m, 0.f, 100.f);

    typename pcl::PointCloud<TypeParam >::Ptr cloud;
    cloud = Mat2PointCloud_<TypeParam >(m);
    m = PointCloud2Mat_(cloud); // already accounts for padding

    VecMat1f v = this->to_(cloud);
    EXPECT_SIZE(m.rows, v);

    for(size_t i=0; i<v.size(); i++) {

        Mat1f m2 = v[i];

        EXPECT_MAT_EQ(m.row(i), m2);
        if(i==0) {
            EXPECT_EQ(m.data, m2.data);
        }
        else {
            EXPECT_NE(m.data, m2.data);
        }
    }
}

TYPED_TEST(VisitorVecMat1fCloudTest, Reset_with_cloud)
{
    EXPECT_NO_THROW(this->to_.Reset());
    EXPECT_NO_THROW(this->to_.Reset());

    Mat1f m(4, this->field_count, 1.f);

    VecMat1f v = this->to_(m);
    EXPECT_NO_THROW(this->to_.Reset());

    typename pcl::PointCloud<TypeParam >::Ptr cloud;
    cloud = Mat2PointCloud_<TypeParam >(m);

    v = this->to_(cloud);
    EXPECT_NO_THROW(this->to_.Reset());

    v = this->to_(m);
    EXPECT_NO_THROW(this->to_.Reset());
    EXPECT_NO_THROW(this->to_.Reset());
}

//// fixtures for VecVertices

//TEST_F(VisitorVecMat1fTest, Empty_VecVertices)
//{
//    EXPECT_TRUE(to_(VecVertices()).empty());
//}

//TEST_F(VisitorVecMat1fTest, Empty_VecVertices_Size)
//{
//    EXPECT_MAT_DIMS_EQ(to_(VecVertices()), Size2i(0, 0));
//}

//TEST_F(VisitorVecMat1fTest, From_VecVertices)
//{
//    Mat1f m(4, 3);
//    for(size_t i=0; i<m.total(); i++) {

//        m(i) = static_cast<float>(randu<uint32_t>() % 256);
//    }
//    VecVertices vv = Mat2VecVertices(m.clone());

//    Mat1f m2 = to_(vv);

//    EXPECT_MAT_EQ(m, m2);
//    EXPECT_NE(m.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
//}

//TEST_F(VisitorVecMat1fTest, DISABLED_Reset_with_VecVertices)
//{
////    EXPECT_NO_THROW(to_.Reset());

////    Mat1f m(4, 3);
////    randn(m, 0.f, 100.f);

////    Mat1f m2 = to_(m);
////    EXPECT_NO_THROW(to_.Reset());

////    CloudXYZPtr cloud = Mat2PointCloud(m);
////    m2 = to_(cloud);
////    EXPECT_NO_THROW(to_.Reset());

////    m2 = to_(m);
////    EXPECT_NO_THROW(to_.Reset());
//}

#endif // __WITH_PCL

} // annonymous namespace for visitors' test fixtures

