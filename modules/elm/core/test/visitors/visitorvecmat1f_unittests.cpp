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

TEST_F(VisitorVecMat1fTest, FromMat_floats)
{
    int N=5;

    while(N--) {

        const int MAT_ROWS = abs(randu<int>()) % 10 + 1;
        const int MAT_COLS = abs(randu<int>()) % 100 + 1;

        Mat m(MAT_ROWS, MAT_COLS, CV_32FC1);
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

// fixtures for VecVertices

TEST_F(VisitorVecMat1fTest, Empty_VecVertices)
{
    EXPECT_TRUE(to_(VecVertices()).empty());
}

TEST_F(VisitorVecMat1fTest, Empty_VecVertices_size)
{
    EXPECT_SIZE(0, to_(VecVertices()));
}

TEST_F(VisitorVecMat1fTest, From_VecVertices_empty_Vertices)
{
    EXPECT_SIZE(1, to_(VecVertices(1)));
    EXPECT_SIZE(2, to_(VecVertices(2)));
}

TEST_F(VisitorVecMat1fTest, From_VecVertices)
{
    Mat1f m(4, 3);
    for(size_t i=0; i<m.total(); i++) {

        m(i) = static_cast<float>(randu<uint32_t>() % 256);
    }
    VecVertices vv = Mat2VecVertices(m);

    VecMat1f vm = to_(vv);

    EXPECT_SIZE(m.rows, vm);

    for(size_t i=0; i<vm.size(); i++) {

        Mat1f m2 = vm[i];
        EXPECT_MAT_EQ(m.row(i), m2);
        EXPECT_NE(m.row(i).data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
    }
}

TEST_F(VisitorVecMat1fTest, From_VecVertices_mixed_empty)
{
    VecVertices vv;
    pcl::Vertices v;
    vv.push_back(v);
    v.vertices.push_back(uint32_t(3));
    vv.push_back(v);
    vv.push_back(pcl::Vertices());

    VecMat1f vm = to_(vv);

    EXPECT_SIZE(3, vm);

    EXPECT_TRUE(vm[0].empty());
    EXPECT_FLOAT_EQ(3.f, vm[1](0));
    EXPECT_TRUE(vm[2].empty());
}

TEST_F(VisitorVecMat1fTest, From_VecVertices_variable_size)
{
    const size_t N = 4;
    VecVertices vv;
    for(size_t i=1; i<=N; i++) {

        pcl::Vertices v;
        for(size_t j=0; j<i; j++) {

            v.vertices.push_back(randu<uint32_t>() % 256);
        }
        vv.push_back(v);
    }

    VecMat1f vm = to_(vv);

    EXPECT_SIZE(N, vm);

    for(size_t i=0; i<vm.size(); i++) {

        EXPECT_EQ(vm[i].total(), vv[i].vertices.size());
        for(size_t j=0; j<vm[i].total(); j++) {

            EXPECT_FLOAT_EQ(static_cast<float>(vv[i].vertices[j]), vm[i](j));
        }
    }
}

TEST_F(VisitorVecMat1fTest, Reset_with_VecVertices)
{
    EXPECT_NO_THROW(to_.Reset());

    Mat1f m(4, 3, 1.f);
    VecVertices vv = Mat2VecVertices(m);

    to_(vv);
    EXPECT_NO_THROW(to_.Reset());

    CloudXYZPtr cloud = Mat2PointCloud_<pcl::PointXYZ>(m);
    to_(cloud);
    EXPECT_NO_THROW(to_.Reset());

    to_(m);
    EXPECT_NO_THROW(to_.Reset());
}

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

#endif // __WITH_PCL

} // annonymous namespace for visitors' test fixtures

