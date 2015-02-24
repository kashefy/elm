/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file test out VecVertices visitor
 */
#ifdef __WITH_PCL // PCL support required for these tests

#include "elm/core/visitors/visitorvertices.h"

#include "elm/core/exception.h"
#include "elm/core/pcl/cloud_.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace elm;

namespace {

/**
 * @brief test class around VisitorVecVertices
 */
class VisitorVecVerticesTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.Reset();
    }

    VisitorVecVertices to_;    ///< test object
};

TEST_F(VisitorVecVerticesTest, EmptyVecVertices)
{
    EXPECT_TRUE(to_(VecVertices()).empty());
}

TEST_F(VisitorVecVerticesTest, EmptyVecVertices_Size)
{
    EXPECT_SIZE(0, to_(VecVertices()));
}

TEST_F(VisitorVecVerticesTest, Empty_VecMat1f)
{
    EXPECT_TRUE(to_(VecMat1f()).empty());
    EXPECT_TRUE(to_(VecMat1f(0)).empty());
}

TEST_F(VisitorVecVerticesTest, Empty_VecMat1f_empty_elements)
{
    EXPECT_SIZE(1, to_(VecMat1f(1, Mat1f())));
    to_.Reset();
    EXPECT_SIZE(2, to_(VecMat1f(2, Mat1f())));
}

TEST_F(VisitorVecVerticesTest, From_VecMat1f)
{
    const int N=5;
    VecMat1f vm;
    for(int i=0; i<N; i++) {

        vm.push_back(Mat1f(i+1, i+2, static_cast<float>(i)));
    }
    VecVertices vv = to_(vm);

    EXPECT_SIZE(N, vv);

    for(int i=0; i<N; i++) {

        EXPECT_SIZE((i+1)*(i+2), vv[i].vertices);

        for(size_t j=0; j<vv[i].vertices.size(); j++) {

            EXPECT_FLOAT_EQ(vm[i](j), static_cast<float>(vv[i].vertices[j]));
        }
    }
}

TEST_F(VisitorVecVerticesTest, Twos_cloudXYZ)
{
    CloudXYZPtr in = Mat2PointCloud_<PointXYZ>(Mat1f(4, 3, 2));

    VecVertices vv = to_(in);

    EXPECT_SIZE(4, vv);
    EXPECT_SIZE(4, vv[0].vertices);

    for(uint32_t i=0; i<in->height; i++) {
        for(uint32_t j=0; j<in->width; j++) {

            EXPECT_EQ(static_cast<uint32_t>(2), vv[i].vertices[j]);
        }
    }
}

template<class T>
class VisitorVecVerticesTypedTest : public VisitorVecVerticesTest
{
protected:
    virtual void SetUp()
    {
        to_.Reset();
    }

    //typedef std::list<T> List;
     static T shared_;
     T value_;
};

/**
 * @brief Value per type to use in fixtures below.
 */
template<class T>
struct Twos_
{
    static const T val;
};

///< Register static variables to work with inside tests
template<> const float Twos_<float>::val    = 2.f;
template<> const int Twos_<int>::val        = 2;
template<> const uchar Twos_<uchar>::val    = 2;
template<> const Mat1f Twos_<Mat1f>::val    = Mat1f(1, 1, 2.f);
template<> const SparseMat1f Twos_<SparseMat1f>::val = SparseMat1f(Mat1f(1, 1, 2.f));

typedef ::testing::Types<Mat1f, SparseMat1f, float, int, uchar> VisitorTypes;
TYPED_TEST_CASE(VisitorVecVerticesTypedTest, VisitorTypes);

TYPED_TEST(VisitorVecVerticesTypedTest, Twos)
{
    TypeParam in = Twos_<TypeParam >::val;

    VecVertices vv = to_(in);

    EXPECT_SIZE(1, vv);
    EXPECT_SIZE(1, vv[0].vertices);

    EXPECT_EQ(static_cast<uint32_t>(2), vv[0].vertices[0]);
}

} // annonymous namespace for VecVertices visitors' test fixtures

#endif // __WITH_PCL
