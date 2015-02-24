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
#include "elm/core/pcl/vertices.h"
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

//TEST_F(VisitorVecMat1fTest, EmptySparseMat1f)
//{
//    EXPECT_TRUE(to_(SparseMat1f()).empty());

//    SparseMat m(Mat1f(1, 1, 1.f));
//    m.clear();  // clearing is equivalent to setting elements to zero
//    EXPECT_FALSE(to_(m).empty());
//}

//TEST_F(VisitorVecMat1fTest, FromSparseMat1f)
//{
//    int N=5;

//    while(N--) {

//        const int MAT_ROWS = abs(randu<int>()) % 10 + 1;
//        const int MAT_COLS = abs(randu<int>()) % 100 + 1;

//        Mat1f m(MAT_ROWS, MAT_COLS);
//        randn(m, 0.f, 100.f);

//        Mat1f m2 = to_(SparseMat1f(m));

//        EXPECT_MAT_EQ(m, m2) << "Matrices are not equal";
//        EXPECT_NE(m.data, m2.data) << "Mats are not pointing to the same data in memory. Expecting shared copy.";
//    }
//}

//TEST_F(VisitorVecMat1fTest, Reset)
//{
//    EXPECT_NO_THROW(to_.Reset());

//    Mat1f m(4, 3);
//    randn(m, 0.f, 100.f);

//    Mat1f m2 = to_(m);
//    EXPECT_NO_THROW(to_.Reset());
//}

//#ifndef __WITH_PCL
//    #warning "Disabling unit tests that require PCL support."
//#else // __WITH_PCL

//TEST_F(VisitorVecMat1fTest, EmptyCLoud)
//{
//    CloudXYZPtr cld(new CloudXYZ);
//    EXPECT_TRUE(to_(cld).empty());
//}

//TEST_F(VisitorVecMat1fTest, EmptyCloudSize)
//{
//    CloudXYZPtr cld(new CloudXYZ);
//    EXPECT_MAT_DIMS_EQ(to_(cld), Size2i(0, 0));
//}

//TEST_F(VisitorVecMat1fTest, FromCloud)
//{
//    Mat1f m(4, 3);
//    randn(m, 0.f, 100.f);
//    CloudXYZPtr cloud = Mat2PointCloud_<pcl::PointXYZ>(m);

//    Mat1f m2 = to_(cloud);
//    hconcat(m, Mat1f(m.rows, 1, 1), m);

//    EXPECT_MAT_EQ(m, m2);
//    EXPECT_NE(m.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
//}

//TEST_F(VisitorVecMat1fTest, Reset_with_cloud)
//{
//    EXPECT_NO_THROW(to_.Reset());

//    Mat1f m(4, 3);
//    randn(m, 0.f, 100.f);

//    Mat1f m2 = to_(m);
//    EXPECT_NO_THROW(to_.Reset());

//    CloudXYZPtr cloud = Mat2PointCloud_<pcl::PointXYZ>(m);
//    m2 = to_(cloud);
//    EXPECT_NO_THROW(to_.Reset());

//    m2 = to_(m);
//    EXPECT_NO_THROW(to_.Reset());
//}

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

//#endif // __WITH_PCL

} // annonymous namespace for visitors' test fixtures

