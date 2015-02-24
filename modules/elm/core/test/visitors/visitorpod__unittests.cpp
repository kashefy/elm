/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file test out POD visitors,
 * some fixtures may be scattered somewhat awkwardly in this file due to conditional support (e.g. PCL support)
 */
#include "elm/core/visitors/visitorpod_.h"

#include "elm/core/exception.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace elm;

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
    EXPECT_THROW(to(Mat1f()), ExceptionBadDims);
    EXPECT_THROW(to(Mat1f(0, 0)), ExceptionBadDims);
    EXPECT_THROW(to(Mat1f(1, 0)), ExceptionBadDims);
    EXPECT_THROW(to(Mat1f(0, 1)), ExceptionBadDims);

    // multiple elements
    EXPECT_THROW(to(Mat1f(2, 1)), ExceptionBadDims);
    EXPECT_THROW(to(Mat1f(1, 2)), ExceptionBadDims);
    EXPECT_THROW(to(Mat1f(2, 2)), ExceptionBadDims);

    EXPECT_THROW(to(Mat1f(2, 1, 1.f)), ExceptionBadDims);
    EXPECT_THROW(to(Mat1f(1, 2, 0.f)), ExceptionBadDims);
    EXPECT_THROW(to(Mat1f(2, 1, 3.f)), ExceptionBadDims);
}

TYPED_TEST_P(VisitorPOD_Test, Value_FromMat1f)
{
    VisitorPOD_<TypeParam > to;

    float _v = 256.f; // to cover a range of values common between all of our PODs
    while(--_v >= 0.f) {

        EXPECT_EQ(static_cast<TypeParam >(_v), to(Mat1f(1, 1, _v))) << "Value mismatch.";
    }
}

TYPED_TEST_P(VisitorPOD_Test, Invalid_SparseMat1f) {

    VisitorPOD_<TypeParam > to;

    // empty
    EXPECT_THROW(to(SparseMat1f()), ExceptionBadDims);

    // multiple elements
    {
        int sz[1] = {2};
        EXPECT_THROW(to(SparseMat1f(1, sz)), ExceptionBadDims);
    }
    {
        int sz[2] = {2, 1};
        EXPECT_THROW(to(SparseMat1f(2, sz)), ExceptionBadDims);
    }
    {
        int sz[2] = {1, 2};
        EXPECT_THROW(to(SparseMat1f(2, sz)), ExceptionBadDims);
    }
}

TYPED_TEST_P(VisitorPOD_Test, Value_FromSparseMat1f)
{
    VisitorPOD_<TypeParam > to;

    float _v = 256.f; // to cover a range of values common between all of our PODs
    while(--_v >= 0.f) {

        EXPECT_EQ(static_cast<TypeParam >(_v), to(SparseMat1f(Mat1f(1, 1, _v)))) << "Value mismatch.";
    }
}

TYPED_TEST_P(VisitorPOD_Test, Value_FromSparseMat1f_ndims)
{
    VisitorPOD_<TypeParam > to;

    float _v = 256.f; // to cover a range of values common between all of our PODs
    while(--_v >= 0.f) {

        for(int d=1; d<5; d++) {

            int *sz = new int[d];
            int *idx = new int[d];
            for(int i=0; i<d; i++) {
                sz[i] = 1;
                idx[i] = 0;
            }

            SparseMat1f m(d, sz);
            m.ref(idx) = _v;

            EXPECT_EQ(static_cast<TypeParam >(_v), to(m)) << "Value mismatch.";

            delete [] sz;
        }
    }
}

TYPED_TEST_P(VisitorPOD_Test, Invalid_VecMat1f) {

    VisitorPOD_<TypeParam > to;

    // empty
    EXPECT_THROW(to(VecMat1f()), ExceptionBadDims);

    // empty element
    EXPECT_THROW(to(VecMat1f(1, Mat1f())), ExceptionBadDims);

    // multiple empty elements
    EXPECT_THROW(to(VecMat1f(5, Mat1f())), ExceptionBadDims);

    // multiple elements
    EXPECT_THROW(to(VecMat1f(2, Mat1f(1, 1, 1.f))), ExceptionBadDims);

    EXPECT_THROW(to(VecMat1f(1, Mat1f(1, 2, 1.f))), ExceptionBadDims);
    EXPECT_THROW(to(VecMat1f(1, Mat1f(2, 1, 1.f))), ExceptionBadDims);
    EXPECT_THROW(to(VecMat1f(2, Mat1f(1, 2, 1.f))), ExceptionBadDims);
    EXPECT_THROW(to(VecMat1f(2, Mat1f(2, 1, 1.f))), ExceptionBadDims);
}

TYPED_TEST_P(VisitorPOD_Test, From_VecMat1f) {

    VisitorPOD_<TypeParam > to;

    float _v = 256.f; // to cover a range of values common between all of our PODs
    while(--_v >= 0.f) {

        EXPECT_EQ(static_cast<TypeParam >(_v),
                  to(VecMat1f(1, Mat1f(1, 1, _v))))
                << "Value mismatch.";
    }
}

#ifdef __WITH_PCL // PCL support required for these tests

TYPED_TEST_P(VisitorPOD_Test, Value_FromVecVertices)
{
    VisitorPOD_<TypeParam > to;

    float _v = 256.f; // to cover a range of values common between all of our PODs
    while(--_v >= 0.f) {

        pcl::Vertices v;
        v.vertices.push_back(static_cast<uint32_t>(_v));
        VecVertices vv;
        vv.push_back(v);

        EXPECT_EQ(static_cast<TypeParam >(_v), to(vv)) << "Value mismatch.";
    }
}

TYPED_TEST_P(VisitorPOD_Test, Invalid_Cloud)
{
    VisitorPOD_<TypeParam > to;

    // empty
    CloudXYZPtr cld(new CloudXYZ);
    EXPECT_THROW(to(cld), ExceptionTypeError);

    // multi-row, 3-col
    cld = Mat2PointCloud_<pcl::PointXYZ>(Mat1f(4, 3, 0.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    cld = Mat2PointCloud_<pcl::PointXYZ>(Mat1f(4, 3, 1.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    cld = Mat2PointCloud_<pcl::PointXYZ>(Mat1f(4, 3, 2.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    // single row
    cld = Mat2PointCloud_<pcl::PointXYZ>(Mat1f(1, 3, 0.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    cld = Mat2PointCloud_<pcl::PointXYZ>(Mat1f(1, 3, 1.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    // 3-channel
    cld = Mat2PointCloud_<pcl::PointXYZ>(Mat3f(1, 1, 0.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);

    cld = Mat2PointCloud_<pcl::PointXYZ>(Mat3f(1, 1, 1.f));
    EXPECT_THROW(to(cld), ExceptionTypeError);
}

TYPED_TEST_P(VisitorPOD_Test, Invalid_VecVertices)
{
    VisitorPOD_<TypeParam > to;

    // empty
    EXPECT_THROW(to(VecVertices()), ExceptionBadDims);

    // multi-row
    {
        pcl::Vertices v;
        v.vertices.push_back(1);
        VecVertices vv;
        vv.push_back(v);
        vv.push_back(v);
        EXPECT_THROW(to(vv), ExceptionBadDims);
    }

    // multi-col
    {
        pcl::Vertices v;
        v.vertices.push_back(1);
        v.vertices.push_back(2);
        VecVertices vv;
        vv.push_back(v);
        EXPECT_THROW(to(vv), ExceptionBadDims);
    }
}
#else // __WITH_PCL

TYPED_TEST_P(VisitorPOD_Test, Value_FromVecVertices)
{
}

TYPED_TEST_P(VisitorPOD_Test, Invalid_Cloud)
{
}

TYPED_TEST_P(VisitorPOD_Test, Invalid_VecVertices)
{
}

#endif // __WITH_PCL

/** Write additional type+value parameterized tests here.
 *  Acquaint yourself with the values passed to along with each type.
 *
 *  Register test names:
 */
REGISTER_TYPED_TEST_CASE_P(VisitorPOD_Test,
                           Invalid_Mat,
                           Value_FromMat1f,
                           Invalid_SparseMat1f,
                           Value_FromSparseMat1f,
                           Value_FromSparseMat1f_ndims,
                           Invalid_VecMat1f,
                           From_VecMat1f,
                           Value_FromVecVertices,
                           Invalid_Cloud,
                           Invalid_VecVertices
                           ); ///< register additional typed_test_p (i.e. unit test) routines here

typedef ::testing::Types<float, int, uchar> PODTypes;  ///< lists the usual suspects of plain old data types
INSTANTIATE_TYPED_TEST_CASE_P(FeatureDataVisitorPOD_Test, VisitorPOD_Test, PODTypes);

} // annonymous namespace for visitors' test fixtures
