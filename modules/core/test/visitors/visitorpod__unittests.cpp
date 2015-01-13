/** @file test out POD visitors,
 * some fixtures may be scattered somewhat awkwardly in this file due to conditional support (e.g. PCL support)
 */
#include "core/visitors/visitorpod_.h"

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

#ifdef __WITH_PCL // PCL support required for these tests

TYPED_TEST_P(VisitorPOD_Test, Invalid_Cloud)
{
    VisitorPOD_<TypeParam > to;

    // empty
    CloudXYZPtr cld(new CloudXYZ);
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

TYPED_TEST_P(VisitorPOD_Test, DISABLED_Invalid_Cloud)
{
}

TYPED_TEST_P(VisitorPOD_Test, DISABLED_Invalid_VecVertices)
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
                           Value,
                           Value_FromVecVertices,
                           Invalid_Cloud,
                           Invalid_VecVertices
                           ); ///< register additional typed_test_p (i.e. unit test) routines here

typedef ::testing::Types<float, int, uchar> PODTypes;  ///< lists the usual suspects of plain old data types
INSTANTIATE_TYPED_TEST_CASE_P(FeatureDataVisitorPOD_Test, VisitorPOD_Test, PODTypes);

} // annonymous namespace for visitors' test fixtures
