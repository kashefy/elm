#include "core/featuredata.h"

#include "core/exception.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

namespace {

/**
 * @brief Test FeatureData class
 * @todo define guards around PCL dependent tests
 */
class FeatureDataTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        mat_ = Mat_f(4, 3);
        randn(mat_, 0.f, 100.f);

        cld_ = Mat2PointCloud(mat_.clone());
        mat_ = PointCloud2Mat(cld_).clone();
    }

    Mat_f mat_;
    CloudXYZPtr cld_;
};

/**
 * @brief Initialize test object with Cloud object, then call getters.
 */
TEST_F(FeatureDataTest, Init_Cloud)
{
    FeatureData to(cld_);

    Mat_f m = to.get<Mat_f>();
    CloudXYZPtr cld = to.get<CloudXYZPtr>();

    EXPECT_MAT_EQ(m, mat_);
    EXPECT_MAT_EQ(PointCloud2Mat(cld), mat_);
}

/**
 * @brief Initialize test object with Mat object, then call getters.
 */
TEST_F(FeatureDataTest, Init_Mat_f)
{
    FeatureData to(mat_);

    Mat_f m = to.get<Mat_f>();
    CloudXYZPtr cld = to.get<CloudXYZPtr>();

    EXPECT_MAT_EQ(m, mat_);
    EXPECT_MAT_EQ(PointCloud2Mat(cld), mat_);
}

/**
 * @brief Verify initializing with Mat and getting a mat only involves a shared copy
 */
TEST_F(FeatureDataTest, Same_Mat)
{
    FeatureData to(mat_);

    Mat_f m = to.get<Mat_f>();
    EXPECT_MAT_EQ(m, mat_);
    EXPECT_EQ(m.data, mat_.data) << "Both Mats are not pointing to the same memory location.";
    ASSERT_NE(m.clone().data, mat_.data) << "Cannot still be pointing to the same memory chunk after cloning.";
}

/**
 * @brief Test caching of cloud reference
 * by comparing what the pointers are pointing at and verifying the use count increased by 1 after calling get
 */
TEST_F(FeatureDataTest, Cached_Cloud)
{
    FeatureData to(mat_);

    CloudXYZPtr cld = to.get<CloudXYZPtr>();
    EXPECT_NE(cld, cld_);
    long old_cld_use_count = cld.use_count();

    CloudXYZPtr cld2 = to.get<CloudXYZPtr>();
    EXPECT_NE(cld2, cld_);
    EXPECT_EQ(cld2, cld);

    // check use count increased
    EXPECT_EQ(cld2.use_count(), cld.use_count());
    EXPECT_EQ(cld2.use_count(), old_cld_use_count+1);
}

/**
 * @brief Type-Parameterized tests around FeatureData class with POD typed feature data
 */
template <class T>
class FeatureDataPOD_Test : public ::testing::Test
{
protected:
};
typedef ::testing::Types<float, int, uchar> PODTypes;
TYPED_TEST_CASE(FeatureDataPOD_Test, PODTypes);

TYPED_TEST(FeatureDataPOD_Test, FromMat_Invalid)
{
    // empty
    EXPECT_THROW(FeatureData(Mat_f()).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat_f(0, 0)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat_f(1, 0)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat_f(0, 1)).get<TypeParam >(), ExceptionBadDims);

    // multiple elements
    EXPECT_THROW(FeatureData(Mat_f(2, 1)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat_f(1, 2)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat_f(2, 2)).get<TypeParam >(), ExceptionBadDims);

    // multiple elements + initialized with scalar
    EXPECT_THROW(FeatureData(Mat_f(2, 1, 1.f)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat_f(1, 2, 0.f)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat_f(2, 2, 3.f)).get<TypeParam >(), ExceptionBadDims);
}

TYPED_TEST(FeatureDataPOD_Test, FromMat)
{
    float _v = 256; // to cover a range of values common between all of our PODs
    while(--_v >= 0) {

        FeatureData to(Mat1f(1, 1, _v));
        EXPECT_EQ(static_cast<TypeParam >(_v), to.get<TypeParam >()) << "Value mismatch.";
    }
}

TYPED_TEST(FeatureDataPOD_Test, cout)
{
    float vf = 3.f; // to cover a range of values common between all of our PODs
    TypeParam _v = static_cast<TypeParam >(vf); // to cover a range of values common between all of our PODs
    while(--vf >= 0.f) {

        FeatureData to(--_v);
        Mat1f m(1, 1, _v);

        stringstream ss1, ss2;
        ss1<<to<<endl;

        EXPECT_GT(ss1.str().size(), size_t(0)) << "Unexpectedly empty.";

        ss2<<m<<endl;

        EXPECT_EQ(ss1.str(), ss2.str()) << "Unexpected string in output stream.";
    }
}

/**
 * @todo: anyway to make this less manual?
 */
TYPED_TEST(FeatureDataPOD_Test, FromPOD)
{
    float vf = 256.f; // to cover a range of values common between all of our PODs
    TypeParam _v = static_cast<TypeParam >(vf); // to cover a range of values common between all of our PODs
    while(--vf >= 0.f) {

        FeatureData to(--_v);
        EXPECT_EQ( static_cast<float >(_v),  to.get<float>() )  << "Value mismatch while getting float.";
        EXPECT_EQ( static_cast<int >(_v),    to.get<int>()   )  << "Value mismatch while getting int.";
        EXPECT_EQ( static_cast<uchar >(_v),  to.get<uchar>() )  << "Value mismatch while getting uchar.";

        EXPECT_MAT_EQ( Mat_f(1, 1, static_cast<float >(_v)), to.get<Mat_f >()) << "Value mismatch while getting Mat_f.";

#ifdef __WITH_PCL
        EXPECT_THROW( to.get<CloudXYZPtr >(), ExceptionTypeError );

        {
            VecVertices vv = to.get<VecVertices >();
            EXPECT_SIZE(1, vv);
            EXPECT_SIZE(1, vv[0].vertices);

            EXPECT_EQ(static_cast<uint32_t>(_v), vv[0].vertices[0]);
        }
#endif // __WITH_PCL
    }
}

#ifdef __WITH_PCL // PCL support required for these tests

TYPED_TEST(FeatureDataPOD_Test, Invalid_Cloud)
{
    // empty
    CloudXYZPtr cld(new CloudXYZ);
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    // multi-row, 3-col
    cld = Mat2PointCloud(Mat1f(4, 3, 0.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    cld = Mat2PointCloud(Mat1f(4, 3, 1.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    cld = Mat2PointCloud(Mat1f(4, 3, 2.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    // single row
    cld = Mat2PointCloud(Mat1f(1, 3, 0.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    cld = Mat2PointCloud(Mat1f(1, 3, 1.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    // 3-channel
    cld = Mat2PointCloud(Mat3f(1, 1, 0.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    cld = Mat2PointCloud(Mat3f(1, 1, 1.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);
}

TYPED_TEST(FeatureDataPOD_Test, Invalid_VecVertices)
{
//    // empty
//    EXPECT_THROW(FeatureData(VecVertices()).get<TypeParam >(), ExceptionTypeError);

//    // multi-row
//    {
//        pcl::Vertices v;
//        v.vertices.push_back(1);
//        VecVertices vv;
//        vv.push_back(v);
//        vv.push_back(v);
//        EXPECT_THROW(FeatureData(vv).get<TypeParam >(), ExceptionBadDims);
//    }

//    // multi-col
//    {
//        pcl::Vertices v;
//        v.vertices.push_back(1);
//        v.vertices.push_back(2);
//        VecVertices vv;
//        vv.push_back(v);
//        EXPECT_THROW(FeatureData(vv).get<TypeParam >(), ExceptionBadDims);
//    }
}

#else // __WITH_PCL

TYPED_TEST(FeatureDataPOD_Test, DISABLED_Invalid_Cloud)
{
}

TYPED_TEST(FeatureDataPOD_Test, Invalid_VecVertices)
{
}

#endif // __WITH_PCL

} // annonymous namespace for test fixtures
