/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/featuredata.h"

#include "elm/core/stl/stl.h"
#include "elm/core/exception.h"
#include "elm/ts/pcl_point_typed_tests.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace elm;

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
    }

    Mat_f mat_;
};


/**
 * @brief Initialize test object with Mat object, then call getters.
 */
TEST_F(FeatureDataTest, Init_Mat_f)
{
    FeatureData to(mat_);

    Mat_f m = to.get<Mat_f>();

    EXPECT_MAT_EQ(m, mat_);
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

#ifdef __WITH_PCL // following test case only applicable with PCL support and Point Cloud definitions

using namespace pcl;

/**
 * @brief Typed tests around FeatureData class with pcl PointCloud typed feature data
 */
template <class TPoint>
class FeatureDataCloud_Test : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        size_t field_count = elm::ts::ExpectedPointAttr_<TPoint>::field_count;
        size_t nb_floats = elm::ts::ExpectedPointAttr_<TPoint>::nb_floats;

        int cols = 2*field_count;
        while(cols % nb_floats == 0 && cols % field_count == 0) {
            cols += field_count;
        }

        mat_ = Mat1f(2, cols);
        randn(mat_, 0.f, 3.f);
        mat_ = Mat1i(mat_);

        cld_ = Mat2PointCloud_<TPoint >(mat_.clone());
        mat_ = PointCloud2Mat_(cld_).clone();
    }

    Mat_f mat_;
    boost::shared_ptr<pcl::PointCloud<TPoint > > cld_;
};

TYPED_TEST_CASE(FeatureDataCloud_Test, PCLPointTypes);

/**
 * @brief Initialize test object with Cloud object, then call getters.
 */
TYPED_TEST(FeatureDataCloud_Test, InitWithCloud)
{
    typedef boost::shared_ptr<pcl::PointCloud< TypeParam > > CloudTPPtr;

    FeatureData to(this->cld_);

    Mat_f m = to.get<Mat_f>();
    CloudTPPtr cld = to.get<CloudTPPtr >();

    EXPECT_MAT_EQ(m, this->mat_);
    EXPECT_MAT_EQ(PointCloud2Mat_<TypeParam>(cld), this->mat_);

    EXPECT_THROW(to.get<float >(), ExceptionTypeError);
    EXPECT_THROW(to.get<int >(), ExceptionTypeError);
}

/**
 * @brief Initialize test object with Mat object, then call cloud getters.
 */
TYPED_TEST(FeatureDataCloud_Test, InitWithMat_f)
{
    typedef boost::shared_ptr<pcl::PointCloud< TypeParam > > CloudTPPtr;

    FeatureData to(this->mat_);

    CloudTPPtr cld = to.get<CloudTPPtr >();

    EXPECT_MAT_EQ(PointCloud2Mat_<TypeParam >(cld), this->mat_);
}

/**
 * @brief Test caching of cloud reference
 * by comparing what the pointers are pointing at and verifying the use count increased by 1 after calling get
 */
TYPED_TEST(FeatureDataCloud_Test, Cached_Cloud)
{
    typedef boost::shared_ptr<pcl::PointCloud< TypeParam > > CloudTPPtr;

    FeatureData to(this->mat_);

    CloudTPPtr cld = to.get<CloudTPPtr >();
    EXPECT_NE(cld, this->cld_);
    long old_cld_use_count = cld.use_count();

    CloudTPPtr cld2 = to.get<CloudTPPtr >();
    EXPECT_NE(cld2, this->cld_);
    EXPECT_EQ(cld2, cld);

    // check use count increased
    EXPECT_EQ(cld2.use_count(), cld.use_count());
    EXPECT_EQ(cld2.use_count(), old_cld_use_count+1);
}

#endif // __WITH_PCL

/**
 * @brief Typed tests around FeatureData class with POD typed feature data
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

        // cloud types
        EXPECT_THROW( to.get<CloudXYZPtr >(), ExceptionTypeError );
        EXPECT_THROW( to.get<CloudNrmlPtr >(), ExceptionTypeError );
        EXPECT_THROW( to.get<CloudPtNrmlPtr >(), ExceptionTypeError );

        // vertices
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

template<typename TypeParam, class TPoint>
void Invalid_Cloud_Tests()
{
    int field_count = static_cast<int>(PCLPointTraits_<TPoint >::FieldCount());
    int nb_floats = static_cast<int>(PCLPointTraits_<TPoint >::NbFloats());

    // empty
    typename PointCloud<TPoint >::Ptr cld(new PointCloud<TPoint >);
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    // multi-row, columns are multiple of field count
    cld = Mat2PointCloud_<TPoint>(Mat1f(4, field_count, 0.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    cld = Mat2PointCloud_<TPoint>(Mat1f(4, field_count, 1.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    cld = Mat2PointCloud_<TPoint>(Mat1f(4, field_count, 2.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    // multi-row, columns are multiple of nb_floats
    cld = Mat2PointCloud_<TPoint>(Mat1f(4, nb_floats, 0.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    cld = Mat2PointCloud_<TPoint>(Mat1f(4, nb_floats, 1.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    cld = Mat2PointCloud_<TPoint>(Mat1f(4, nb_floats, 2.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    // single row, columns are multiple of field count
    cld = Mat2PointCloud_<TPoint>(Mat1f(1, field_count, 0.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    cld = Mat2PointCloud_<TPoint>(Mat1f(1, field_count, 1.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    // single row, columns are multiple of nb_floats
    cld = Mat2PointCloud_<TPoint>(Mat1f(1, nb_floats, 0.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);

    cld = Mat2PointCloud_<TPoint>(Mat1f(1, nb_floats, 1.f));
    EXPECT_THROW(FeatureData(cld).get<TypeParam >(), ExceptionTypeError);
}

TYPED_TEST(FeatureDataPOD_Test, Invalid_Cloud)
{
    Invalid_Cloud_Tests<TypeParam, PointXYZ >();
    Invalid_Cloud_Tests<TypeParam, Normal >();
    Invalid_Cloud_Tests<TypeParam, PointNormal >();
}

TYPED_TEST(FeatureDataPOD_Test, Invalid_VecVertices)
{
    // empty
    EXPECT_THROW(FeatureData(VecVertices()).get<TypeParam >(), ExceptionBadDims);

    // multi-row
    {
        pcl::Vertices v;
        v.vertices.push_back(1);
        VecVertices vv;
        vv.push_back(v);
        vv.push_back(v);
        EXPECT_THROW(FeatureData(vv).get<TypeParam >(), ExceptionBadDims);
    }

    // multi-col
    {
        pcl::Vertices v;
        v.vertices.push_back(1);
        v.vertices.push_back(2);
        VecVertices vv;
        vv.push_back(v);
        EXPECT_THROW(FeatureData(vv).get<TypeParam >(), ExceptionBadDims);
    }
}

#else // __WITH_PCL

TYPED_TEST(FeatureDataPOD_Test, DISABLED_Invalid_Cloud)
{
}

TYPED_TEST(FeatureDataPOD_Test, DISABLED_Invalid_VecVertices)
{
}

#endif // __WITH_PCL

} // annonymous namespace for test fixtures
