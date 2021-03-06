/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/featuredata.h"

#include "elm/core/debug_utils.h"
#include "elm/core/stl/stl_inl.h"
#include "elm/core/exception.h"
#include "elm/ts/pcl_point_typed_tests.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

/**
 * @brief For testing FeatureData's protected default constructor
 */
class FeatureDataProtected : public elm::FeatureData
{
public:
    FeatureDataProtected()
        : FeatureData()
    {}

    template <class T>
    FeatureDataProtected(const T &t)
        : FeatureData(t)
    {}

    void Init()
    {
        FeatureData::Init();
    }

    void Reset()
    {
        FeatureData::Reset();
    }
};

TEST(FeatureDataProtectedTest, Constructor)
{
    EXPECT_NO_THROW(FeatureDataProtected to);
}

TEST(FeatureDataProtectedTest, Init)
{
    FeatureDataProtected to;
    EXPECT_NO_THROW(to.Init());
}

TEST(FeatureDataProtectedTest, Reset)
{
    {
        FeatureDataProtected to;
        to.Init();
        to.Reset();
        to.Reset();
        to.Reset();
    }

    {
        FeatureDataProtected to;
        to.Reset();
        to.Reset();
        to.Reset();
    }
}

/**
 * @brief Test FeatureData class
 */
class FeatureDataTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        mat_ = Mat1f(4, 3);
        randn(mat_, 0.f, 100.f);
    }

    Mat1f mat_;
};

/**
 * @brief Initialize test object with Mat object, then call getters.
 */
TEST_F(FeatureDataTest, Init_Mat1f)
{
    FeatureData to(mat_);

    Mat1f m = to.get<Mat1f>();

    EXPECT_MAT_EQ(m, mat_);
}

TEST_F(FeatureDataTest, Init_Mat1f_get_VecMat1f)
{
    FeatureData to(mat_);

    EXPECT_SIZE(1, to.get<VecMat1f>());
    Mat1f m = to.get<VecMat1f>()[0];

    EXPECT_MAT_EQ(m, mat_);
}

TEST_F(FeatureDataTest, InitWithSparseMat1f)
{
    SparseMat1f m_sparse(mat_);
    FeatureData to(m_sparse);

    Mat1f m = to.get<Mat1f>();

    EXPECT_MAT_EQ(m, mat_);
}

TEST_F(FeatureDataTest, InitWithSparseMat1f_get_VecMat1f)
{
    SparseMat1f m_sparse(mat_);
    FeatureData to(m_sparse);

    EXPECT_SIZE(1, to.get<VecMat1f>());
    Mat1f m = to.get<VecMat1f>()[0];

    EXPECT_MAT_EQ(m, mat_);
}

TEST_F(FeatureDataTest, InitWithVecMat1f)
{
    FeatureData to(VecMat1f(1, mat_));

    Mat1f m = to.get<Mat1f>();

    EXPECT_MAT_EQ(m, mat_);

    m = to.get<VecMat1f>()[0];

    EXPECT_MAT_EQ(m, mat_);
}

/**
 * @brief Verify initializing with Mat and getting a mat only involves a shared copy
 */
TEST_F(FeatureDataTest, Same_Mat)
{
    FeatureData to(mat_);

    Mat1f m = to.get<Mat1f>();
    EXPECT_MAT_EQ(m, mat_);
    EXPECT_EQ(m.data, mat_.data) << "Both Mats are not pointing to the same memory location.";
    ASSERT_NE(m.clone().data, mat_.data) << "Cannot still be pointing to the same memory chunk after cloning.";
}

TEST_F(FeatureDataTest, VecMat1f_Same_Mat)
{
    FeatureData to(VecMat1f(1, mat_));

    {
        Mat1f m = to.get<Mat1f>();
        EXPECT_MAT_EQ(m, mat_);
        EXPECT_EQ(m.data, mat_.data) << "Both Mats are not pointing to the same memory location.";
        ASSERT_NE(m.clone().data, mat_.data) << "Cannot still be pointing to the same memory chunk after cloning.";
    }
    {
        Mat1f m = to.get<VecMat1f>()[0];
        EXPECT_MAT_EQ(m, mat_);
        EXPECT_EQ(m.data, mat_.data) << "Both Mats are not pointing to the same memory location.";
        ASSERT_NE(m.clone().data, mat_.data) << "Cannot still be pointing to the same memory chunk after cloning.";
    }
}

#ifdef __WITH_PCL // following test case only applicable with PCL support and Point Cloud definitions

using namespace pcl;

TEST(FeatureDataProtectedTest, Reset_valid)
{
    Mat1f m = Mat1f(4, 3);
    randn(m, 0.f, 100.f);

    typedef boost::shared_ptr<PointCloud<PointXYZ > > CloudTPPtr;

    FeatureDataProtected to(m);
    CloudTPPtr cld1 = to.get<CloudTPPtr>();
    EXPECT_NO_THROW(to.Reset());
}

/**
 * @brief Typed tests around FeatureData class with pcl PointCloud typed feature data
 */
template <class TPoint>
class FeatureDataCloud_Test : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        field_count_ = static_cast<int>(elm::ts::ExpectedPointAttr_<TPoint>::field_count);
        nb_floats_ = static_cast<int>(elm::ts::ExpectedPointAttr_<TPoint>::nb_floats);

        int cols = 2*field_count_;
        while(cols % nb_floats_ == 0 && cols % field_count_ == 0) {

            cols += field_count_;
        }

        mat_ = Mat1f(2, cols);
        randn(mat_, 0.f, 3.f);
        mat_ = static_cast<Mat1f>(Mat1i(mat_));

        cld_ = Mat2PointCloud_<TPoint >(mat_.clone());
        mat_ref_ = PointCloud2Mat_(cld_).clone();
    }

    virtual void TearDown()
    {
        cld_.reset();
    }

    Mat1f mat_;
    Mat1f mat_ref_;
    boost::shared_ptr<pcl::PointCloud<TPoint > > cld_;

    int field_count_;
    int nb_floats_;
};

TYPED_TEST_CASE(FeatureDataCloud_Test, PCLPointTypes);

/**
 * @brief Initialize test object with Cloud object, then call getters.
 */
TYPED_TEST(FeatureDataCloud_Test, InitWithCloud)
{
    typedef boost::shared_ptr<pcl::PointCloud< TypeParam > > CloudTPPtr;

    FeatureData to(this->cld_);

    Mat1f m = to.get<Mat1f>();
    CloudTPPtr cld = to.get<CloudTPPtr >();

    EXPECT_MAT_EQ(m, this->mat_ref_);
    EXPECT_MAT_EQ(PointCloud2Mat_<TypeParam>(cld), m);

    EXPECT_THROW(to.get<float >(), ExceptionTypeError);
    EXPECT_THROW(to.get<int >(), ExceptionTypeError);
}

/**
 * @brief Initialize test object with Mat object, then call cloud getters.
 */
TYPED_TEST(FeatureDataCloud_Test, InitWithMat1f)
{
    typedef boost::shared_ptr<pcl::PointCloud< TypeParam > > CloudTPPtr;

    FeatureData to(this->mat_);

    CloudTPPtr cld = to.get<CloudTPPtr >();

    Mat1f m = PointCloud2Mat_<TypeParam >(cld);

    int new_rows = static_cast<int>(m.total())/this->nb_floats_;
    int new_cols = this->field_count_;
    m = m.reshape(1, new_rows).colRange(1, new_cols);
    this->mat_ref_ = this->mat_ref_.reshape(1, new_rows).colRange(1, new_cols);

    EXPECT_MAT_EQ(m, this->mat_ref_);
}

/**
 * @brief Initialize test object with Mat object, then call cloud getters.
 */
TYPED_TEST(FeatureDataCloud_Test, InitWithSparseMat1f)
{
    typedef boost::shared_ptr<pcl::PointCloud< TypeParam > > CloudTPPtr;

    FeatureData to(SparseMat1f(this->mat_));

    CloudTPPtr cld = to.get<CloudTPPtr >();

    Mat1f m = PointCloud2Mat_<TypeParam >(cld);

    int new_rows = static_cast<int>(m.total())/this->nb_floats_;
    int new_cols = this->field_count_;
    m = m.reshape(1, new_rows).colRange(1, new_cols);
    this->mat_ref_ = this->mat_ref_.reshape(1, new_rows).colRange(1, new_cols);

    EXPECT_MAT_EQ(m, this->mat_ref_);
}

/**
 * @brief Initialize test object with VecMat1f object, then call cloud getters.
 */
TYPED_TEST(FeatureDataCloud_Test, InitWithVecMat1f)
{
    typedef boost::shared_ptr<pcl::PointCloud< TypeParam > > CloudTPPtr;

    // VecMat1f to Cloud conversion assumes 1 point per VecMat1f element

    this->mat_ = this->mat_.reshape(1, this->mat_.total()/this->field_count_);

    VecMat1f vm;
    for(int r=0; r<this->mat_.rows; r++) {

        vm.push_back(this->mat_.row(r));
    }

    FeatureData to(vm);

    // VecMat1f to Cloud conversion results in unorganized point cloud
    // so we'll need to reshape before comparing.
    CloudTPPtr cld = to.get<CloudTPPtr >();

    Mat1f m = PointCloud2Mat_<TypeParam >(cld);
    int new_rows = static_cast<int>(m.total())/this->nb_floats_;
    int new_cols = this->field_count_;
    m = m.reshape(1, new_rows).colRange(1, new_cols);
    this->mat_ref_ = this->mat_ref_.reshape(1, new_rows).colRange(1, new_cols);

    EXPECT_MAT_EQ(m, this->mat_ref_);
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

TYPED_TEST(FeatureDataCloud_Test, Cached_Cloud_protected_reset)
{
    typedef boost::shared_ptr<pcl::PointCloud< TypeParam > > CloudTPPtr;

    FeatureDataProtected to(this->mat_);

    CloudTPPtr cld = to.get<CloudTPPtr >();
    EXPECT_NE(cld, this->cld_);
    long old_cld_use_count = cld.use_count();

    to.Reset();

    CloudTPPtr cld2 = to.get<CloudTPPtr >();

    EXPECT_EQ(old_cld_use_count-1, cld.use_count()) << "use count did not decrease.";

    EXPECT_NE(cld2, this->cld_);
    EXPECT_NE(cld2, cld) << "Still same object after reset";

    // but we expect a new copy of the same data
    EXPECT_EQ(this->cld_->height, cld2->height) << "Unexpected height after reset";
    EXPECT_EQ(this->cld_->width, cld2->width) << "Unexpected width after reset";

    Mat1f m = PointCloud2Mat_<TypeParam >(cld2);
    int new_rows = static_cast<int>(m.total())/this->nb_floats_;
    int new_cols = this->field_count_;
    m = m.reshape(1, new_rows).colRange(1, new_cols);
    this->mat_ref_ = this->mat_ref_.reshape(1, new_rows).colRange(1, new_cols);

    EXPECT_MAT_EQ(m, this->mat_ref_) << "data mismatch after reset";
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
    EXPECT_THROW(FeatureData(Mat1f()).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat1f(0, 0)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat1f(1, 0)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat1f(0, 1)).get<TypeParam >(), ExceptionBadDims);

    // multiple elements
    EXPECT_THROW(FeatureData(Mat1f(2, 1)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat1f(1, 2)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat1f(2, 2)).get<TypeParam >(), ExceptionBadDims);

    // multiple elements + initialized with scalar
    EXPECT_THROW(FeatureData(Mat1f(2, 1, 1.f)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat1f(1, 2, 0.f)).get<TypeParam >(), ExceptionBadDims);
    EXPECT_THROW(FeatureData(Mat1f(2, 2, 3.f)).get<TypeParam >(), ExceptionBadDims);
}

TYPED_TEST(FeatureDataPOD_Test, FromMat1f)
{
    float _v = 256.f; // to cover a range of values common between all of our PODs
    while(--_v >= 0.f) {

        FeatureData to(Mat1f(1, 1, _v));
        EXPECT_EQ(static_cast<TypeParam >(_v), to.get<TypeParam >()) << "Value mismatch.";
    }
}

TYPED_TEST(FeatureDataPOD_Test, FromSparseMat1f)
{
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

            FeatureData to(m);

            EXPECT_EQ(static_cast<TypeParam >(_v), to.get<TypeParam >()) << "Value mismatch.";

            delete [] sz;
        }
    }
}

TYPED_TEST(FeatureDataPOD_Test, FromVecMat1f)
{
    float _v = 256.f; // to cover a range of values common between all of our PODs
    while(--_v >= 0.f) {

        FeatureData to(VecMat1f(1, Mat1f(1, 1, _v)));
        EXPECT_EQ(static_cast<TypeParam >(_v), to.get<TypeParam >()) << "Value mismatch.";
    }
}

TYPED_TEST(FeatureDataPOD_Test, FromVecMat1f_invalid)
{
    {
        FeatureData to(VecMat1f(2, Mat1f(1, 1, 0.f)));
        EXPECT_THROW(to.get<TypeParam >(), ExceptionBadDims);
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

        EXPECT_MAT_EQ( Mat1f(1, 1, static_cast<float >(_v)), to.get<Mat1f >()) << "Value mismatch while getting Mat1f.";

        EXPECT_MAT_EQ( Mat1f(1, 1, static_cast<float >(_v)), to.get<VecMat1f >()[0]) << "Value mismatch while getting VecMat1f.";

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
