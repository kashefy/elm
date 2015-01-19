/** @file test out cloud visitor
 */
#ifdef __WITH_PCL // PCL support required for these tests
#include "sem/core/visitors/visitorcloud_.h"

#include "sem/core/exception.h"
#include "sem/core/pcl/vertices.h"
#include "sem/ts/ts.h"
#include "sem/ts/pcl_point_typed_tests.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace sem;
using namespace sem::ts;

namespace {

/**
 * @brief test class around VisitorCloud_ with different point types
 */
template <class TPoint>
class VisitorCloud_PointTypedTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        this->to_.Reset();

        this->m_ = Mat1f(4, PCLPointTraits_<TPoint >::FieldCount());
        randn(this->m_, 0.f, 100.f);
    }

    VisitorCloud_<TPoint > to_;     ///< test object
    Mat1f m_;                       ///< fake data
};
TYPED_TEST_CASE(VisitorCloud_PointTypedTest, PCLPointTypes);

TYPED_TEST(VisitorCloud_PointTypedTest, Empty)
{
    typedef boost::shared_ptr<PointCloud<TypeParam > > CloudTPPtr;

    EXPECT_TRUE(this->to_(Mat_f())->empty());
    EXPECT_TRUE(this->to_(Mat_f(0, 0))->empty());
    EXPECT_TRUE(this->to_(Mat_f(1, 0))->empty());
    EXPECT_TRUE(this->to_(Mat_f(0, 1))->empty());

    CloudTPPtr cld(new PointCloud<TypeParam >());
    EXPECT_TRUE(this->to_(cld)->empty());
}

TYPED_TEST(VisitorCloud_PointTypedTest, EmptySize)
{
    typedef boost::shared_ptr<PointCloud<TypeParam > > CloudTPPtr;

    EXPECT_EQ(size_t(0), this->to_(Mat_f())->size());
    EXPECT_EQ(size_t(0), this->to_(Mat_f(0, 0))->size());
    EXPECT_EQ(size_t(0), this->to_(Mat_f(1, 0))->size());
    EXPECT_EQ(size_t(0), this->to_(Mat_f(0, 1))->size());

    CloudTPPtr cld(new PointCloud<TypeParam >());
    EXPECT_EQ(size_t(0), this->to_(cld)->size());
}

TYPED_TEST(VisitorCloud_PointTypedTest, FromMat_f)
{
    typedef boost::shared_ptr<PointCloud<TypeParam > > CloudTPPtr;

    CloudTPPtr cld = this->to_(this->m_);
    Mat1f m2 = PointCloud2Mat_<TypeParam >(cld);

    int padding = static_cast<int>(PCLPointTraits_<TypeParam>::NbFloats()-PCLPointTraits_<TypeParam>::FieldCount());
    EXPECT_MAT_DIMS_EQ(m2, Size2i(this->m_.cols+padding, this->m_.rows));

    EXPECT_MAT_EQ(this->m_, m2.colRange(0, this->m_.cols));
    EXPECT_NE(this->m_.data, m2.data) << "Expecting deep copy. If intentionally optimized to be a shared copy, please update test.";
}

TYPED_TEST(VisitorCloud_PointTypedTest, Cloud)
{
    typedef boost::shared_ptr<PointCloud<TypeParam > > CloudTPPtr;

    CloudTPPtr cld = Mat2PointCloud_<TypeParam >(this->m_);

    CloudTPPtr cld2 = this->to_(cld);

    EXPECT_EQ(cld->size(), cld2->size()) << "Size mismatch.";
    EXPECT_EQ(cld->width, cld2->width) << "Width mismatch.";
    EXPECT_EQ(cld->height, cld2->height) << "Height mismatch.";

    for(typename PointCloud<TypeParam >::iterator itr1=cld->begin(), itr2=cld2->begin();
        itr1 != cld->end(); ++itr1, ++itr2) {

        TypeParam _p1 = *itr1;
        TypeParam _p2 = *itr2;

        float *pt_data_ptr1 = reinterpret_cast<float*>(&_p1);
        float *pt_data_ptr2 = reinterpret_cast<float*>(&_p2);

        EXPECT_NE(pt_data_ptr1, pt_data_ptr2) << "Pointers are pointing to the same memory location. Is this intentional?";

        for(size_t i=0; i<PCLPointTraits_<TypeParam >::FieldCount(); i++) {

            EXPECT_FLOAT_EQ(pt_data_ptr1[i], pt_data_ptr2[i]) << "Unexpected value for coordinate i=" << i;
        }
    }
}

TYPED_TEST(VisitorCloud_PointTypedTest, ResetValid)
{
    typedef boost::shared_ptr<PointCloud<TypeParam > > CloudTPPtr;

    CloudTPPtr cld1 = this->to_(this->m_);
    EXPECT_NO_THROW(this->to_.Reset());

    CloudTPPtr cld2 = Mat2PointCloud_<TypeParam >(this->m_);
    CloudTPPtr cld3 = this->to_(cld2);
    EXPECT_NO_THROW(this->to_.Reset());

    cld1 = this->to_(this->m_);
    EXPECT_NO_THROW(this->to_.Reset());
}

/**
 * @brief Test this visitor's caching
 */
TYPED_TEST(VisitorCloud_PointTypedTest, Reset)
{
    typedef boost::shared_ptr<PointCloud<TypeParam > > CloudTPPtr;

    this->m_.setTo(1.f);
    Mat1f m0 = this->m_.clone();

    CloudTPPtr cld = this->to_(this->m_);

    Mat1f mat_cld = PointCloud2Mat_<TypeParam >(cld);

    int padding = static_cast<int>(PCLPointTraits_<TypeParam>::NbFloats()-PCLPointTraits_<TypeParam>::FieldCount());
    EXPECT_MAT_DIMS_EQ(mat_cld, Size2i(m0.cols+padding, m0.rows));
    EXPECT_MAT_EQ(m0, mat_cld.colRange(0, m0.cols)) << "Not using cahced cloud.";

    cld = this->to_(this->m_); // result of first call is cached.

    const int N=2;
    ASSERT_GT(N, 1) << "This test must iterate at least twice to see effect of Reset() on cached reference.";
    for(int i=0; i<2; i++) {

        cld = this->to_(this->m_+1);

        if(i == 0) {

            mat_cld = PointCloud2Mat_<TypeParam >(cld);
            EXPECT_MAT_DIMS_EQ(mat_cld, Size2i(m0.cols+padding, m0.rows));
            EXPECT_MAT_EQ(m0, mat_cld.colRange(0, m0.cols)) << "Not using cahced cloud.";
        }
        else {

            Mat1f m2 = m0.clone();
            m2.colRange(0, m2.cols).setTo(2.f);

            mat_cld = PointCloud2Mat_<TypeParam >(cld);
            EXPECT_MAT_DIMS_EQ(mat_cld, Size2i(m2.cols+padding, m2.rows));
            EXPECT_MAT_EQ(m2, mat_cld.colRange(0, m2.cols)) << "Still using cached cloud";
        }
        this->to_.Reset();
    }
}

TYPED_TEST(VisitorCloud_PointTypedTest, FromVecVertices)
{
    typedef boost::shared_ptr<PointCloud<TypeParam > > CloudTPPtr;

    for(size_t i=0; i<this->m_.total(); i++) {

        this->m_(i) = static_cast<float>(randu<uint32_t>() % 256);
    }

    CloudTPPtr cld = this->to_(Mat2VecVertices(this->m_));

    Mat1f mat_cld = PointCloud2Mat_<TypeParam >(cld);

    int padding = static_cast<int>(PCLPointTraits_<TypeParam>::NbFloats()-PCLPointTraits_<TypeParam>::FieldCount());

    EXPECT_MAT_DIMS_EQ(mat_cld, Size2i(this->m_.cols+padding, this->m_.rows));
    EXPECT_MAT_EQ(this->m_, mat_cld.colRange(0, this->m_.cols));
    EXPECT_NE(this->m_.data, mat_cld.data) << "Expecting deep copy. " <<
                                              "If intentionally optimized to be a shared copy, please update test.";
}

} // annonymous namespace for cloud visitors' test fixtures

#else // __WITH_PCL
    #warning "Skipping building cloud visitor unit tests due to no pcl support."
#endif // __WITH_PCL
