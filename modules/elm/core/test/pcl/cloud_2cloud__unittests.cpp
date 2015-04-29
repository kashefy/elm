/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifdef __WITH_PCL // PCL support required for these tests
#include "elm/core/pcl/cloud_2cloud_.h"

#include "gtest/gtest.h"

#include "elm/core/exception.h"
#include "elm/ts/pcl_point_typed_tests.h"

using namespace cv;
using namespace pcl;
using namespace elm;

namespace {

template <class TPoint>
class Cloud_2Cloud_TypedTests : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        field_count_ = static_cast<int>(PCLPointTraits_<TPoint >::FieldCount());
        this->m_ = Mat1f(4, field_count_);
        randn(this->m_, 0.f, 100.f);
    }

    Mat1f m_;                       ///< fake data
    int field_count_;
};

TYPED_TEST_CASE(Cloud_2Cloud_TypedTests, PCLPointTypes);

TYPED_TEST(Cloud_2Cloud_TypedTests, Convert_same)
{
    Cloud_2Cloud_<TypeParam, TypeParam > to;     ///< test object
    typedef boost::shared_ptr<PointCloud<TypeParam > > CloudTPPtr;

    CloudTPPtr cld = Mat2PointCloud_<TypeParam >(this->m_);

    CloudTPPtr cld2;

    to.Convert(cld, cld2);

    EXPECT_EQ(cld->size(), cld2->size()) << "Size mismatch.";
    EXPECT_EQ(cld->width, cld2->width) << "Width mismatch.";
    EXPECT_EQ(cld->height, cld2->height) << "Height mismatch.";

    for(typename PointCloud<TypeParam >::iterator itr1=cld->begin(), itr2=cld2->begin();
        itr1 != cld->end();
        ++itr1, ++itr2) {

        TypeParam _p1 = *itr1;
        TypeParam _p2 = *itr2;

        float *pt_data_ptr1 = reinterpret_cast<float*>(&_p1);
        float *pt_data_ptr2 = reinterpret_cast<float*>(&_p2);

        EXPECT_NE(pt_data_ptr1, pt_data_ptr2) << "Pointers are pointing to the same memory location. Is this intentional?";

        for(int i=0; i<this->field_count_; i++) {

            EXPECT_FLOAT_EQ(pt_data_ptr1[i], pt_data_ptr2[i]) << "Unexpected value for coordinate i=" << i;
        }
    }
}

/**
 * @brief update for supported conversions
 */
TYPED_TEST(Cloud_2Cloud_TypedTests, Convert_not_supported)
{
    typedef boost::shared_ptr<PointCloud<TypeParam > > CloudTPPtr;

    CloudTPPtr cld = Mat2PointCloud_<TypeParam >(this->m_);

    size_t nb_floats_src = elm::PCLPointTraits_<TypeParam >::NbFloats();
    if(nb_floats_src != elm::PCLPointTraits_<PointXYZ >::NbFloats()) {

        CloudXYZPtr cld2;

        Cloud_2Cloud_<TypeParam, PointXYZ > to;     ///< test object

        if(nb_floats_src == elm::PCLPointTraits_<PointNormal >::NbFloats()) {

            EXPECT_NO_THROW(to.Convert(cld, cld2));
        }
        else {
            EXPECT_THROW(to.Convert(cld, cld2), ExceptionNotImpl);
        }
    }
    else if(nb_floats_src != elm::PCLPointTraits_<pcl::Normal >::NbFloats()) {

        CloudNrmlPtr cld2;
        Cloud_2Cloud_<TypeParam, Normal > to;     ///< test object
        EXPECT_THROW(to.Convert(cld, cld2), ExceptionNotImpl);
    }
    else if(nb_floats_src != elm::PCLPointTraits_<PointNormal >::NbFloats()) {

        CloudPtNrmlPtr cld2;
        Cloud_2Cloud_<TypeParam, PointNormal > to;     ///< test object
        EXPECT_THROW(to.Convert(cld, cld2), ExceptionNotImpl);
    }
}

class Cloud_2Cloud_PointNormalTruncationTests : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        this->m_ = Mat1f(4, PCLPointTraits_<PointNormal >::FieldCount());
        randn(this->m_, 0.f, 100.f);
    }

    Cloud_2Cloud_<PointNormal, PointXYZ > to_xyz_;     ///< test object
    Mat1f m_;                       ///< fake data
};

TEST_F(Cloud_2Cloud_PointNormalTruncationTests, PointNormalXYZTruncation) {

    CloudPtNrmlPtr cld = Mat2PointCloud_<PointNormal >(this->m_);
    CloudXYZPtr cld2;

    to_xyz_.Convert(cld, cld2);

    EXPECT_EQ(cld->size(), cld2->size()) << "Size mismatch.";
    EXPECT_EQ(cld->width, cld2->width) << "Width mismatch.";
    EXPECT_EQ(cld->height, cld2->height) << "Height mismatch.";

    typename PointCloud<PointNormal >::iterator itr1=cld->begin();
    typename PointCloud<PointXYZ >::iterator itr2=cld2->begin();

    for(;
        itr1 != cld->end();
        ++itr1, ++itr2) {

        PointNormal _p1 = *itr1;
        PointXYZ _p2 = *itr2;

        float *pt_data_ptr1 = reinterpret_cast<float*>(&_p1);
        float *pt_data_ptr2 = reinterpret_cast<float*>(&_p2);

        EXPECT_NE(pt_data_ptr1, pt_data_ptr2) << "Pointers are pointing to the same memory location. Is this intentional?";

        for(size_t i=0; i<PCLPointTraits_<PointXYZ >::FieldCount(); i++) {

            EXPECT_FLOAT_EQ(pt_data_ptr1[i], pt_data_ptr2[i]) << "Unexpected value for coordinate i=" << i;
        }
    }
}

} // annonymous namespace for unit tests

#endif // __WITH_PCL
