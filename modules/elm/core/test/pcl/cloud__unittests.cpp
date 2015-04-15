/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/pcl/cloud_.h"

#ifdef __WITH_PCL // PCL support required for these tests

#include "elm/core/exception.h"
#include "elm/core/pcl/point_traits.h"
#include "elm/ts/pcl_point_typed_tests.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;
using namespace elm::ts;

namespace {

template<typename TPoint>
class PCL_Mat2PointCloud_Single_Ch_TypedTests : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        typedef PCLPointTraits_<TPoint > PTraits;

        this->field_count_i_ = static_cast<int>(PTraits::FieldCount());

        this->nb_floats_i_ = static_cast<int>(PTraits::NbFloats());
    }

    int field_count_i_;
    int nb_floats_i_;
};

TYPED_TEST_CASE(PCL_Mat2PointCloud_Single_Ch_TypedTests, PCLPointTypes);

TYPED_TEST(PCL_Mat2PointCloud_Single_Ch_TypedTests, Empty)
{
    EXPECT_TRUE(Mat2PointCloud_<TypeParam >(Mat1f())->empty());
    EXPECT_TRUE(Mat2PointCloud_<TypeParam >(Mat1f(1, 0))->empty());
    EXPECT_TRUE(Mat2PointCloud_<TypeParam >(Mat1f(0, 1))->empty());
    EXPECT_TRUE(Mat2PointCloud_<TypeParam >(Mat1f(0, 0))->empty());
}

TYPED_TEST(PCL_Mat2PointCloud_Single_Ch_TypedTests, EmptySize)
{
    EXPECT_EQ(size_t(0), Mat2PointCloud_<TypeParam >(Mat1f())->size());
    EXPECT_EQ(size_t(0), Mat2PointCloud_<TypeParam >(Mat1f(1, 0))->size());
    EXPECT_EQ(size_t(0), Mat2PointCloud_<TypeParam >(Mat1f(0, 1))->size());
    EXPECT_EQ(size_t(0), Mat2PointCloud_<TypeParam >(Mat1f(0, 0))->size());
}

TYPED_TEST(PCL_Mat2PointCloud_Single_Ch_TypedTests, Invalid)
{
    EXPECT_THROW(Mat2PointCloud_<TypeParam >(Mat1f::ones(1, 1)), ExceptionBadDims);
    EXPECT_THROW(Mat2PointCloud_<TypeParam >(Mat1f::ones(2, 1)), ExceptionBadDims);
    EXPECT_THROW(Mat2PointCloud_<TypeParam >(Mat1f::ones(1, 2)), ExceptionBadDims);
    EXPECT_THROW(Mat2PointCloud_<TypeParam >(Mat1f::ones(2, 2)), ExceptionBadDims);
    EXPECT_THROW(Mat2PointCloud_<TypeParam >(Mat1f::ones(5, 1)), ExceptionBadDims);
}

TYPED_TEST(PCL_Mat2PointCloud_Single_Ch_TypedTests, Dims_field_count)
{
    int n = 2;
    int cols = n*this->field_count_i_;
    while(cols % this->nb_floats_i_ == 0) {

        cols += this->field_count_i_;
        ++n;
    }

    Mat1f m(1, cols);
    randn(m, 0.f, 100.f);

    typename PointCloud<TypeParam >::Ptr cloud_ptr;

    cloud_ptr = Mat2PointCloud_<TypeParam >(m);
    EXPECT_EQ(m.total()/static_cast<size_t>(this->field_count_i_), cloud_ptr->size());
    EXPECT_EQ(n, static_cast<int>(cloud_ptr->width));
    EXPECT_EQ(1, static_cast<int>(cloud_ptr->height));

//    cloud_ptr = Mat2PointCloud_<PointXYZ>(m.reshape(1, 3));
//    EXPECT_EQ(m.total()/static_cast<size_t>(this->field_count_i_), cloud_ptr->size());
//    EXPECT_EQ(1, static_cast<int>(cloud_ptr->width));
//    EXPECT_EQ(3, static_cast<int>(cloud_ptr->height));

    Mat1f m2(4, this->field_count_i_);
    randn(m2, 0.f, 100.f);
    cloud_ptr = Mat2PointCloud_<TypeParam >(m2);
    EXPECT_EQ(4, static_cast<int>(cloud_ptr->size()));
    EXPECT_EQ(1, static_cast<int>(cloud_ptr->width));
    EXPECT_EQ(4, static_cast<int>(cloud_ptr->height));
}

TYPED_TEST(PCL_Mat2PointCloud_Single_Ch_TypedTests, Dims_nb_floats)
{
    typename PointCloud<TypeParam >::Ptr cloud_ptr;

    cloud_ptr = Mat2PointCloud_<TypeParam >(Mat1f::zeros(3, this->nb_floats_i_));
    EXPECT_EQ(3, static_cast<int>(cloud_ptr->size()));
    EXPECT_EQ(1, static_cast<int>(cloud_ptr->width));
    EXPECT_EQ(3, static_cast<int>(cloud_ptr->height));
}

TYPED_TEST(PCL_Mat2PointCloud_Single_Ch_TypedTests, IsOrganized)
{
    Mat1f m(9, 3*this->field_count_i_);
    randn(m, 0.f, 100.f);

    EXPECT_TRUE(Mat2PointCloud_<TypeParam >(m)->isOrganized());
    EXPECT_TRUE(Mat2PointCloud_<TypeParam >(m.reshape(1, 3))->isOrganized());
    EXPECT_FALSE(Mat2PointCloud_<TypeParam >(m.reshape(1, 1))->isOrganized()) << "Row matrices yield un-organized point clouds";
}

/**
 * @brief check values stored in non-organized point cloud after conversion
 */
TYPED_TEST(PCL_Mat2PointCloud_Single_Ch_TypedTests, ValuesNonOrganized)
{
    Mat1f m(1, 3*this->field_count_i_);
    randn(m, 0.f, 100.f);

    typename PointCloud<TypeParam >::Ptr cloud_ptr = Mat2PointCloud_<TypeParam >(m);

    int i=0;
    for(typename PointCloud<TypeParam >::iterator itr=cloud_ptr->begin(); itr != cloud_ptr->end(); ++itr) {

        TypeParam pt = *itr;
        float *pt_data_ptr = reinterpret_cast<float*>(&pt);
        for(int field_idx=0; field_idx<this->field_count_i_; field_idx++) {

            EXPECT_FLOAT_EQ(m(i++), pt_data_ptr[field_idx]) << "Unexpected value for field " << i << ".";
        }
    }
}

/**
 * @brief check values stored in organized point cloud after conversion
 */
TYPED_TEST(PCL_Mat2PointCloud_Single_Ch_TypedTests, Values_Organized)
{
    // organized point cloud
    Mat1f m(4, this->field_count_i_);
    randn(m, 0.f, 100.f);
    typename PointCloud<TypeParam >::Ptr cloud_ptr = Mat2PointCloud_<TypeParam >(m);

    for(size_t r=0; r<cloud_ptr->height; r++) {

        int i=0;
        for(size_t c=0; c<cloud_ptr->width; c++) {

            TypeParam pt = cloud_ptr->at(c, r);;
            float *pt_data_ptr = reinterpret_cast<float*>(&pt);

            for(int field_idx=0; field_idx<this->field_count_i_; field_idx++) {

                EXPECT_FLOAT_EQ(m(r, i++), pt_data_ptr[field_idx]) << "Unexpected value for field " << i << ".";
            }
        }
    }
}

// reverse conversion direction
TEST(PCLUtilsPointCloud_2MatTEST, Empty)
{
    CloudXYZPtr cloud_ptr;
    cloud_ptr.reset(new CloudXYZ());
    EXPECT_TRUE(PointCloud2Mat_<PointXYZ>(cloud_ptr).empty());

    cloud_ptr.reset(new CloudXYZ(0, 0));
    EXPECT_TRUE(PointCloud2Mat_<PointXYZ>(cloud_ptr).empty());

    cloud_ptr.reset(new CloudXYZ(0, 1));
    EXPECT_TRUE(PointCloud2Mat_<PointXYZ>(cloud_ptr).empty());

    cloud_ptr.reset(new CloudXYZ(1, 0));
    EXPECT_TRUE(PointCloud2Mat_<PointXYZ>(cloud_ptr).empty());
}


TEST(PCLUtilsPointCloud_2MatTEST, Dims)
{
    Mat1f m(9, 9);
    randn(m, 0.f, 100.f);

    CloudXYZPtr cloud_ptr;

    cloud_ptr = Mat2PointCloud_<PointXYZ>(m);
    EXPECT_MAT_DIMS_EQ(PointCloud2Mat_<PointXYZ>(cloud_ptr), Size2i(12, m.rows));

    cloud_ptr = Mat2PointCloud_<PointXYZ>(m.reshape(1, 27));
    EXPECT_MAT_DIMS_EQ(PointCloud2Mat_<PointXYZ>(cloud_ptr), Size2i(4, 27));
}

TEST(PCLUtilsPointCloud_2MatTEST, Values)
{
    // organized point cloud
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZPtr cloud_ptr = Mat2PointCloud_<PointXYZ>(m);

    Mat1f m2 = PointCloud2Mat_<PointXYZ>(cloud_ptr);
    hconcat(m, Mat1f(m.rows, 1, 1), m);

    EXPECT_MAT_EQ(m, m2);
}

/**
 * @brief Verify that data ownership remains with source point cloud
 */
TEST(PCLUtilsPointCloud_2MatTEST, Owner)
{
    // organized point cloud
    Mat1f m(4, 3);
    randn(m, 0.f, 100.f);
    CloudXYZPtr cloud_ptr = Mat2PointCloud_<PointXYZ>(m);

    Mat1f m2 = PointCloud2Mat_<PointXYZ>(cloud_ptr);

    for(size_t r=0; r<cloud_ptr->height; r++) {

        int i=0;
        for(size_t c=0; c<cloud_ptr->width; c++, i+=4) {

            PointXYZ _p = cloud_ptr->at(c, r);
            EXPECT_FLOAT_EQ(m2(r, i)  , _p.x) << "Unexpected value for x coordinate.";
            EXPECT_FLOAT_EQ(m2(r, i+1), _p.y) << "Unexpected value for y coordinate.";
            EXPECT_FLOAT_EQ(m2(r, i+2), _p.z) << "Unexpected value for z coordinate.";

            PointXYZ _p2;
            _p2.x += 1.f;
            _p2.y += 2.f;
            _p2.z += 3.f;

            cloud_ptr->at(c, r) = _p2;

            EXPECT_FLOAT_EQ(m2(r, i)  , _p2.x) << "Unexpected value for x coordinate.";
            EXPECT_FLOAT_EQ(m2(r, i+1), _p2.y) << "Unexpected value for y coordinate.";
            EXPECT_FLOAT_EQ(m2(r, i+2), _p2.z) << "Unexpected value for z coordinate.";
        }
    }
}

} // annonymous namespace around tests

#endif // __WITH_PCL
