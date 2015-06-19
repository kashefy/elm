/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/pcl/cloud_impl_.h"

#ifdef __WITH_PCL // following test cases require PCL support

#include "elm/core/pcl/point_traits.h"
#include "elm/ts/pcl_point_typed_tests.h"
#include "elm/ts/mat_assertions.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;
using namespace elm::ts;

namespace {

ELM_SET_EXPECTED_POINT_ATTR(pcl::PointXYZ,    3,  4);  // x, y, z
ELM_SET_EXPECTED_POINT_ATTR(pcl::Normal,      4,  8);  // n1, n2, n3, curvature
ELM_SET_EXPECTED_POINT_ATTR(pcl::PointNormal, 7, 12);  // PointXYZ + Normal = 4+3

/**
 * @brief test Mat to Point cloud conversion with multi-channel Mat input.
 * Since OpenCV doesn't allow creation of Mat objects with > 4 channels, we'll limit tests to an appropriate subset
 * OpenCV enforces this through the following assertion:
 *  OpenCV Error: Assertion failed (cn <= 4) in scalarToRawData,
 *  file /media/206CDC456CDC177E/Users/woodstock/dev/src/opencv/modules/core/src/matrix.cpp,
 *  line 1036
 *
 * We use a static assertion to validate a type at compile time (see SetUp() method)
 */
template<typename TPoint>
class PCL_Cloud_T_Conversion_Multi_Ch_TypedTests : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        /** Sanity check that our subset of PointTypes is fit for mutli-channel tests
         */
        static_assert(ExpectedPointAttr_<TPoint >::field_count <= 4,
                      "No. of fields exceed max no. of channels allowed by OpenCV.");

        typedef PCLPointTraits_<TPoint > PTraits;

        field_count_sz_ = PTraits::FieldCount();
        field_count_i_ = static_cast<int>(field_count_sz_);

        nb_floats_sz_ = PTraits::NbFloats();
        nb_floats_i_ = static_cast<int>(nb_floats_sz_);
    }

    size_t field_count_sz_;
    int field_count_i_;

    size_t nb_floats_sz_;
    int nb_floats_i_;
};
typedef ::testing::Types<PointXYZ, Normal> PCLPointTypes_LE4Ch;

TYPED_TEST_CASE(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, PCLPointTypes_LE4Ch);

/**
 * @brief The function checks for empty Mat empty on, it doesn't look much into if channels are sufficient for conversion
 * This test will tell us if that assumption should ever be reconsidered.
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Empty)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    EXPECT_TRUE(C_::Mat2PointCloud(Mat2f())->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat2f(1, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat2f(0, 1))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat2f(0, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat2f(3, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat2f(0, 3))->empty());

    EXPECT_TRUE(C_::Mat2PointCloud(Mat3f())->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat3f(1, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat3f(0, 1))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat3f(0, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat3f(3, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat3f(0, 3))->empty());

    EXPECT_TRUE(C_::Mat2PointCloud(Mat4f())->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat4f(1, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat4f(0, 1))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat4f(0, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat4f(3, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat4f(0, 3))->empty());

    typedef Mat_<Vec<float, ExpectedPointAttr_<TypeParam >::field_count > > Matnf;

    EXPECT_TRUE(C_::Mat2PointCloud(Matnf())->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Matnf(1, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Matnf(0, 1))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Matnf(0, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Matnf(3, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Matnf(0, 3))->empty());
}

TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Empty_Size)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat2f())->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat2f(1, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat2f(0, 1))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat2f(0, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat2f(3, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat2f(0, 3))->size());

    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat3f())->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat3f(1, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat3f(0, 1))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat3f(0, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat3f(3, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat3f(0, 3))->size());

    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat4f())->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat4f(1, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat4f(0, 1))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat4f(0, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat4f(3, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat4f(0, 3))->size());

    typedef Mat_<Vec<float, ExpectedPointAttr_<TypeParam >::field_count > > Matnf;

    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Matnf())->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Matnf(1, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Matnf(0, 1))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Matnf(0, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Matnf(3, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Matnf(0, 3))->size());
}

TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Nb_channels_invalid)
{
    typedef ConverterCloudMat_<TypeParam > C_;
    EXPECT_THROW(C_::Mat2PointCloud(Mat_<Vec<float, 2> >(4, 1, 1.f)), ExceptionBadDims);
}

/**
 * @brief Multi-channel Mat where no. of channels are equal to Point type's field count
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Dims_field_count)
{
    typedef ConverterCloudMat_<TypeParam > C_;
    typedef Mat_<Vec<float, ExpectedPointAttr_<TypeParam >::field_count > > Matnf;

    for(int r=1; r<7; r++) {

        for(int c=1; c<7; c++) {

            Matnf m(r, c, 1.f);
            typename PointCloud<TypeParam >::Ptr cloud_ptr;

            cloud_ptr = C_::Mat2PointCloud(m);
            EXPECT_EQ(m.total(), cloud_ptr->size());
            EXPECT_EQ(c, static_cast<int>(cloud_ptr->width));
            EXPECT_EQ(r, static_cast<int>(cloud_ptr->height));

            // reshape returns a Mat wich invokes the single-channel overload. Therefore, we need an explicit cast.
            cloud_ptr = C_::Mat2PointCloud(static_cast<Matnf>(m.reshape(m.channels(), 1)));
            EXPECT_EQ(m.total(), cloud_ptr->size());
            EXPECT_EQ(c*r, static_cast<int>(cloud_ptr->width));
            EXPECT_EQ(1, static_cast<int>(cloud_ptr->height));
        }
    }
}

/**
 * @brief Multi-channel Mat where no. of channels are equal to Point type's nb_floats
 * Not applied for Point types with nb_floats > 4. This is OpenCV's upper limit on no. of channels.
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Dims_nb_floats)
{
    typedef ConverterCloudMat_<TypeParam > C_;
    if(ExpectedPointAttr_<TypeParam >::nb_floats <= 4) {

        typedef Mat_<Vec<float, ExpectedPointAttr_<TypeParam >::nb_floats > > Matnf;
        for(int r=1; r<7; r++) {

            for(int c=1; c<7; c++) {

                Matnf m(r, c, 1.f);
                typename PointCloud<TypeParam >::Ptr cloud_ptr;

                cloud_ptr = C_::Mat2PointCloud(m);
                EXPECT_EQ(m.total(), cloud_ptr->size());
                EXPECT_EQ(c, static_cast<int>(cloud_ptr->width));
                EXPECT_EQ(r, static_cast<int>(cloud_ptr->height));

                // reshape returns a Mat wich invokes the single-channel overload. Therefore, we need an explicit cast.
                cloud_ptr = C_::Mat2PointCloud(static_cast<Matnf>(m.reshape(m.channels(), 1)));
                EXPECT_EQ(m.total(), cloud_ptr->size());
                EXPECT_EQ(c*r, static_cast<int>(cloud_ptr->width));
                EXPECT_EQ(1, static_cast<int>(cloud_ptr->height));
            }
        }
    }
}

TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, IsOrganized)
{
    typedef ConverterCloudMat_<TypeParam > C_;
    typedef Mat_<Vec<float, ExpectedPointAttr_<TypeParam >::field_count > > Matnf;
    Matnf m(9, 9, 1.f);

    EXPECT_TRUE(C_::Mat2PointCloud(m)->isOrganized());
    EXPECT_TRUE(C_::Mat2PointCloud(m.reshape(m.channels(), 3))->isOrganized());
    EXPECT_FALSE(C_::Mat2PointCloud(m.reshape(m.channels(), 1))->isOrganized()) << "Row matrices yield un-organized point clouds";
}

/**
 * @brief check values stored in non-organized point cloud after conversion
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Values_NonOrganized)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    const int CLOUD_COLS=3;

    Mat_<Vec<float, ExpectedPointAttr_<TypeParam >::field_count > > m(9, CLOUD_COLS, 1.f);
    randn(m, 0.f, 100.f);
    typename PointCloud<TypeParam >::Ptr cloud_ptr = C_::Mat2PointCloud(m);

    int i=0;
    for(typename PointCloud<TypeParam >::iterator itr=cloud_ptr->begin(); itr != cloud_ptr->end(); ++itr) {

        TypeParam pt = *itr;
        float *pt_data_ptr = reinterpret_cast<float*>(&pt);

        Vec<float, ExpectedPointAttr_<TypeParam >::field_count > v = m(i++);

        for(int field_idx=0; field_idx<this->field_count_i_; field_idx++) {

            EXPECT_FLOAT_EQ(v[field_idx], pt_data_ptr[field_idx]) << "Unexpected value for field " << i << ".";
        }
    }
}

/**
 * @brief check values stored in organized point cloud after conversion
 * no. of channels equal to Point type field count
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Values_Organized_field_count)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    const int CLOUD_COLS=3;

    // organized point cloud
    Mat_<Vec<float, ExpectedPointAttr_<TypeParam >::field_count > > m(9, CLOUD_COLS, 1.f);
    randn(m, 0.f, 100.f);
    typename PointCloud<TypeParam >::Ptr cloud_ptr = C_::Mat2PointCloud(m);

    for(size_t r=0; r<cloud_ptr->height; r++) {

        for(size_t c=0; c<cloud_ptr->width; c++) {

            TypeParam pt = cloud_ptr->at(c, r);;
            float *pt_data_ptr = reinterpret_cast<float*>(&pt);

            Vec<float, ExpectedPointAttr_<TypeParam >::field_count > v = m(r, c);

            for(int field_idx=0; field_idx<this->field_count_i_; field_idx++) {

                EXPECT_FLOAT_EQ(v[field_idx], pt_data_ptr[field_idx]) << "Unexpected value for field " << field_idx << "at r=" << r << ", c=" << c <<".";
            }
        }
    }
}

/**
 * @brief check values stored in organized point cloud after conversion
 * no. of channels equal to point's nb floats
 * Not applied for Point types with nb_floats > 4. This is OpenCV's upper limit on no. of channels.
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Values_Organized_nb_floats)
{
    if(ExpectedPointAttr_<TypeParam >::nb_floats <= 4) {

        typedef ConverterCloudMat_<TypeParam > C_;

        const int CLOUD_COLS=3;

        // organized point cloud
        Mat_<Vec<float, ExpectedPointAttr_<TypeParam >::nb_floats > > m(9, CLOUD_COLS, 1.f);
        randn(m, 0.f, 100.f);
        typename PointCloud<TypeParam >::Ptr cloud_ptr = C_::Mat2PointCloud(m);

        for(size_t r=0; r<cloud_ptr->height; r++) {

            for(size_t c=0; c<cloud_ptr->width; c++) {

                TypeParam pt = cloud_ptr->at(c, r);;
                float *pt_data_ptr = reinterpret_cast<float*>(&pt);

                Vec<float, ExpectedPointAttr_<TypeParam >::nb_floats > v = m(r, c);

                for(int field_idx=0; field_idx<this->nb_floats_i_; field_idx++) {

                    EXPECT_FLOAT_EQ(v[field_idx], pt_data_ptr[field_idx]) << "Unexpected value for field " << field_idx << "at r=" << r << ", c=" << c <<".";
                }
            }
        }
    }
}


/**
 * @brief Populate a single and multi-channel matrix with same values
 * and compare that both result into the same point cloud.
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Single_Ch_vs_multi_channel_mat)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    const int ROWS=3;
    const int CLOUD_COLS=2;

    // randomize matrix and represent two copies of the same data in single and multi-channel format
    Mat1f m_single_ch(ROWS, CLOUD_COLS*this->field_count_i_);
    randn(m_single_ch, 0.f, 100.f);

    Mat_<Vec<float, ExpectedPointAttr_<TypeParam >::field_count > > m_multi_ch(ROWS, CLOUD_COLS);

    for(size_t i=0, j=0; i<m_single_ch.total(); i+=this->field_count_i_) {

        Vec<float, ExpectedPointAttr_<TypeParam >::field_count > tmp;
        for(int k=0; k<this->field_count_i_; k++) {

            tmp[k] = m_single_ch(i+k);
        }
        m_multi_ch(j++) = tmp;
    }

    // perform conversion
    typename PointCloud<TypeParam >::Ptr cloud_ptr_single_ch = C_::Mat2PointCloud(m_single_ch);
    typename PointCloud<TypeParam >::Ptr cloud_ptr_multi_ch = C_::Mat2PointCloud(m_multi_ch);

    // compare resulting cloud points
    for(size_t r=0; r<cloud_ptr_single_ch->height; r++) {

        int i=0;
        for(size_t c=0; c<cloud_ptr_single_ch->width; c++, i++) {

            TypeParam pt_single_ch = cloud_ptr_single_ch->at(c, r);
            float *pt_data_ptr_single_ch = reinterpret_cast<float*>(&pt_single_ch);

            TypeParam pt_multi_ch = cloud_ptr_multi_ch->at(c, r);
            float *pt_data_ptr_multi_ch = reinterpret_cast<float*>(&pt_multi_ch);

            EXPECT_NE(pt_data_ptr_single_ch, pt_data_ptr_multi_ch) << "Pointers are pointing to the same memory location. Is this intentional?";

            for(int i=0; i<this->field_count_i_; i++) {

                EXPECT_FLOAT_EQ(pt_data_ptr_single_ch[i], pt_data_ptr_multi_ch[i]) << "Unexpected value for coordinate i=" << i;
            }
        }
    }
}

} // annonymous namespace for tests

#endif // WITH_PCL
