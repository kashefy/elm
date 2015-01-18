#include "sem/core/pcl/cloud_impl_.h"
#include <pcl/point_traits.h>

#include "sem/ts/ts.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;

namespace {

template<typename TPoint>
class PCL_Cloud_T_TypedTests : public ::testing::Test
{
protected:
};
typedef ::testing::Types<PointXYZ, Normal, PointNormal> PCLPointTypes;

/** @brief helper struct with expected results
 */
template <typename TPoint>
struct Expected
{
    static const size_t field_count;
    static const size_t nb_floats;
    static const std::string name;
};

#define SEM_SET_EXPECTED_POINT_NAME(TPoint)             template<> const std::string Expected<TPoint>::name = #TPoint
#define SEM_SET_EXPECTED_POINT_FIELD_COUNT(TPoint, c)   template<> const size_t Expected<TPoint>::field_count = c
#define SEM_SET_EXPECTED_POINT_NB_FLOAT(TPoint, n) template<> const size_t Expected<TPoint>::nb_floats = n

/** Set expected attributes for a Point type used in tests below.
 *  @param TPoint point type
 *  @param c field count    (no. of floats without padding)
 *  @param n no. of floats occupied (no. of floats with padding)
  */
#define SEM_SET_EXPECTED_POINT_ATTR(TPoint, c, n) SEM_SET_EXPECTED_POINT_NAME(TPoint); SEM_SET_EXPECTED_POINT_FIELD_COUNT(TPoint, c); SEM_SET_EXPECTED_POINT_NB_FLOAT(TPoint, n)

SEM_SET_EXPECTED_POINT_ATTR(PointXYZ,   3,  4);  // x, y, z
SEM_SET_EXPECTED_POINT_ATTR(Normal,     4,  8);  // n1, n2, n3, curvature
SEM_SET_EXPECTED_POINT_ATTR(PointNormal, 7, 12); // PointXYZ + Normal = 4+3

TYPED_TEST_CASE(PCL_Cloud_T_TypedTests, PCLPointTypes);

TYPED_TEST(PCL_Cloud_T_TypedTests, FieldCount)
{
    ASSERT_GT(ConverterCloudMat_<TypeParam >::FieldCount(), size_t(0)) << "This point type does not have any fields.";
    EXPECT_EQ(ConverterCloudMat_<TypeParam >::FieldCount(), Expected<TypeParam >::field_count);
}

TYPED_TEST(PCL_Cloud_T_TypedTests, NbFloats)
{
    ASSERT_GT(ConverterCloudMat_<TypeParam >::NbFloats()*sizeof(float), size_t(0));
    EXPECT_EQ(sizeof(TypeParam), ConverterCloudMat_<TypeParam >::NbFloats()*sizeof(float));
    EXPECT_EQ(ConverterCloudMat_<TypeParam >::NbFloats(), Expected<TypeParam >::nb_floats);
}

template<typename TPoint>
class PCL_Cloud_T_Conversion_Single_Ch_TypedTests : public PCL_Cloud_T_TypedTests<TPoint >
{
protected:
    virtual void SetUp()
    {
        typedef ConverterCloudMat_<TPoint > C_;

        field_count_sz_ = C_::FieldCount();
        field_count_i_ = static_cast<int>(field_count_sz_);

        nb_floats_sz_ = C_::NbFloats();
        nb_floats_i_ = static_cast<int>(nb_floats_sz_);
    }

    size_t field_count_sz_;
    int field_count_i_;

    size_t nb_floats_sz_;
    int nb_floats_i_;
};

TYPED_TEST_CASE(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, PCLPointTypes);

/**
 * @brief The function checks for empty Mat empty on, it doesn't look much into if channels are sufficient for conversion
 * This test will tell us if that assumption should ever be reconsidered.
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, Empty)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    EXPECT_TRUE(C_::Mat2PointCloud(Mat1f())->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat1f(1, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat1f(0, 1))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat1f(0, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat1f(3, 0))->empty());
    EXPECT_TRUE(C_::Mat2PointCloud(Mat1f(0, 3))->empty());
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, Empty_Size)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat1f())->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat1f(1, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat1f(0, 1))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat1f(0, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat1f(3, 0))->size());
    EXPECT_EQ(size_t(0), C_::Mat2PointCloud(Mat1f(0, 3))->size());
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, Invalid)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    for(int r=1; r<7; r++) {

        for(int c=1; c<15; c++) {

            Mat1f m(r, c, 1.f);
            if(c % this->field_count_i_ == 0 || c % this->nb_floats_i_ == 0) {

                EXPECT_NO_THROW(C_::Mat2PointCloud(m));
            }
            else {
                EXPECT_THROW(C_::Mat2PointCloud(m), ExceptionBadDims);
            }
        }
    }
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, Invalid_field_count)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    for(int r=1; r<7; r++) {

        for(int c=1; c<15; c++) {

            Mat1f m(r, c, 1.f);
            if(c % this->nb_floats_i_ != 0) {
                if(c % this->field_count_i_ == 0) {

                    EXPECT_NO_THROW(C_::Mat2PointCloud(m));
                }
                else {
                    EXPECT_THROW(C_::Mat2PointCloud(m), ExceptionBadDims);
                }
            }
        }
    }
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, Invalid_nb_floats)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    for(int r=1; r<7; r++) {

        for(int c=1; c<15; c++) {

            Mat1f m(r, c, 1.f);
            if(c % this->field_count_i_ != 0) {
                if(c % this->nb_floats_i_ == 0) {

                    EXPECT_NO_THROW(C_::Mat2PointCloud(m));
                }
                else {
                    EXPECT_THROW(C_::Mat2PointCloud(m), ExceptionBadDims);
                }
            }
        }
    }
}

/**
 * @brief Pass a Mat with columns that are multiple of one but not the other
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, Dims_field_count)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    for(int r=1; r<7; r++) {

        for(int n=1; n<7; n++) {

            int c = n*this->field_count_i_;
            Mat1f m(r, c, 1.f);

            typename PointCloud<TypeParam >::Ptr cloud_ptr;
            cloud_ptr = C_::Mat2PointCloud(m);

            EXPECT_EQ(r*n, static_cast<int>(cloud_ptr->size()));
            EXPECT_EQ(n, static_cast<int>(cloud_ptr->width));
            EXPECT_EQ(r, static_cast<int>(cloud_ptr->height));
        }
    }
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, Dims_nb_floats)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    for(int r=1; r<7; r++) {

        for(int n=1; n<7; n++) {

            int c = n*this->nb_floats_i_;
            if(c % this->field_count_i_ != 0) {

                Mat1f m(r, c, 1.f);

                typename PointCloud<TypeParam >::Ptr cloud_ptr;
                cloud_ptr = C_::Mat2PointCloud(m);

                EXPECT_EQ(r*n, static_cast<int>(cloud_ptr->size()));
                EXPECT_EQ(n, static_cast<int>(cloud_ptr->width));
                EXPECT_EQ(r, static_cast<int>(cloud_ptr->height));
            }
        }
    }
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, IsOrganized)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    Mat1f m(9, 3*this->field_count_i_);
    randn(m, 0.f, 100.f);

    EXPECT_TRUE(C_::Mat2PointCloud(m)->isOrganized());
    EXPECT_TRUE(C_::Mat2PointCloud(m.reshape(1, 3))->isOrganized());
    EXPECT_FALSE(C_::Mat2PointCloud(m.reshape(1, 1))->isOrganized()) << "Row matrices yield un-organized point clouds";
}

/**
 * @brief check values stored in non-organized point cloud after conversion
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, Values_NonOrganized)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    const int CLOUD_COLS=3;

    Mat1f m(1, CLOUD_COLS*this->field_count_i_);
    randn(m, 0.f, 100.f);
    typename PointCloud<TypeParam >::Ptr cloud_ptr = C_::Mat2PointCloud(m);

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
TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_TypedTests, Values_Organized)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    const int CLOUD_COLS=3;

    // organized point cloud
    Mat1f m(4, CLOUD_COLS*this->field_count_i_);
    randn(m, 0.f, 100.f);
    typename PointCloud<TypeParam >::Ptr cloud_ptr = C_::Mat2PointCloud(m);

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
class PCL_Cloud_T_Conversion_Multi_Ch_TypedTests : public PCL_Cloud_T_Conversion_Single_Ch_TypedTests<TPoint >
{
protected:
    virtual void SetUp()
    {

        /** Sanity check that our subset of PointTypes is fit for mutli-channel tests
         */
        static_assert(Expected<TPoint >::field_count <= 4,
                      "No. of fields exceed max no. of channels allowed by OpenCV.");
        PCL_Cloud_T_Conversion_Single_Ch_TypedTests<TPoint>::SetUp();
    }
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
}

TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Dims)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    for(int r=1; r<7; r++) {

        for(int c=1; c<7; c++) {

            Mat_<Vec<float, Expected<TypeParam >::field_count > > m(r, c, 1.f);

            typename PointCloud<TypeParam >::Ptr cloud_ptr;

            cloud_ptr = C_::Mat2PointCloud(m);
            EXPECT_EQ(m.total(), cloud_ptr->size());
            EXPECT_EQ(c, static_cast<int>(cloud_ptr->width));
            EXPECT_EQ(r, static_cast<int>(cloud_ptr->height));

            cloud_ptr = C_::Mat2PointCloud(m.reshape(1, 1));
            EXPECT_EQ(m.total(), cloud_ptr->size());
            EXPECT_EQ(c*r, static_cast<int>(cloud_ptr->width));
            EXPECT_EQ(1, static_cast<int>(cloud_ptr->height));
        }
    }
}

TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, IsOrganized)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    Mat_<Vec<float, Expected<TypeParam >::field_count > > m(9, 9, 1.f);

    EXPECT_TRUE(C_::Mat2PointCloud(m)->isOrganized());
    EXPECT_TRUE(C_::Mat2PointCloud(m.reshape(1, 3))->isOrganized());
    EXPECT_FALSE(C_::Mat2PointCloud(m.reshape(1, 1))->isOrganized()) << "Row matrices yield un-organized point clouds";
}

/**
 * @brief check values stored in non-organized point cloud after conversion
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Values_NonOrganized)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    const int CLOUD_COLS=3;

    Mat_<Vec<float, Expected<TypeParam >::field_count > > m(9, CLOUD_COLS, 1.f);
    randn(m, 0.f, 100.f);
    typename PointCloud<TypeParam >::Ptr cloud_ptr = C_::Mat2PointCloud(m);

    int i=0;
    for(typename PointCloud<TypeParam >::iterator itr=cloud_ptr->begin(); itr != cloud_ptr->end(); ++itr) {

        TypeParam pt = *itr;
        float *pt_data_ptr = reinterpret_cast<float*>(&pt);

        Vec<float, Expected<TypeParam >::field_count > v = m(i++);

        for(int field_idx=0; field_idx<this->field_count_i_; field_idx++) {

            EXPECT_FLOAT_EQ(v[field_idx], pt_data_ptr[field_idx]) << "Unexpected value for field " << i << ".";
        }
    }
}

/**
 * @brief check values stored in organized point cloud after conversion
 */
TYPED_TEST(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, Values_Organized)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    const int CLOUD_COLS=3;

    // organized point cloud
    Mat_<Vec<float, Expected<TypeParam >::field_count > > m(9, CLOUD_COLS, 1.f);
    randn(m, 0.f, 100.f);
    typename PointCloud<TypeParam >::Ptr cloud_ptr = C_::Mat2PointCloud(m);

    for(size_t r=0; r<cloud_ptr->height; r++) {

        for(size_t c=0; c<cloud_ptr->width; c++) {

            TypeParam pt = cloud_ptr->at(c, r);;
            float *pt_data_ptr = reinterpret_cast<float*>(&pt);

            Vec<float, Expected<TypeParam >::field_count > v = m(r, c);

            for(int field_idx=0; field_idx<this->field_count_i_; field_idx++) {

                EXPECT_FLOAT_EQ(v[field_idx], pt_data_ptr[field_idx]) << "Unexpected value for field " << field_idx << "at r=" << r << ", c=" << c <<".";
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

    Mat_<Vec<float, Expected<TypeParam >::field_count > > m_multi_ch(ROWS, CLOUD_COLS);

    for(size_t i=0, j=0; i<m_single_ch.total(); i+=this->field_count_i_) {

        Vec<float, Expected<TypeParam >::field_count > tmp;
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
