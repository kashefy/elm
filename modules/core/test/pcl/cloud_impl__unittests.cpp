#include "core/pcl/cloud_impl_.h"

#include "ts/ts.h"

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
};

template<> const size_t Expected<PointXYZ>::field_count     = 3;    // x, y, z
template<> const size_t Expected<Normal>::field_count       = 4;    // n1, n2, n3, curvature
template<> const size_t Expected<PointNormal>::field_count  = 7;    // PointXYZ + Normal = 4+3

template<> const size_t Expected<PointXYZ>::nb_floats       = 3;    // x, y, z
template<> const size_t Expected<Normal>::nb_floats         = 4;    // n1, n2, n3, curvature
template<> const size_t Expected<PointNormal>::nb_floats    = 7;    // PointXYZ + Normal = 4+3


TYPED_TEST_CASE(PCL_Cloud_T_TypedTests, PCLPointTypes);

TYPED_TEST(PCL_Cloud_T_TypedTests, FieldCount)
{
    ASSERT_GT(ConverterCloudMat_<TypeParam >::FieldCount(), size_t(0)) << "This point type does not have any fields.";
    EXPECT_EQ(ConverterCloudMat_<TypeParam >::FieldCount(), Expected<TypeParam >::field_count);
}

TYPED_TEST(PCL_Cloud_T_TypedTests, NbFloats)
{
    ASSERT_GT(ConverterCloudMat_<TypeParam >::NbFloats()*sizeof(float), size_t(0));
    cout<<ConverterCloudMat_<TypeParam >::NbFloats()<<endl;
    EXPECT_EQ(sizeof(TypeParam), ConverterCloudMat_<TypeParam >::NbFloats()*sizeof(float));
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

template<typename TPoint>
class PCL_Cloud_T_Conversion_Multi_Ch_TypedTests : public PCL_Cloud_T_Conversion_Single_Ch_TypedTests<TPoint >
{
protected:
    virtual void SetUp()
    {
        PCL_Cloud_T_Conversion_Single_Ch_TypedTests<TPoint>::SetUp();
    }
};

TYPED_TEST_CASE(PCL_Cloud_T_Conversion_Multi_Ch_TypedTests, PCLPointTypes);

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

} // annonymous namespace for tests
