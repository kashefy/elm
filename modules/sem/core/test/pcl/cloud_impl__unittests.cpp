#include "sem/core/pcl/cloud_impl_.h"

#include "sem/core/pcl/point_traits.h"
#include "sem/ts/pcl_point_typed_tests.h"
#include "sem/ts/ts.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace sem;
using namespace sem::ts;

namespace {

template<typename TPoint>
class PCL_Cloud_T_Conversion_Single_Ch_TypedTests : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
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

            if((c % this->field_count_i_ != 0) && (c % this->nb_floats_i_ != 0)) {

                EXPECT_THROW(C_::Mat2PointCloud(m), ExceptionBadDims) << "with cols = " << c;
            }
            else if((c % this->field_count_i_ == 0) && (c % this->nb_floats_i_ == 0)) {

                // ambiguous
                EXPECT_NO_THROW(C_::Mat2PointCloud(m)) << "with cols = " << c;
            }
            else {
                if(c % this->field_count_i_ == 0) {

                    EXPECT_NO_THROW(C_::Mat2PointCloud(m)) << "with cols = " << c;
                }

                if(this->nb_floats_i_ == 0) {

                    EXPECT_NO_THROW(C_::Mat2PointCloud(m)) << "with cols = " << c;
                }
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

                    EXPECT_NO_THROW(C_::Mat2PointCloud(m)) << "with cols = " << c;
                }
                else {
                    EXPECT_THROW(C_::Mat2PointCloud(m), ExceptionBadDims) << "with cols = " << c;
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
            if(c % this->nb_floats_i_ != 0) {

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
class PCL_Cloud_T_Conversion_Single_Ch_IsPadded_TypedTests : public PCL_Cloud_T_Conversion_Single_Ch_TypedTests<TPoint >
{
protected:
};

TYPED_TEST_CASE(PCL_Cloud_T_Conversion_Single_Ch_IsPadded_TypedTests, PCLPointTypes);

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_IsPadded_TypedTests, EmptyMat)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    EXPECT_FALSE(C_::IsPaddedMat(Mat1f()));
    EXPECT_FALSE(C_::IsPaddedMat(Mat1f(1, 0)));
    EXPECT_FALSE(C_::IsPaddedMat(Mat1f(0, 1)));

    EXPECT_FALSE(C_::IsPaddedMat(Mat1f(this->field_count_i_, 0)));
    EXPECT_FALSE(C_::IsPaddedMat(Mat1f(0, this->field_count_i_)));

    EXPECT_FALSE(C_::IsPaddedMat(Mat1f(this->nb_floats_i_, 0)));
    EXPECT_FALSE(C_::IsPaddedMat(Mat1f(0, this->nb_floats_i_)));
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_IsPadded_TypedTests, Multiple_FieldCount)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    for(int n=1; n<9; n++) {

        int cols = n*this->field_count_i_;
        if(cols % this->nb_floats_i_ != 0) {

            Mat1f m(3, cols);
            randn(m, 0.f, 100.f);
            EXPECT_FALSE(C_::IsPaddedMat(m));
        }
    }
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_IsPadded_TypedTests, Multiple_NbFloats)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    for(int n=1; n<9; n++) {

        int cols = n*this->nb_floats_i_;
        if(cols % this->field_count_i_ != 0) {

            Mat1f m(3, cols);
            randn(m, 0.f, 100.f);
            EXPECT_TRUE(C_::IsPaddedMat(m));
        }
    }
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_IsPadded_TypedTests, No_Multiple)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    for(int cols=1; cols<=this->field_count_i_*this->nb_floats_i_; cols++) {

        if((cols % this->field_count_i_ != 0) && (cols % this->nb_floats_i_ != 0)) {

            Mat1f m(3, cols);
            randn(m, 0.f, 100.f);
            EXPECT_THROW(C_::IsPaddedMat(m), ExceptionBadDims) << "where cols = " << cols;
        }
    }
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_IsPadded_TypedTests, Ambiguous_Padded)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    EXPECT_TRUE(C_::IsPaddedMat(Mat1f::ones(3, this->field_count_i_*this->nb_floats_i_)));
    EXPECT_TRUE(C_::IsPaddedMat(Mat1f::zeros(3, this->field_count_i_*this->nb_floats_i_)));

    Mat1f m(3, this->field_count_i_*this->nb_floats_i_);
    randn(m, 0.f, 100.f);

    int p = this->nb_floats_i_ - this->field_count_i_;

    for(int col=this->field_count_i_; col<m.cols; col+=this->nb_floats_i_) {

        m.colRange(col, col+p).setTo(0.f);
    }
    EXPECT_TRUE(C_::IsPaddedMat(m));

    EXPECT_TRUE(C_::IsPaddedMat(m.row(0)));
    EXPECT_TRUE(C_::IsPaddedMat(m.rowRange(0, 2)));
}

TYPED_TEST(PCL_Cloud_T_Conversion_Single_Ch_IsPadded_TypedTests, Ambiguous_NotPadded)
{
    typedef ConverterCloudMat_<TypeParam > C_;

    Mat1f m(3, this->field_count_i_*this->nb_floats_i_);
    randn(m, 0.f, 100.f);
    EXPECT_FALSE(C_::IsPaddedMat(m));

    EXPECT_TRUE(C_::IsPaddedMat(m.row(0)));
    EXPECT_FALSE(C_::IsPaddedMat(m.rowRange(0, 2)));
}

// must preceed static_assertions
SEM_SET_EXPECTED_POINT_ATTR(pcl::PointXYZ,   3,  4);  // x, y, z
SEM_SET_EXPECTED_POINT_ATTR(pcl::Normal,     4,  8);  // n1, n2, n3, curvature
SEM_SET_EXPECTED_POINT_ATTR(pcl::PointNormal, 7, 12); // PointXYZ + Normal = 4+3

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
        static_assert(ExpectedPointAttr_<TPoint >::field_count <= 4,
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
