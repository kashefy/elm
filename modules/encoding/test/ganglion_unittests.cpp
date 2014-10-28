#include "encoding/ganglion.h"

#include <opencv2/imgproc.hpp>

#include "core/exception.h"
#include "ts/ts.h"

using namespace cv;

class DiffOfGaussians2dSqTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        radius_ = 13;
        scale_ = 1.f;
        to_.Init(radius_, scale_, true);
    }

    DiffOfGaussians2dSq to_;    ///< test object
    int radius_;                ///< kernel radius
    float scale_;               ///< scale
};

TEST_F(DiffOfGaussians2dSqTest, InvalidInit)
{
    EXPECT_THROW(to_.Init(0, 1, true), ExceptionBadDims);
    EXPECT_THROW(to_.Init(-1, 1, true), ExceptionBadDims);
}

TEST_F(DiffOfGaussians2dSqTest, KernelProperties)
{
    Mat kernel = to_.Kernel();

    EXPECT_EQ(1, kernel.rows % 2) << "No. of rows must be odd";
    EXPECT_EQ(1, kernel.cols % 2) << "No. of columns must be odd";
    EXPECT_MAT_DIMS_EQ(kernel, Size2i(radius_*2+1, radius_*2+1));
    EXPECT_MAT_TYPE(kernel, CV_32F);

    EXPECT_MAT_EQ(kernel, kernel.t());
}

TEST_F(DiffOfGaussians2dSqTest, Kernel)
{
    Mat kernel = to_.Kernel();
    Mat mid_row = kernel.row(radius_);
    Mat mid_col = kernel.row(radius_);


}

TEST_F(DiffOfGaussians2dSqTest, OnOff)
{
    DiffOfGaussians2dSq to2;
    to2.Init(radius_, scale_, false);
    EXPECT_MAT_EQ(to_.Kernel(), -to2.Kernel());
}


