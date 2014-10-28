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
        center_on_ = true;
        to_.Init(radius_, scale_, center_on_);
    }

    DiffOfGaussians2dSq to_;    ///< test object
    int radius_;                ///< kernel radius
    float scale_;               ///< scale
    bool center_on_;            ///< center on surround off
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
    Mat kernel_flipped;
    flip(kernel, kernel_flipped, 0);
    EXPECT_MAT_EQ(kernel, kernel_flipped);
    flip(kernel, kernel_flipped, 1);
    EXPECT_MAT_EQ(kernel, kernel_flipped);
    flip(kernel.t(), kernel_flipped, 1);
    EXPECT_MAT_EQ(kernel, kernel_flipped);
}

TEST_F(DiffOfGaussians2dSqTest, Kernel)
{
    Mat kernel = to_.Kernel();
    Mat1f mid_row = kernel.row(radius_);
    Mat1f mid_col = kernel.col(radius_);

    EXPECT_MAT_EQ(mid_col, mid_row.t());

    double min_val, max_val;
    int min_idx[2] = {-1, -1};
    int max_idx[2] = {-1, -1};
    minMaxIdx(mid_row, &min_val, &max_val, min_idx, max_idx);

    EXPECT_NE(max_val, min_val);
    EXPECT_GT(max_val, 0);
    EXPECT_EQ(max_idx[1], kernel.cols/2);
    EXPECT_LT(min_val, 0);
    EXPECT_GE(mid_row(0), min_val);
    EXPECT_GE(mid_row(mid_row.cols-1), min_val);
    EXPECT_TRUE(max_idx[2]-abs(max_idx[2]-min_idx[2]) < kernel.cols/3);
}

TEST_F(DiffOfGaussians2dSqTest, OnOff)
{
    DiffOfGaussians2dSq to2;
    to2.Init(radius_, scale_, false);
    EXPECT_MAT_EQ(to_.Kernel(), -to2.Kernel());
}


