#include "sem/encoding/ganglion.h"

#include "sem/core/exception.h"
#include "sem/io/synth.h"
#include "sem/ts/ts.h"

using namespace cv;
using namespace elm;

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
    EXPECT_LT(max_idx[2]-abs(max_idx[2]-min_idx[2]), kernel.cols/3);
}

TEST_F(DiffOfGaussians2dSqTest, OnOff)
{
    DiffOfGaussians2dSq to2;
    to2.Init(radius_, scale_, false);
    EXPECT_MAT_EQ(to_.Kernel(), -to2.Kernel());
}

TEST_F(DiffOfGaussians2dSqTest, Compute_impulse)
{
    Size2i size(radius_*2+1, radius_*2+1);
    Mat1f impulse = Mat1f::zeros(size);
    impulse(size.height/2, size.width/2) = 1.f;
    EXPECT_MAT_EQ(to_.Kernel(), to_.Compute(impulse));
}

TEST_F(DiffOfGaussians2dSqTest, Compute_zero)
{
    Size2i size(radius_*2+1, radius_*2+1);
    EXPECT_MAT_EQ(Mat1f::zeros(size), to_.Compute(Mat1f::zeros(size)));
}

TEST_F(DiffOfGaussians2dSqTest, Compute_ones)
{
    Size2i size(radius_*2+1, radius_*2+1);
    EXPECT_EQ(countNonZero(to_.Compute(Mat1f::ones(size)) > 1e-7), 0);
}

TEST_F(DiffOfGaussians2dSqTest, Compute_bar)
{
    SynthBars b;
    b.Reset(radius_*2+1, radius_*2+1, 1);
    Mat img;
    b.Draw(90.f, img);
    img.convertTo(img, CV_32F, 1./255.);

    EXPECT_MAT_NEAR(to_.Compute(img), to_.Compute(img.t()).t(), 1e-6);
}



