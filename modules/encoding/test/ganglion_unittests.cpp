#include "encoding/ganglion.h"

#include "core/exception.h"
#include "ts/ts.h"

using cv::Mat1f;
using cv::Size2i;

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
    EXPECT_MAT_DIMS_EQ(to_.Kernel(), Size2i(radius_*2+1, radius_*2+1));
    EXPECT_MAT_TYPE(to_.Kernel(), CV_32F);
}

TEST_F(DiffOfGaussians2dSqTest, OnOff)
{
    DiffOfGaussians2dSq to2;
    to2.Init(radius_, scale_, false);
    EXPECT_MAT_EQ(to_.Kernel(), -to2.Kernel());
}


