#include "core/area.h"

#include "gtest/gtest.h"
#include "core/exception.h"

using cv::Mat1f;
using namespace sem;

class TrapzTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = Trapz();
    }

    Trapz to_; ///< test object
};

TEST_F(TrapzTest, BadDims) {

    Mat1f x;
    EXPECT_THROW(to_(x, Mat1f::zeros(1, 2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat1f::zeros(1, 2), x), ExceptionBadDims);

    EXPECT_THROW(to_(x, x), ExceptionBadDims);

    EXPECT_THROW(to_(Mat1f::zeros(1, 1), Mat1f::zeros(1, 2)), ExceptionBadDims);
    EXPECT_THROW(to_(Mat1f::zeros(1, 2), Mat1f::zeros(1, 1)), ExceptionBadDims);

    EXPECT_NO_THROW(to_(Mat1f::zeros(1, 2), Mat1f::zeros(1, 2)));
    EXPECT_NO_THROW(to_(Mat1f::zeros(1, 2), Mat1f::zeros(2, 1)));
    EXPECT_NO_THROW(to_(Mat1f::zeros(2, 1), Mat1f::zeros(2, 1)));
    EXPECT_NO_THROW(to_(Mat1f::zeros(2, 1), Mat1f::zeros(1, 2)));

    EXPECT_NO_THROW(to_(Mat1f::zeros(10, 1), Mat1f::zeros(10, 1)));
}

TEST_F(TrapzTest, Trapezoidal) {

    float values[2] = {0.f, 1.f};
    Mat1f x(1, 2, values);
    Mat1f y = x.clone();

    EXPECT_FLOAT_EQ(to_(x, y), 0.5f);
    EXPECT_FLOAT_EQ(to_(x, Mat1f::zeros(1, 2)), 0.f);
    EXPECT_FLOAT_EQ(to_(x, Mat1f::ones(1, 2)), 1.f);
}

