#include "encoding/intensitycontrast.h"

#include "core/exception.h"
#include "io/synth.h"
#include "ts/ts.h"

using namespace cv;

class IntensityConstrastRetGangTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        radius_ = 13;
        scale_ = 1.f;
        to_.Init(radius_, scale_);
    }

    RetGang to_;    ///< test object
    int radius_;
    int scale_;
};

TEST_F(IntensityConstrastRetGangTest, Init)
{
    EXPECT_THROW(to_.Init(0, 1.f), ExceptionBadDims);
    EXPECT_THROW(to_.Init(-1, 1.f), ExceptionBadDims);
    EXPECT_THROW(to_.Init(-20, 1.f), ExceptionBadDims);
}

TEST_F(IntensityConstrastRetGangTest, Uniform)
{
    for(float v=-1.5f; v<=1.5f; v++) {

        Mat1f stimulus = Mat1f(radius_*3, radius_*3, v);
        to_.Compute(stimulus);
        EXPECT_EQ(countNonZero(to_.State() > 1e-6), 0) << "Encountered large values.";
        EXPECT_MAT_EQ(to_.Response(), Mat1f::zeros(stimulus.size())) << "Edges detected in constant stimulus";
    }
}

/**
 * @brief test that response does not change with stimulus repition
 */
TEST_F(IntensityConstrastRetGangTest, Static)
{
    Mat1f stimulus = Mat1f(radius_*3, radius_*3, randu<float>());
    to_.Compute(stimulus);
    Mat s = to_.State();
    Mat r = to_.Response();

    int i = 100;
    while(i-- > 0)
    {
        to_.Compute(stimulus);
        EXPECT_MAT_EQ(to_.State(), s) << "State changed.";
        EXPECT_MAT_EQ(to_.Response(), r) << "Response changed.";
    }
}

TEST_F(IntensityConstrastRetGangTest, Dims)
{
    int i = 5;
    while(--i >= 2)
    {
        Size2i sz(radius_*i, radius_*i);
        Mat1f stimulus = Mat1f(sz, randu<float>());
        to_.Compute(stimulus);
        EXPECT_MAT_DIMS_EQ(to_.State(), sz);
        EXPECT_MAT_DIMS_EQ(to_.Response(), sz);
    }
}

TEST_F(IntensityConstrastRetGangTest, BarVertical)
{
    SynthBars b;
    b.Reset(radius_*2+1, radius_*2+1, 1);
    Mat img;
    b.Draw(90.f, img);
    img.convertTo(img, CV_32F, 1./255.);

    to_.Compute(img);

    Mat1f expected_r = Mat1f::zeros(1, img.cols);
    expected_r(9) = 1;
    expected_r(img.cols-9) = 1;
    expected_r = repeat(expected_r, img.rows, 1);
    EXPECT_MAT_EQ(to_.Response(), expected_r);
}

TEST_F(IntensityConstrastRetGangTest, BarHoriztonal)
{
    SynthBars b;
    b.Reset(radius_*2+1, radius_*2+1, 1);
    Mat img;
    b.Draw(0.f, img);
    img.convertTo(img, CV_32F, 1./255.);

    Mat1f expected_r = Mat1f::zeros(img.rows, 1);
    expected_r(9) = 1;
    expected_r(img.rows-9) = 1;
    expected_r = repeat(expected_r, 1, img.cols);

    to_.Compute(img);

    EXPECT_MAT_EQ(to_.Response(), expected_r);
}
