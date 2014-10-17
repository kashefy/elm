#include "io/synth.h"

#include "ts/ts.h"

#include "core/exception.h"

#include <opencv2/highgui.hpp>

using namespace std;

class SynthBarsTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = SynthBars();
    }

    SynthBars to_;  ///< test object
};

TEST_F(SynthBarsTest, Reset)
{
    EXPECT_THROW(to_.Reset(3, 3, 0), ExceptionValueError);

    for(int i=1; i<6; i++) {

        to_.Reset(3, 3, i);
        for(int j=0; j<100; j++) {

            EXPECT_FLOAT_EQ(to_.IndexToDeg(j), static_cast<float>(j*180.f/i));
        }
    }
}

TEST_F(SynthBarsTest, IndexToDegrees)
{
    for(int i=0; i<100; i++) {

        EXPECT_FLOAT_EQ(to_.IndexToDeg(i), static_cast<float>(i*180.f/6.f));
    }
}

TEST_F(SynthBarsTest, Next)
{
    const int N=50;
    const int ROWS=100;
    const int COLS=100;
    const int NB_VARIATIONS=6;
    to_.Reset(ROWS, COLS, NB_VARIATIONS);
    for(int i=0; i<N; i++) {

        cv::Mat img = to_.Next();
        EXPECT_MAT_DIMS_EQ(img, cv::Mat1b(ROWS, COLS));
        EXPECT_EQ(img.at<uchar>(ROWS/2, COLS/2), static_cast<uchar>(255));
        //cv::imshow("x", img);
        //cv::waitKey();
    }
}

TEST_F(SynthBarsTest, DISABLED_Display)
{
    to_.Reset(100, 100, 6);
    for(int i=0; i<10; i++) {

        cv::Mat img = to_.Next();
        cv::imshow(FullTestName(test_info_), img);
        cv::waitKey();
    }
}
