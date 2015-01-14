#include "io/synth.h"

#include "ts/ts.h"

#include "core/exception.h"

#include <opencv2/highgui.hpp>

using namespace std;
using namespace sem;

class SynthBarsTest : public ::testing::Test
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
        float angle = 0.f;
        float delta = static_cast<float>(180.f/i);
        for(int j=0; j<100; j++) {

            EXPECT_FLOAT_EQ(to_.IndexToDeg(j), static_cast<float>(angle));
            angle += delta;
            if(angle >= 180.f) { angle = 0.f; }
        }
    }
}

TEST_F(SynthBarsTest, IndexToDegrees)
{
    float angle = 0.f;
    float delta = static_cast<float>(180.f/6.f);
    for(int i=0; i<70; i++) {

        EXPECT_FLOAT_EQ(to_.IndexToDeg(i), static_cast<float>(angle));

        angle += delta;
        if(angle >= 180.f) { angle = 0.f; }
    }

    EXPECT_NE(to_.IndexToDeg(0), to_.IndexToDeg(1)) << "No variation.";
    EXPECT_NE(to_.IndexToDeg(0), to_.IndexToDeg(5)) << "No variation.";
    EXPECT_FLOAT_EQ(to_.IndexToDeg(0), to_.IndexToDeg(6)) << "0 should equal 180";
}

TEST_F(SynthBarsTest, Next)
{
    const int N=50;
    const int ROWS=100;
    const int COLS=100;
    const int NB_VARIATIONS=6;
    to_.Reset(ROWS, COLS, NB_VARIATIONS);
    for(int i=0; i<N; i++) {

        cv::Mat img, label;
        to_.Next(img, label);

        EXPECT_MAT_DIMS_EQ(img, cv::Mat1b(ROWS, COLS));
        EXPECT_MAT_TYPE(img, CV_8U);
        EXPECT_EQ(img.at<uchar>(ROWS/2, COLS/2), static_cast<uchar>(255));

        EXPECT_MAT_DIMS_EQ(label, cv::Mat1f(1, 1));
        EXPECT_MAT_TYPE(label, CV_32F);
        EXPECT_IN_LCLOSED_ROPEN(label.at<float>(0), 0.f, 180.f);
    }
}

/**
 * @brief test that bar orientations are unifromly distributed
 */
TEST_F(SynthBarsTest, Uniform)
{
    const int N=4e3;
    const int ROWS=3;
    const int COLS=3;
    const int NB_VARIATIONS=4;
    to_.Reset(ROWS, COLS, NB_VARIATIONS);

    cv::Mat1f counts = cv::Mat1f::zeros(1, NB_VARIATIONS);

    for(int i=0; i<N; i++) {

        cv::Mat img, label;
        to_.Next(img, label);

        float angle = label.at<float>(0);

        EXPECT_IN_LCLOSED_ROPEN(angle, 0, 180);

        int bin = static_cast<int>(angle/180.f*NB_VARIATIONS);

        EXPECT_LT(bin, NB_VARIATIONS);

        counts(bin)++;
    }

    EXPECT_FLOAT_EQ(cv::sum(counts)(0), N);

    cv::Mat1f hist_normalized = counts/static_cast<float>(N);

    // check histogram is near uniform
    cv::Mat m, s;
    cv::meanStdDev(hist_normalized, m, s);
    EXPECT_MAT_NEAR(m, cv::Mat1d(1, 1, 1.f/static_cast<double>(NB_VARIATIONS)), 1e-5);
    EXPECT_MAT_NEAR(s, cv::Mat1d::zeros(1, 1), 0.013);
}

/**
 * @brief Spot-check bar lines
 */
TEST_F(SynthBarsTest, BarLines)
{
    const int SIZE=28;
    const int NB_VARIATIONS=4;
    const float DELTA=180.f/NB_VARIATIONS;
    cv::Mat1b variations_covered = cv::Mat1b::zeros(1, NB_VARIATIONS);
    cv::Mat1b covered_enough = cv::Mat1b::ones(1, NB_VARIATIONS);
    to_.Reset(SIZE, SIZE, NB_VARIATIONS);

    const uchar ON=static_cast<uchar>(255);
    const uchar OFF=static_cast<uchar>(0);

    int i=0;
    int MAX_ITERATIONS=50;

    while(cv::countNonZero(covered_enough) > 0 && i < MAX_ITERATIONS) {

        cv::Mat img, label;
        to_.Next(img, label);

        float angle = label.at<float>(0);
        if(angle == 0.f) {

            // horizontal line
            for(int i=0; i<SIZE; i++) {

                EXPECT_EQ(img.at<uchar>(SIZE/2, i), ON) << "No horizontal line.";
            }
            EXPECT_MAT_EQ(img.rowRange(0, 4), cv::Mat1b(4, SIZE, OFF));
            EXPECT_MAT_EQ(img.rowRange(SIZE-4, SIZE), cv::Mat1b(4, SIZE, OFF));
        }
        else if(angle == 45.f) {

            EXPECT_EQ(img.at<uchar>(0, 0), OFF) << "diagonal not /";
            EXPECT_EQ(img.at<uchar>(SIZE-1, 0), ON) << "diagonal not /";
            EXPECT_EQ(img.at<uchar>(0, SIZE-1), ON) << "diagonal not /";
            EXPECT_EQ(img.at<uchar>(SIZE-1, SIZE-1), OFF) << "diagonal not /";

        }
        else if(angle == 90.f) {

            // vertical line
            for(int i=0; i<SIZE; i++) {

                EXPECT_EQ(img.at<uchar>(i, SIZE/2), ON) << "No vertical line.";
            }
            EXPECT_MAT_EQ(img.colRange(0, 4), cv::Mat1b(SIZE, 4, OFF));
            EXPECT_MAT_EQ(img.colRange(SIZE-4, SIZE), cv::Mat1b(SIZE, 4, OFF));
        }
        else if(angle == 135.f) {

            EXPECT_EQ(img.at<uchar>(0, 0), ON) << "diagonal not \\";
            EXPECT_EQ(img.at<uchar>(SIZE-1, 0), OFF) << "diagonal not \\";
            EXPECT_EQ(img.at<uchar>(0, SIZE-1), OFF) << "diagonal not \\";
            EXPECT_EQ(img.at<uchar>(SIZE-1, SIZE-1), ON) << "diagonal not \\";
        }

        EXPECT_EQ(img.at<uchar>(SIZE/2, SIZE/2), ON) << "not centered";

        int bin = static_cast<int>(angle/DELTA);
        variations_covered(bin)++;

        cv::compare(variations_covered,
                    cv::Mat1b(1, NB_VARIATIONS, static_cast<uchar>(5)),
                    covered_enough, cv::CMP_LT);
    }

    EXPECT_FALSE(cv::countNonZero(covered_enough) > 0 && i < MAX_ITERATIONS)
            << "Insufficient coverage of orientation variations" << variations_covered;
}

TEST_F(SynthBarsTest, Draw)
{
    const int SIZE=28;
    const int NB_VARIATIONS=4;
    const float DELTA=180.f/NB_VARIATIONS;
    to_.Reset(SIZE, SIZE, NB_VARIATIONS);

    const uchar ON=static_cast<uchar>(255);
    const uchar OFF=static_cast<uchar>(0);

    for(float angle=0.f; angle<=360.f; angle+=DELTA) {

        cv::Mat1f img;

        to_.Draw(angle, img);
        if(angle == 0.f) {

            // horizontal line
            for(int i=0; i<SIZE; i++) {

                EXPECT_EQ(img.at<uchar>(SIZE/2, i), ON) << "No horizontal line.";
            }
            EXPECT_MAT_EQ(img.rowRange(0, 4), cv::Mat1b(4, SIZE, OFF));
            EXPECT_MAT_EQ(img.rowRange(SIZE-4, SIZE), cv::Mat1b(4, SIZE, OFF));
        }
        else if(angle == 45.f) {

            EXPECT_EQ(img.at<uchar>(0, 0), OFF) << "diagonal not /";
            EXPECT_EQ(img.at<uchar>(SIZE-1, 0), ON) << "diagonal not /";
            EXPECT_EQ(img.at<uchar>(0, SIZE-1), ON) << "diagonal not /";
            EXPECT_EQ(img.at<uchar>(SIZE-1, SIZE-1), OFF) << "diagonal not /";

        }
        else if(angle == 90.f) {

            // vertical line
            for(int i=0; i<SIZE; i++) {

                EXPECT_EQ(img.at<uchar>(i, SIZE/2), ON) << "No vertical line.";
            }
            EXPECT_MAT_EQ(img.colRange(0, 4), cv::Mat1b(SIZE, 4, OFF));
            EXPECT_MAT_EQ(img.colRange(SIZE-4, SIZE), cv::Mat1b(SIZE, 4, OFF));
        }
        else if(angle == 135.f) {

            EXPECT_EQ(img.at<uchar>(0, 0), ON) << "diagonal not \\";
            EXPECT_EQ(img.at<uchar>(SIZE-1, 0), OFF) << "diagonal not \\";
            EXPECT_EQ(img.at<uchar>(0, SIZE-1), OFF) << "diagonal not \\";
            EXPECT_EQ(img.at<uchar>(SIZE-1, SIZE-1), ON) << "diagonal not \\";
        }

        EXPECT_EQ(img.at<uchar>(SIZE/2, SIZE/2), ON) << "not centered";
    }
}

TEST_F(SynthBarsTest, DISABLED_Display)
{
    to_.Reset(100, 100, 6);
    for(int i=0; i<10; i++) {

        cv::Mat img, label;
        to_.Next(img, label);
        cv::imshow(FullTestName(test_info_), img);
        cv::waitKey();
    }
}
