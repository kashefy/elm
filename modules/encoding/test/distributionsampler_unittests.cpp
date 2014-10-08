#include "encoding/distributionsampler.h"

#include "ts/ts.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace {

TEST(DistrSampler1D, Uniform) {

    const int SIZE = 10, N=2000;
    MatF pdf_uniform = MatF::ones(1, SIZE);

    DistributionSampler1D to; // test object
    to.pdf(pdf_uniform);

    MatF hist = MatF::zeros(1, SIZE);

    for(int i=0; i<N; i++) {

        int sampled_index = to.Sample();
        EXPECT_IN_CLOSED(sampled_index, 0, SIZE-1);
        hist(sampled_index)++;
    }

    EXPECT_MAT_NEAR(hist/static_cast<float>(N)*SIZE, pdf_uniform, 0.2);

    cv::Mat m, s;
    cv::meanStdDev(hist, m, s);
    EXPECT_NEAR(s.at<float>(0, 0), 0, 1e-5);
}

TEST(DistrSampler1D, Gaussian) {

    const int SIZE = 10, N=2000;
    const float MEAN = 5.f, STD_DEV = 2.f;

    // generate gaussian pdf
    cv::Mat data(1, N, CV_32FC1);
    for(int i=0; i<N; i++) {

        cv::randn(data, MEAN, STD_DEV);
    }

    float pdf_values[SIZE] = {};
    for (int i=0; i<N; i++) {

        float v = data.at<float>(i);
        if ( v >= 0.f && v<10.0) {
            ++pdf_values[static_cast<int>(v)];
        }
    }

    DistributionSampler1D to; // test object
    cv::Mat pdf = cv::Mat(1,10, CV_32FC1, pdf_values)/static_cast<float>(N)*SIZE;
    to.pdf(pdf);

    MatF hist = MatF::zeros(1, SIZE);

    for(int i=0; i<N; i++) {

        int sampled_index = to.Sample();
        EXPECT_IN_CLOSED(sampled_index, 0, SIZE-1);
        hist(sampled_index)++;
    }

    EXPECT_MAT_NEAR(hist/static_cast<float>(N)*SIZE, pdf, 0.2f);
}

TEST(DistrSampler2D, Uniform) {

    const int ROWS = 7, COLS = 10, N = 2e4;
    MatF pdf_uniform = MatF::ones(ROWS, COLS);

    DistributionSampler2D to; // test object
    to.pdf(pdf_uniform);

    MatF hist = MatF::zeros(ROWS, COLS);

    for(int i=0; i<N; i++) {

        cv::Point2i sampled_point = to.Sample();
        EXPECT_TRUE( sampled_point.inside(cv::Rect(0, 0, COLS, ROWS)) );
        hist(sampled_point.y, sampled_point.x)++;
    }

    EXPECT_MAT_NEAR(hist/static_cast<float>(N)*ROWS*COLS, pdf_uniform, 0.2f);

    cv::Mat m, s;
    cv::meanStdDev(hist, m, s);
    EXPECT_NEAR(s.at<float>(0, 0), 0, 1e-5);
}

class PoissonProcessTest : public testing::Test
{
protected:
    PoissonProcessTest()
        : frequency_(-5.f),
          delta_t_msec_(1.f),
          to_(0.f, 0.f) // dummy initialization
    {

    }

    virtual void SetUp()
    {
        to_ = PoissonProcess(frequency_, delta_t_msec_); // -ve frequency => inf firing rate, 1 msec time resolution
    }

    float frequency_;
    float delta_t_msec_;

    PoissonProcess to_; ///< test object
};

TEST_F(PoissonProcessTest, PDF)
{
    MatF pdf = to_.pdf();
    EXPECT_MAT_DIMS_EQ(pdf, MatF(1, 1));

    float firing_prob = pdf(0, 0);
    EXPECT_LT(firing_prob, 0);
    EXPECT_FLOAT_EQ(firing_prob, frequency_*delta_t_msec_);
}

} // namespace
