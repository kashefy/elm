#include "core/distributionsampler.h"

#include "ts/ts.h"
#include <opencv2/imgproc.hpp>

namespace {

TEST(DistrSampler1D, Uniform) {

    const int SIZE = 10, N = 1e4;
    const float MEAN = 1.f/static_cast<float>(SIZE);
    MatF pdf_uniform = MatF::ones(1, SIZE)*MEAN;

    DistributionSampler1D to; // test object
    to.pdf(pdf_uniform);

    MatF hist = MatF::zeros(1, SIZE);

    for(int i=0; i<N; i++) {

        int sampled_index = to.Sample();
        EXPECT_IN_CLOSED(sampled_index, 0, SIZE-1);
        hist(sampled_index)++;
    }

    EXPECT_EQ(cv::sum(hist)(0), N);
    MatF hist_normalized = hist/static_cast<float>(N);
    EXPECT_MAT_NEAR(hist_normalized, pdf_uniform, 0.2f);

    cv::Mat m, s;
    cv::meanStdDev(hist_normalized, m, s);
    EXPECT_NEAR(m.at<double>(0, 0), MEAN, 0.1);
    EXPECT_NEAR(s.at<double>(0, 0), 0., 0.1);
}

TEST(DistrSampler1D, Gaussian) {

    const int SIZE = 10, N=2000;
    const float MEAN = 5.f, STD_DEV = 2.f;

    // generate gaussian pdf
    cv::Mat data(1, N, CV_32FC1);
    cv::randn(data, MEAN, STD_DEV);

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

    EXPECT_EQ(cv::sum(hist)(0), N);
    MatF hist_normalized = hist/static_cast<float>(N)*SIZE;
    EXPECT_MAT_NEAR(hist_normalized, pdf, 0.2f);
}

TEST(DistrSampler2D, Uniform) {

    const int ROWS = 7, COLS = 10, N = 2e4;
    const float MEAN = 1.f/static_cast<float>(ROWS*COLS);
    MatF pdf_uniform = MatF::ones(ROWS, COLS)*MEAN;

    DistributionSampler2D to; // test object
    to.pdf(pdf_uniform);

    MatF hist = MatF::zeros(ROWS, COLS);

    for(int i=0; i<N; i++) {

        cv::Point2i sampled_point = to.Sample();
        EXPECT_TRUE( sampled_point.inside(cv::Rect(0, 0, COLS, ROWS)) );
        hist(sampled_point.y, sampled_point.x)++;
    }

    EXPECT_EQ(cv::sum(hist)(0), N);
    MatF hist_normalized = hist/static_cast<float>(N);
    EXPECT_MAT_NEAR(hist_normalized, pdf_uniform, 0.2f);

    cv::Mat m, s;
    cv::meanStdDev(hist_normalized, m, s);
    EXPECT_NEAR(m.at<double>(0, 0), MEAN, 0.1);
    EXPECT_NEAR(s.at<double>(0, 0), 0., 0.1);
}

class PoissonProcessTest : public testing::Test
{
protected:
    PoissonProcessTest()
        : frequency_(1.f),
          delta_t_msec_(1000.f),
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

    float lambda = pdf(0, 0);
    EXPECT_LE(lambda, 0);
    EXPECT_FLOAT_EQ(lambda, 1.f-frequency_*delta_t_msec_/1000.f);
}

TEST_F(PoissonProcessTest, Sample)
{
    const int N = 100;
    for(int i=0; i<N; i++) {

        EXPECT_EQ(1, to_.Sample());
    }
}

TEST_F(PoissonProcessTest, SampleZeroFreq)
{
    PoissonProcess to(0.f, 1.f);
    const int N = 100;
    for(int i=0; i<N; i++) {

        EXPECT_EQ(0, to.Sample());
    }
}

TEST_F(PoissonProcessTest, SampleZeroDeltaT)
{
    PoissonProcess to(40.f, 0.f);
    const int N = 100;
    for(int i=0; i<N; i++) {

        EXPECT_EQ(0, to.Sample());
    }
}

TEST_F(PoissonProcessTest, SampleZeroFreqAndDeltaT)
{
    PoissonProcess to(0.f, 0.f);
    const int N = 100;
    for(int i=0; i<N; i++) {

        EXPECT_EQ(0, to.Sample());
    }
}

TEST_F(PoissonProcessTest, SampleCoinToss)
{
    PoissonProcess to(50.f, 10.f);
    const int N = 1e4;
    int net = 0;
    for(int i=0; i<N; i++) {

        net += to.Sample();
    }
    float rate = net/static_cast<float>(N);
    EXPECT_NEAR(rate, 0.5f, 0.02f);
}

} // namespace
