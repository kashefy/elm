#include "core/distributionsampler.h"

#include <opencv2/imgproc.hpp>

#include "core/area.h"
#include "ts/ts.h"

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

/**
 * @brief Compare theoretical pdf for exponential distr.
 * to pdf generate from 1) pdf sampler 2) randexp()
 */
TEST(ExponentialPDFTest, Sample)
{
    const double PDF_END=10.;   // last x element in pdf function
    const int SIZE=30, N=2000;  // no. of bins, no. of samples to draw
    const float LAMBDA = 1.f;

    // generate exponential pdf
    MatF pdf(1, SIZE);
    double x = 0.;
    for (int i=0; i<SIZE; i++,
         x += PDF_END/static_cast<double>(SIZE)) {

        double f = exp(-static_cast<double>(LAMBDA)*x);
        pdf(i) = LAMBDA*static_cast<float>(f);
    }

    // add generated pdf to a sampler and draw samples from it
    DistributionSampler1D to; // test object
    to.pdf(pdf);

    // generate a histogram from sampled values
    // also sample values from randexp() function
    MatF hist1 = MatF::zeros(1, SIZE);
    MatF hist2 = MatF::zeros(1, SIZE);

    for(int i=0; i<N; i++) {

        // collect samples drawn from pdf into a histogram
        int sampled_index = to.Sample();
        EXPECT_IN_CLOSED(sampled_index, 0, SIZE-1) << "Sample out of bounds.";
        hist1(sampled_index)++;

        // collect samples drawn from exp. sampling function into a second histogram
        float v = sem::randexp(LAMBDA);
        int bin = static_cast<int>(v*SIZE/PDF_END); // find the right bin for it
        if(bin < SIZE) {
            hist2(bin)++;
        }
    }

    EXPECT_EQ(cv::sum(hist1)(0), N);

    // compare each histogram of drawn samples to original exponential pdf
    MatF hist1_normalized = hist1/hist1(0);
    EXPECT_MAT_NEAR(hist1_normalized, pdf, 0.2f);

    MatF hist2_normalized = hist2/hist2(0);
    EXPECT_MAT_NEAR(hist2_normalized, pdf, 0.2f);
}

/**
 * @brief Compare samples from randexp() with different lambda values
 */
TEST(ExponentialPDFTest, Lambda)
{
    const float PDF_END=5.;   // last x element in pdf function
    const int SIZE=10, N=2000;  // no. of bins, no. of samples to draw
    const float PDF_SCALE_FACTOR=static_cast<float>(SIZE)/PDF_END;

    // generate a histogram from sampled values
    // also sample values from randexp() function
    std::vector<MatF> hists;

    // collect samples drawn from exp. sampling function into a second histogram
    int hist_index=0;
    for(float lambda=0.5f; lambda<=1.5f; lambda+=0.5f, hist_index++) {

        hists.push_back(MatF::zeros(1, SIZE)); // initialize histogram counts
        for(int i=0; i<N; i++) {

            float v = sem::randexp(lambda);
            int bin = static_cast<int>(v*PDF_SCALE_FACTOR); // find the right bin for it
            if(bin < SIZE) {
                hists[hist_index](bin)++;
            }
        }
    }

    // compare pdf amplitudes
    for(int i=1; i<static_cast<int>(hists.size()); i++) {

        // left part of pdf
        cv::Mat gt = hists[i] > hists[i-1];

        EXPECT_EQ(cv::countNonZero(gt.colRange(0, 2)), 2)
                << "hist[" << i <<"] <= hist[" << i-1 << "]";

        // pdf tail
        cv::Mat lt = hists[i] < hists[i-1];
        EXPECT_EQ(cv::countNonZero(lt.colRange(lt.cols-4, lt.cols-2)), 2)
                << "hist[" << i <<"] >= hist[" << i-1 << "]";
    }
}

TEST(ExponentialPDFTest, PDFArea)
{
    const float PDF_END=10.;   // last x element in pdf function
    const int SIZE=50, N=2000;  // no. of bins, no. of samples to draw
    const float PDF_SCALE_FACTOR=static_cast<float>(SIZE)/PDF_END;

    MatF x(1, SIZE);
    float x_val = 0.f;
    for(int i=0; i<SIZE; i++) {

        x(i) = x_val;
        x_val += 1.f/PDF_SCALE_FACTOR;
    }

    // generate a histogram from sampled values
    // also sample values from randexp() function
    std::vector<MatF> hists;

    // collect samples drawn from exp. sampling function into a second histogram
    int hist_index=0;
    for(float lambda=0.5f; lambda<=1.5f; lambda+=0.5f, hist_index++) {

        hists.push_back(MatF::zeros(1, SIZE)); // initialize histogram counts
        for(int i=0; i<N; i++) {

            float v = sem::randexp(lambda);
            int bin = static_cast<int>(v*PDF_SCALE_FACTOR); // find the right bin for it
            if(bin < SIZE) {
                hists[hist_index](bin)++;
            }
        }
    }

    // check moments
    double lambda = 0.5;
    for(int i=0; i<static_cast<int>(hists.size()); i++) {

        MatF pdf;
        cv::divide(hists[i], cv::sum(hists[i]/PDF_SCALE_FACTOR)(0), pdf);

        EXPECT_NEAR(Trapz()(x, pdf), 1.f, 0.2f);
        lambda += 0.5;
    }
}

} // namespace
