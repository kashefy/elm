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

} // namespace
