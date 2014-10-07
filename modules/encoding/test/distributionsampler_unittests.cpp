#include "encoding/distributionsampler1d.h"

#include "gtest/gtest.h"
#include "ts/ts.h"
#include <iostream>

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

} // namespace
