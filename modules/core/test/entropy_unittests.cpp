#include "core/entropy.h"

#include "gtest/gtest.h"

#include "core/exception.h"

using namespace cv;
using namespace sem;

TEST(CondEntropyTest, Empty)
{
    EXPECT_FLOAT_EQ(0., CondEntropy(Mat1f()));
}

TEST(CondEntropyTest, Invalid)
{
    EXPECT_THROW(CondEntropy(Mat1f::zeros(1, 1)), ExceptionValueError);
}

TEST(CondEntropyTest, Uniform)
{
    for(int r=1; r<=20; r+=2) {

        for(int c=2; c<=5; c++) {

            Mat1f pdf(r, c, 1.f/static_cast<float>(r*c));
            EXPECT_GE(CondEntropy(pdf), -2.*logf(0.5)*0.5);
        }
    }
}

TEST(CondEntropyTest, CondEntropy)
{
    const int SIZE=3;
    float data[SIZE] = {0.1f, 0.1f, 0.8f};
    Mat pdf(1, SIZE, CV_32FC1, data);
    float cond_entropy = CondEntropy(pdf);
    EXPECT_LT(cond_entropy, -2.*logf(0.5)*0.5)
            << "Cond. Entropy must fall below that of 50/50 coin toss.";
    EXPECT_NEAR(cond_entropy, 0.639f, 1e-4);
}

TEST(CondEntropyTest, OrderInvariant)
{
    const int SIZE=3, N=10;
    float data[SIZE] = {0.1f, 0.1f, 0.8f};
    Mat pdf(1, SIZE, CV_32FC1, data);
    float cond_entropy_initial = CondEntropy(pdf);
    ASSERT_LT(cond_entropy_initial, -2.*logf(0.5)*0.5)
            << "Cond. Entropy must fall below that of 50/50 coin toss.";

    for(int i=0; i<N; i++) {

        randShuffle(pdf);
        EXPECT_FLOAT_EQ(cond_entropy_initial, CondEntropy(pdf));
    }
}
