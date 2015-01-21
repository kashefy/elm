#include "sem/core/entropy.h"

#include "sem/ts/ts.h"

#include "sem/core/exception.h"
#include "sem/core/stl/typedefs.h"

using namespace cv;
using namespace sem;

TEST(CondEntropyTest, Empty)
{
    EXPECT_FLOAT_EQ(0., CondEntropy(Mat1f()));
    EXPECT_FLOAT_EQ(0., CondEntropy(VecF()));
}

TEST(CondEntropyTest, Invalid)
{
    EXPECT_THROW(CondEntropy(Mat1f::zeros(1, 1)), ExceptionValueError);
    EXPECT_THROW(CondEntropy(VecF(1, 0)), ExceptionValueError);
}

TEST(CondEntropyTest, Uniform)
{
    for(int r=1; r<=20; r+=2) {


        for(int c=2; c<=5; c++) {

            float value = 1.f/static_cast<float>(r*c);
            Mat1f pdf(r, c, value);
            EXPECT_GE(CondEntropy(pdf), -2.*logf(0.5)*0.5);
            EXPECT_GE(CondEntropy(VecF(r*c, value)), -2.*logf(0.5)*0.5);
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

    // try again with vector of floats
    VecF pdf_vec(data, data+SIZE);
    cond_entropy = CondEntropy(pdf_vec);
    EXPECT_LT(cond_entropy, -2.*logf(0.5)*0.5)
            << "Cond. Entropy must fall below that of 50/50 coin toss.";
    EXPECT_NEAR(cond_entropy, 0.639f, 1e-4);
}

TEST(CondEntropyTest, InputUnchanged)
{
    const int SIZE=3;
    float data[SIZE] = {0.1f, 0.1f, 0.8f};
    Mat pdf(1, SIZE, CV_32FC1, data);
    Mat pdf0 = pdf.clone();
    CondEntropy(pdf);
    EXPECT_MAT_EQ(pdf0, pdf) << "Input changed.";

    VecF pdf_vec(data, data+SIZE);
    VecF pdf_vec0(pdf_vec);

    CondEntropy(pdf_vec);
    EXPECT_EQ(pdf_vec0, pdf_vec) << "Input changed.";
}

TEST(CondEntropyTest, OrderInvariant)
{
    const int SIZE=3, N=10;
    float data[SIZE] = {0.1f, 0.1f, 0.8f};
    Mat pdf(1, SIZE, CV_32FC1, data);
    float cond_entropy_initial = CondEntropy(pdf);
    ASSERT_LT(cond_entropy_initial, -2.*logf(0.5)*0.5)
            << "Cond. Entropy must fall below that of 50/50 coin toss.";

    VecF pdf_vec(data, data+SIZE);

    for(int i=0; i<N; i++) {

        randShuffle(pdf);
        EXPECT_FLOAT_EQ(cond_entropy_initial, CondEntropy(pdf));

        randShuffle(pdf_vec);
        EXPECT_FLOAT_EQ(cond_entropy_initial, CondEntropy(pdf_vec));
    }
}
