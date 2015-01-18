#include "sem/core/percentile.h"

#include "sem/core/exception.h"
#include "sem/ts/ts.h"

using namespace cv;

/**
 * @brief class for testing percentile calculation
 */
class PercentileTest : public ::testing::Test
{
protected:
    virtual void SetUp() {

        to_ = Percentile();

        data_ = Mat1f(1, 10);
        for(int i=0; i<static_cast<int>(data_.total()); i++) {

            data_(i) = static_cast<float>(i);
        }
        randShuffle(data_);
    }

    Percentile to_; ///< test object
    Mat1f data_;    ///< test data
};

/**
 * @brief Initialize input matrix to have at least one empty dimension.
 */
TEST_F(PercentileTest, Empty) {

    EXPECT_THROW(to_.CalcPercentile(Mat1f(), 0.5f), sem::ExceptionBadDims);
    EXPECT_THROW(to_.CalcPercentile(Mat1f(1, 0), 0.5f), sem::ExceptionBadDims);
    EXPECT_THROW(to_.CalcPercentile(Mat1f(0, 1), 0.5f), sem::ExceptionBadDims);
    EXPECT_THROW(to_.CalcPercentile(Mat1f(0, 0), 0.5f), sem::ExceptionBadDims);
}

/**
 * @brief throw invalid input at it
 */
TEST_F(PercentileTest, Invalid) {

    EXPECT_THROW(to_.CalcPercentile(data_, -0.01f), sem::ExceptionValueError);
    EXPECT_THROW(to_.CalcPercentile(data_, 1.01f), sem::ExceptionValueError);

    EXPECT_NO_THROW(to_.CalcPercentile(data_, 1.f));
    EXPECT_NO_THROW(to_.CalcPercentile(data_, 0.f));
    EXPECT_NO_THROW(to_.CalcPercentile(data_, -0.f));
}

/**
 * @brief test with one dimensional input
 */
TEST_F(PercentileTest, Percentile_OneDim) {

    for(float p=0.f; p<=1.f; p+=0.1f) {
        EXPECT_FLOAT_EQ(p*10.f, to_.CalcPercentile(data_, p));
    }
}

/**
 * @brief test with two-dimensonal input
 */
TEST_F(PercentileTest, Percentile_TwoDim) {

    repeat(data_, 3, 1);
    for(float p=0.f; p<=1.f; p+=0.1f) {
        EXPECT_FLOAT_EQ(p*10.f, to_.CalcPercentile(data_, p));
    }
}

/**
 * @brief duplicate a value inside the data
 */
TEST_F(PercentileTest, DuplicateValues) {

    data_(0) = 10.f;
    data_(1) = 10.f;
    EXPECT_FLOAT_EQ(10.f, to_.CalcPercentile(data_, 0.9f));
}
