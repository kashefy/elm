#include "ts/ts.h"

using namespace cv;

namespace {

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims and elem. values)
 */
TEST(MatAssertionsTest, MatFEq) {

    const int R = 3;
    const int C = 2;
    Mat a = Mat::zeros(R, C, CV_32FC1);
    Mat b = Mat::zeros(R, C, CV_32FC1);

    EXPECT_EQ(a.rows, R) << "No. of rows do not match.";
    EXPECT_EQ(a.cols, C) << "No. of columns do not match.";
    EXPECT_EQ(a.rows, b.rows) << "No. of rows do not match.";
    EXPECT_EQ(a.cols, b.cols) << "No. of columns do not match.";
    EXPECT_EQ(a.total(), b.total()) << "No. of elements do not match.";
    EXPECT_EQ(a.total(), R*C) << "No. of elements do not match.";
    EXPECT_TRUE( EqualDims(a, b) );
    EXPECT_MAT_DIMS_EQ(a, b);

    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {

            EXPECT_FLOAT_EQ(a.at<float>(r, c), 0.f);
            EXPECT_FLOAT_EQ(b.at<float>(r, c), 0.f);
            EXPECT_FLOAT_EQ(a.at<float>(r, c), b.at<float>(r, c));
        }
    }
    EXPECT_TRUE( Equal(a, b) );
    EXPECT_MAT_EQ(a, b);
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims, non-equal elem. values)
 */
TEST(MatAssertionsTest, MatFNotEq) {

    const int R=3, C=2;
    Mat a = Mat::zeros(R, C, CV_32FC1);
    Mat b = Mat::ones(R, C, CV_32FC1);

    EXPECT_MAT_DIMS_EQ(a, b);
    EXPECT_FALSE( Equal(a, b) );
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims, single value different)
 */
TEST(MatAssertionsTest, MatFNotEqSingleEl) {

    const int R=3, C=2;
    Mat a = Mat::ones(R, C, CV_32FC1);

    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {

            Mat b = a.clone();
            EXPECT_MAT_DIMS_EQ(a, b);
            EXPECT_MAT_EQ(a, b);
            b.at<float>(r, c) += 123;
            EXPECT_FALSE( Equal(a, b) );
        }
    }
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (non-equal dims, non-equal elem. values)
 */
TEST(MatAssertionsTest, MatFNotEqDims) {

    Mat a = Mat::zeros(3, 2, CV_32FC1);
    Mat b = Mat::ones(2, 3, CV_32FC1);

    EXPECT_FALSE( EqualDims(a, b) );
    EXPECT_FALSE( Equal(a, b) );
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (non-equal dims, equal elem. values)
 */
TEST(MatAssertionsTest, MatFNotEqDimsAllZeros) {

    Mat a = Mat::zeros(3, 2, CV_32FC1);
    Mat b = Mat::zeros(2, 3, CV_32FC1);

    EXPECT_FALSE( EqualDims(a, b) );
    EXPECT_FALSE( Equal(a, b) );
}

/**
 * @brief test failure message
 */
TEST(MatAssertionsTest, FailureMessage) {

    Mat a = Mat::zeros(3, 2, CV_32FC1);
    Mat b = Mat::ones(3, 2, CV_32FC1);
    Mat cmp_out;
    compare(a, b, cmp_out, CMP_NE);
    EXPECT_GT( MatFailureMessageNonZero(a, b, cmp_out).size(), 0) << "Failure message is empty";
}

} // namespace

