#include "ts/ts.h"

#include "core/typedefs.h"

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
TEST(MatAssertionsTest, FailureMessage)
{
    Mat a = Mat::zeros(3, 2, CV_32FC1);
    Mat b = Mat::ones(3, 2, CV_32FC1);
    Mat cmp_out;
    compare(a, b, cmp_out, CMP_NE);
    EXPECT_GT( MatFailureMessageNonZero(a, b, cmp_out).size(), 0) << "Failure message is empty";
}

/**
 * Series of tests around Mat type failure message
 */
TEST(MatTypeAssertionsTest, FailureMessage)
{
    Mat matf = Mat::zeros(3, 2, CV_32FC1);
    EXPECT_GT( MatTypeFailureMessage(matf, CV_32S).size(), 0 ) << "Failure message is empty";
    EXPECT_GT( MatTypeFailureMessage(matf, CV_8UC1).size(), 0 ) << "Failure message is empty";
    EXPECT_GT( MatTypeFailureMessage(matf, CV_16SC2).size(), 0 ) << "Failure message is empty";
}

/**
 * Series of tests around Mat type assertions
 */
TEST(MatTypeAssertionsTest, Type)
{
    Mat a = Mat(1, 1, CV_32FC1);
    EXPECT_TRUE( IsType(a, CV_32F)) << "It's a 32-bit float matrix.";
    EXPECT_TRUE( IsType(Mat(1, 1, CV_32FC2), a.type()) ) << "It's a 32-bit float matrix.";
    EXPECT_FALSE( IsType(Mat(1, 1, CV_32SC2), a.type()) ) << " 32-bit signed int != 32-bit float.";
}

TEST(MatTypeAssertionsTest, TemplateType)
{
    MatF a = MatF(1, 1);
    EXPECT_TRUE( IsType(a, CV_32F)) << "It's a 32-bit float matrix.";
    EXPECT_TRUE( IsType(Mat(1, 1, CV_32FC2), a.type()) ) << "It's a 32-bit float matrix.";
    EXPECT_FALSE( IsType(Mat(1, 1, CV_32SC2), a.type()) ) << " 32-bit signed int != 32-bit float.";

    MatI b = MatI(1, 1);
    EXPECT_TRUE( IsType(b, CV_32S)) << "It's a 32-bit signed integer matrix.";
    EXPECT_TRUE( IsType(Mat(1, 1, CV_32SC2), b.type()) ) << "It's a 32-bit signed int matrix.";
    EXPECT_FALSE( IsType(Mat(1, 1, CV_16SC1), b.type()) ) << "16-bit signed int != 32-bit signed int.";
}

/**
 * @brief test utility function for getting mat type string representation
 * with same type but different number of channels
 */
TEST(MatTypeAssertionsTest, TypeChannels)
{
    Size s(1, 1);
    for(int ch=1; ch<=4; ch++) {

        EXPECT_TRUE( IsType(Mat(s, CV_MAKETYPE(CV_8U, ch)),  CV_8U) );
        EXPECT_TRUE( IsType(Mat(s, CV_MAKETYPE(CV_8S, ch)),  CV_8S) );
        EXPECT_TRUE( IsType(Mat(s, CV_MAKETYPE(CV_16U, ch)), CV_16U) );
        EXPECT_TRUE( IsType(Mat(s, CV_MAKETYPE(CV_16S, ch)), CV_16S) );
        EXPECT_TRUE( IsType(Mat(s, CV_MAKETYPE(CV_32S, ch)), CV_32S) );
        EXPECT_TRUE( IsType(Mat(s, CV_MAKETYPE(CV_32F, ch)), CV_32F) );
        EXPECT_TRUE( IsType(Mat(s, CV_MAKETYPE(CV_64F, ch)), CV_64F) );
    }
}

} // namespace

