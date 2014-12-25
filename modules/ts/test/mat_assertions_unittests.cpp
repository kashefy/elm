#include "ts/ts.h"

#include "core/typedefs.h"

using namespace cv;

namespace {

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims and elem. values)
 */
TEST(MatAssertionsTest, Mat1fEq) {

    const int R = 3;
    const int C = 2;
    Mat a = Mat::zeros(R, C, CV_32FC1);
    Mat b = Mat::zeros(R, C, CV_32FC1);

    EXPECT_EQ(a.rows, R) << "No. of rows do not match.";
    EXPECT_EQ(a.cols, C) << "No. of columns do not match.";
    EXPECT_EQ(a.rows, b.rows) << "No. of rows do not match.";
    EXPECT_EQ(a.cols, b.cols) << "No. of columns do not match.";
    EXPECT_EQ(a.total(), b.total()) << "No. of elements do not match.";
    EXPECT_EQ(static_cast<int>(a.total()), R*C) << "No. of elements do not match.";
    EXPECT_TRUE( EqualDims(a, b) );
    EXPECT_MAT_DIMS_EQ(a, b);
    EXPECT_MAT_DIMS_EQ(a, b.size());
    EXPECT_MAT_DIMS_EQ(a, Size2i(C, R));

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
TEST(MatAssertionsTest, Mat1fNotEq) {

    const int R=3, C=2;
    Mat a = Mat::zeros(R, C, CV_32FC1);
    Mat b = Mat::ones(R, C, CV_32FC1);

    EXPECT_MAT_DIMS_EQ(a, b);
    EXPECT_MAT_DIMS_EQ(a, b.size());
    EXPECT_MAT_DIMS_EQ(a, Size2i(C, R));
    EXPECT_FALSE( Equal(a, b) );
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims, single value different)
 */
TEST(MatAssertionsTest, Mat1fNotEqSingleEl) {

    const int R=3, C=2;
    Mat a = Mat::ones(R, C, CV_32FC1);

    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {

            Mat b = a.clone();
            EXPECT_MAT_DIMS_EQ(a, b);
            EXPECT_MAT_DIMS_EQ(a, b.size());
            EXPECT_MAT_DIMS_EQ(a, Size2i(C, R));
            EXPECT_MAT_EQ(a, b);
            b.at<float>(r, c) += 123;
            EXPECT_FALSE( Equal(a, b) );
        }
    }
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (non-equal dims, non-equal elem. values)
 */
TEST(MatAssertionsTest, Mat1fNotEqDims) {

    Mat a = Mat::zeros(3, 2, CV_32FC1);
    Mat b = Mat::ones(2, 3, CV_32FC1);

    EXPECT_FALSE( EqualDims(a, b) );
    EXPECT_FALSE( Equal(a, b) );
}

/**
 * @brief repeat Mat1fNotEqDims test with cv::Size overload
 */
TEST(MatAssertionsTest, Mat1fNotEqDims_size) {

    Mat a = Mat::zeros(3, 2, CV_32FC1);
    Size2i s(3, 2);
    Mat b = Mat::ones(s, CV_32FC1);

    EXPECT_FALSE( EqualDims(a, s) );
    EXPECT_FALSE( EqualDims(a, b.size()) );
    EXPECT_FALSE( Equal(a, b) );
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (non-equal dims, equal elem. values)
 */
TEST(MatAssertionsTest, Mat1fNotEqDimsAllZeros) {

    Mat a = Mat::zeros(3, 2, CV_32FC1);

    EXPECT_FALSE( EqualDims(a, a.t()) );
    EXPECT_FALSE( Equal(a, a.t()) );
}


/**
 * @brief repeat Mat1fNotEqDimsAllZeros test with cv::Size overload
 */
TEST(MatAssertionsTest, Mat1fNotEqDimsAllZeros_size) {

    Mat a = Mat::zeros(3, 2, CV_32FC1);

    EXPECT_FALSE( EqualDims(a, Size2i(3, 2)) );
    EXPECT_FALSE( EqualDims(a, a.t().size()) );
    EXPECT_FALSE( Equal(a, a.t()) );
}

/**
 * @brief test EqualDims with empty matrix
 */
TEST(MatAssertionsTest, MatEqDimsEmpty) {

    EXPECT_TRUE( EqualDims(Mat(), Size2i(0, 0)) );
    EXPECT_TRUE( EqualDims(Mat(), Mat()) );
    EXPECT_TRUE( EqualDims(Mat(), Mat().size()) );
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
    EXPECT_FALSE( MatFailureMessageNonZero(a, b, cmp_out).empty() ) << "Failure message is empty";
}

/**
 * Series of tests around Mat type failure message
 */
TEST(MatTypeAssertionsTest, FailureMessage)
{
    Mat matf = Mat::zeros(3, 2, CV_32FC1);
    EXPECT_FALSE( MatTypeFailureMessage(matf, CV_32S).empty() ) << "Failure message is empty";
    EXPECT_FALSE( MatTypeFailureMessage(matf, CV_8UC1).empty() ) << "Failure message is empty";
    EXPECT_FALSE( MatTypeFailureMessage(matf, CV_16SC2).empty() ) << "Failure message is empty";
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
    Mat1f a = Mat1f(1, 1);
    EXPECT_TRUE( IsType(a, CV_32F)) << "It's a 32-bit float matrix.";
    EXPECT_TRUE( IsType(Mat(1, 1, CV_32FC2), a.type()) ) << "It's a 32-bit float matrix.";
    EXPECT_FALSE( IsType(Mat(1, 1, CV_32SC2), a.type()) ) << " 32-bit signed int != 32-bit float.";

    Mat1i b = Mat1i(1, 1);
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

TEST(MatAssertionsTest, Empty) {

    Mat1i i;
    Mat1b b;
    Mat1f f;
    Mat m;
    EXPECT_TRUE( Empty(i) );
    EXPECT_TRUE( Empty(b) );
    EXPECT_TRUE( Empty(f) );
    EXPECT_TRUE( Empty(m) );

    // test macro
    EXPECT_EMPTY( i );
    EXPECT_EMPTY( b );
    EXPECT_EMPTY( f );
    EXPECT_EMPTY( m );

    i.push_back(123);
    b.push_back(Scalar_<uchar>(255));
    b.push_back(Scalar_<uchar>(0));

    EXPECT_EQ( size_t(1), i.total() );
    EXPECT_FALSE( Empty(i) );

    EXPECT_EQ( size_t(2), b.total() );
    EXPECT_FALSE( Empty(b) );

    i.pop_back();
    EXPECT_TRUE( Empty(i) );
    EXPECT_EMPTY( i );

    EXPECT_FALSE( Empty(Mat1f::zeros(1, 1)) );
}

TEST(MatAssertionsTest, LT)
{
    EXPECT_FALSE( LT(Mat1f(3, 1, 0.f), -1.f) );
    EXPECT_FALSE( LT(Mat1f(3, 1, 0.f), 0.f) );
    EXPECT_TRUE( LT(Mat1f(3, 1, 0.f), 1.f) );
    EXPECT_FALSE( LT<float>(Mat1f::zeros(3, 1), -1.f) ); // need to specify type because ::zeros returns MatExpr
    EXPECT_FALSE( LT<float>(Mat1f::zeros(3, 1), 0.f) );
    EXPECT_TRUE( LT<float>(Mat1f::zeros(3, 1), 1.f) );
    EXPECT_MAT_LT(Mat1f(3, 1, 0.f), 1.f);

    EXPECT_FALSE( LT(Mat1i(3, 1, 0), -1) );
    EXPECT_FALSE( LT(Mat1i(3, 1, 0), 0) );
    EXPECT_TRUE( LT(Mat1i(3, 1, 0), 1) );
    EXPECT_FALSE( LT<int>(Mat1i::zeros(3, 1), -1) );
    EXPECT_FALSE( LT<int>(Mat1i::zeros(3, 1), 0) );
    EXPECT_TRUE( LT<int>(Mat1i::zeros(3, 1), 1) );
    EXPECT_MAT_LT( Mat1i(3, 1, 0), 1);

    EXPECT_FALSE( LT<uchar>(Mat1b::zeros(3, 1), 0) );
    EXPECT_FALSE( LT<uchar>(Mat1b(3, 1, static_cast<uchar>(101)), static_cast<uchar>(100)) );
    EXPECT_TRUE( LT<uchar>(Mat1b(3, 1, static_cast<uchar>(100)), static_cast<uchar>(255)) );
    EXPECT_MAT_LT(Mat1b(3, 1, static_cast<uchar>(100)), static_cast<uchar>(255));
}

} // namespace

