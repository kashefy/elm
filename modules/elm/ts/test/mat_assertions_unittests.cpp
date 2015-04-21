/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** Write unit tests around Mat assertions.
 * @todo: Refactor into TYPED_TEST to reduce clutter
 */
#include "elm/ts/ts.h"

#include "elm/core/exception.h"
#include "elm/core/typedefs.h"

using namespace cv;
using namespace elm;

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

// test equality with template Mat objects

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims and elem. values)
 */
TEST(MatAssertionsTest, Mat_Eq) {

    const int R = 3;
    const int C = 2;
    Mat1f a = Mat1i::zeros(R, C);
    Mat1i b = Mat1f::zeros(R, C);

    ASSERT_EQ(a.rows, R) << "No. of rows do not match.";
    ASSERT_EQ(a.cols, C) << "No. of columns do not match.";
    ASSERT_EQ(a.rows, b.rows) << "No. of rows do not match.";
    ASSERT_EQ(a.cols, b.cols) << "No. of columns do not match.";
    ASSERT_EQ(a.total(), b.total()) << "No. of elements do not match.";
    ASSERT_EQ(static_cast<int>(a.total()), R*C) << "No. of elements do not match.";
    ASSERT_TRUE( EqualDims(a, b) );

    EXPECT_MAT_DIMS_EQ(a, b);
    EXPECT_MAT_DIMS_EQ(a, b.size());
    EXPECT_MAT_DIMS_EQ(a, Size2i(C, R));

    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {

            EXPECT_FLOAT_EQ(a(r, c), 0.f);
            EXPECT_FLOAT_EQ(static_cast<float>(b(r, c)), 0.f);
            EXPECT_FLOAT_EQ(a(r, c), static_cast<float>(b(r, c)));
        }
    }
    EXPECT_TRUE( Equal(a, b) );
    EXPECT_MAT_EQ(a, b);
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims, non-equal elem. values)
 */
TEST(MatAssertionsTest, Mat_NotEq) {

    const int R=3, C=2;
    Mat1f a = Mat1f::zeros(R, C);
    Mat1i b = Mat1i::ones(R, C);

    EXPECT_MAT_DIMS_EQ(a, b);
    EXPECT_MAT_DIMS_EQ(a, b.size());
    EXPECT_MAT_DIMS_EQ(a, Size2i(C, R));
    EXPECT_FALSE( Equal(a, b) );
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims, single value different)
 */
TEST(MatAssertionsTest, Mat_NotEqSingleEl) {

    const int R=3, C=2;
    Mat1f a = Mat1f::ones(R, C);

    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {

            Mat1f b = a.clone();
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
TEST(MatAssertionsTest, Mat_NotEqDims) {

    Mat1f a = Mat1f::zeros(3, 2);
    Mat1i b = Mat1i::ones(2, 3);

    EXPECT_FALSE( EqualDims(a, b) );
    EXPECT_FALSE( Equal(a, b) );
}

/**
 * @brief repeat Mat1fNotEqDims test with cv::Size overload
 */
TEST(MatAssertionsTest, Mat_NotEqDims_size) {

    Mat1f a = Mat1f::zeros(3, 2);
    Size2i s(3, 2);
    Mat1i b = Mat1i::ones(s);

    EXPECT_FALSE( EqualDims(a, s) );
    EXPECT_FALSE( EqualDims(a, b.size()) );
    EXPECT_FALSE( Equal(a, b) );
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (non-equal dims, equal elem. values)
 */
TEST(MatAssertionsTest, Mat_NotEqDimsAllZeros) {

    Mat1f a = Mat1f::zeros(3, 2);

    EXPECT_FALSE( EqualDims(a, a.t()) );
    EXPECT_FALSE( Equal(a, a.t()) );
}

/**
 * @brief repeat Mat1fNotEqDimsAllZeros test with cv::Size overload
 */
TEST(MatAssertionsTest, Mat_NotEqDimsAllZeros_size) {

    Mat1f a = Mat1f::zeros(3, 2);

    EXPECT_FALSE( EqualDims(a, Size2i(3, 2)) );
    EXPECT_FALSE( EqualDims(a, a.t().size()) );
    EXPECT_FALSE( Equal(a, a.t()) );
}

/**
 * @brief Test SparseMat assertions
 */
TEST(SparseMatAssertionsTest, Mat1fEq) {

    const int R = 3;
    const int C = 2;
    SparseMat a(Mat::zeros(R, C, CV_32FC1));
    Mat b = Mat::zeros(R, C, CV_32FC1);

    EXPECT_EQ(a.size(0), R) << "No. of rows do not match.";
    EXPECT_EQ(a.size(1), C) << "No. of columns do not match.";
    EXPECT_EQ(a.size(0), b.rows) << "No. of rows do not match.";
    EXPECT_EQ(a.size(1), b.cols) << "No. of columns do not match.";
    EXPECT_EQ(static_cast<size_t>(a.size(0)*a.size(1)), b.total()) << "No. of elements do not match.";
    EXPECT_EQ(a.size(0)*a.size(1), R*C) << "No. of elements do not match.";
    EXPECT_TRUE( EqualDims(a, b) );
    EXPECT_TRUE( EqualDims(b, a) );
    EXPECT_MAT_DIMS_EQ(a, b);
    EXPECT_MAT_DIMS_EQ(b, a);
    EXPECT_MAT_DIMS_EQ(a, b.size());
    EXPECT_MAT_DIMS_EQ(a, Size2i(C, R));

    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {

            EXPECT_FLOAT_EQ(a.ref<float>(r, c), 0.f);
            EXPECT_FLOAT_EQ(b.at<float>(r, c), 0.f);
            EXPECT_FLOAT_EQ(a.ref<float>(r, c), b.at<float>(r, c));
        }
    }
    EXPECT_TRUE( Equal(a, b) );
    EXPECT_TRUE( Equal(b, a) );
    EXPECT_MAT_EQ(a, b);
    EXPECT_MAT_EQ(b, a);
}

TEST(SparseMatAssertionsTest, SparseMat1fEq) {

    const int R = 3;
    const int C = 2;
    SparseMat a(Mat::ones(R, C, CV_32FC1));
    SparseMat b(Mat::ones(R, C, CV_32FC1));

    EXPECT_EQ(a.dims(), 2) << "Dims do not match.";
    EXPECT_EQ(a.dims(), b.dims()) << "Dims do not match.";
    EXPECT_EQ(a.size(0), R) << "No. of rows do not match.";
    EXPECT_EQ(a.size(1), C) << "No. of columns do not match.";
    EXPECT_EQ(a.size(0), b.size(0)) << "No. of rows do not match.";
    EXPECT_EQ(a.size(1), b.size(1)) << "No. of columns do not match.";
    EXPECT_EQ(a.size(0)*a.size(1), b.size(0)*b.size(1)) << "No. of elements do not match.";
    EXPECT_EQ(a.size(0)*a.size(1), R*C) << "No. of elements do not match.";
    EXPECT_TRUE( EqualDims(a, b) );
    EXPECT_MAT_DIMS_EQ(a, b);
    EXPECT_MAT_DIMS_EQ(a, Size2i(C, R));

    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {

            EXPECT_FLOAT_EQ(a.ref<float>(r, c), 1.f);
            EXPECT_FLOAT_EQ(b.ref<float>(r, c), 1.f);
            EXPECT_FLOAT_EQ(a.ref<float>(r, c), b.ref<float>(r, c));
        }
    }
    EXPECT_TRUE( Equal(a, b) );
    EXPECT_MAT_EQ(a, b);
}

TEST(SparseMatAssertionsTest, NDims) {

    const int N = 10;

    for(int n=1; n<N; n++) {

        int dims = n;
        int *sz = new int[dims];
        for(int i=0; i<dims; i++) {

            sz[i] = i+1;
        }

        SparseMat a(dims, sz, CV_32FC1);
        SparseMat b(dims, sz, CV_32FC1);
        Mat d(dims, sz, CV_32FC1);


        // c has different dims
        sz[n-1]++;
        SparseMat c(dims, sz, CV_32FC1);

        EXPECT_TRUE( EqualDims(a, b) );
        EXPECT_MAT_DIMS_EQ(a, b);
        EXPECT_FALSE( EqualDims(a, c) );

        if(n>2) {

            EXPECT_THROW( EqualDims(a, d), ExceptionNotImpl) << "update test if this is no longer applicable.";
            EXPECT_THROW( EqualDims(d, a), ExceptionNotImpl) << "update test if this is no longer applicable.";
        }
        else if(n==2) { // dims always >= 2 for dense Mat

            EXPECT_TRUE( EqualDims(a, d) );
            EXPECT_TRUE( EqualDims(d, a) );
            EXPECT_FALSE( EqualDims(c, d) );
            EXPECT_FALSE( EqualDims(d, c) );
        }

        delete []sz;
    }
}

TEST(SparseMatAssertionsTest, BadDims) {

    {
        int sz[1] = {3};
        SparseMat a(1, sz, CV_32FC1);
        EXPECT_THROW( EqualDims(a, Size2i(1, sz[0])), ExceptionBadDims);
        EXPECT_THROW( EqualDims(a, Size2i(sz[0], 1)), ExceptionBadDims);
    }
    {
        int sz[3] = {3, 4, 5};
        SparseMat a(1, sz, CV_32FC1);
        EXPECT_THROW( EqualDims(a, Size2i(sz[0], sz[1])), ExceptionBadDims);
        EXPECT_THROW( EqualDims(a, Size2i(sz[1], sz[0])), ExceptionBadDims);
        EXPECT_THROW( EqualDims(a, Size2i(sz[2], sz[1])), ExceptionBadDims);
        EXPECT_THROW( EqualDims(a, Size2i(sz[1], sz[2])), ExceptionBadDims);
        EXPECT_THROW( EqualDims(a, Size2i(sz[2], sz[0])), ExceptionBadDims);
        EXPECT_THROW( EqualDims(a, Size2i(sz[0], sz[2])), ExceptionBadDims);
    }
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims, non-equal elem. values)
 */
TEST(SparseMatAssertionsTest, Mat1fNotEq) {

    const int R=3, C=2;
    SparseMat a(Mat::zeros(R, C, CV_32FC1));
    Mat b = Mat::ones(R, C, CV_32FC1);

    EXPECT_MAT_DIMS_EQ(a, b);
    EXPECT_MAT_DIMS_EQ(a, b.size());
    EXPECT_MAT_DIMS_EQ(a, Size2i(C, R));
    EXPECT_FALSE( Equal(a, b) );
    EXPECT_FALSE( Equal(b, a) );
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims, single value different)
 */
TEST(SparseMatAssertionsTest, Mat1fNotEqSingleEl) {

    const int R=3, C=2;
    Mat tmp = Mat::ones(R, C, CV_32FC1);
    SparseMat a(tmp);

    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {

            Mat b = tmp.clone();
            EXPECT_MAT_DIMS_EQ(a, b);
            EXPECT_MAT_DIMS_EQ(b, a);
            EXPECT_MAT_DIMS_EQ(a, b.size());
            EXPECT_MAT_DIMS_EQ(a, Size2i(C, R));
            EXPECT_MAT_EQ(a, b);
            EXPECT_MAT_EQ(b, a);
            b.at<float>(r, c) += 123;
            EXPECT_FALSE( Equal(a, b) );
            EXPECT_FALSE( Equal(b, a) );
        }
    }
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (non-equal dims, non-equal elem. values)
 */
TEST(SparseMatAssertionsTest, Mat1fNotEqDims) {

    SparseMat a(Mat::zeros(3, 2, CV_32FC1));
    Mat b = Mat::ones(2, 3, CV_32FC1);

    EXPECT_FALSE( EqualDims(a, b) );
    EXPECT_FALSE( EqualDims(b, a) );
    EXPECT_FALSE( Equal(a, b) );
    EXPECT_FALSE( Equal(b, a) );
}

TEST(SparseMatAssertionsTest, DimsMismatch) {

    const int _sizes[1] = {2};
    SparseMat a(1, _sizes, CV_32FC1);
    Mat b = Mat::ones(2, 3, CV_32FC1);

    EXPECT_FALSE( EqualDims(a, b) );
    EXPECT_FALSE( EqualDims(b, a) );
    EXPECT_FALSE( Equal(a, b) );
    EXPECT_FALSE( Equal(b, a) );
}

/**
 * @brief repeat Mat1fNotEqDims test with cv::Size overload
 */
TEST(SparseMatAssertionsTest, Mat1fNotEqDims_size) {

    SparseMat a(Mat::zeros(3, 2, CV_32FC1));
    Size2i s(3, 2);
    Mat b = Mat::ones(s, CV_32FC1);

    EXPECT_FALSE( EqualDims(a, s) );
    EXPECT_FALSE( EqualDims(a, b.size()) );
    EXPECT_FALSE( Equal(a, b) );
    EXPECT_FALSE( Equal(b, a) );
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (non-equal dims, equal elem. values)
 */
TEST(SparseMatAssertionsTest, Mat1fNotEqDimsAllZeros) {

    Mat tmp = Mat::zeros(3, 2, CV_32FC1);
    SparseMat a(tmp);

    EXPECT_FALSE( EqualDims(a, tmp.t()) );
    EXPECT_FALSE( Equal(a, tmp.t()) );
}


/**
 * @brief repeat Mat1fNotEqDimsAllZeros test with cv::Size overload
 */
TEST(SparseMatAssertionsTest, Mat1fNotEqDimsAllZeros_size) {

    Mat tmp = Mat::zeros(3, 2, CV_32FC1);
    SparseMat a(tmp);

    EXPECT_FALSE( EqualDims(a, Size2i(3, 2)) );
    EXPECT_FALSE( EqualDims(a, tmp.t().size()) );
    EXPECT_FALSE( Equal(a, tmp.t()) );
}

/**
 * @brief test EqualDims with empty sparse matrix
 */
TEST(SparseMatAssertionsTest, MatEqDimsEmpty) {

    EXPECT_TRUE( EqualDims(SparseMat(), Size2i(0, 0)) );
    EXPECT_TRUE( EqualDims(SparseMat(), Mat()) );
    EXPECT_TRUE( EqualDims(SparseMat(), Mat().size()) );
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (equal dims, single value different)
 */
TEST(SparseMatAssertionsTest, Mat_NotEqSingleEl) {

    const int R=3, C=2;
    SparseMat1f a(Mat1f::ones(R, C));

    for(int r=0; r<R; r++) {
        for(int c=0; c<C; c++) {

            Mat1f b;
            a.convertTo(b, CV_32FC1);

            EXPECT_MAT_DIMS_EQ(a, b);
            EXPECT_MAT_DIMS_EQ(b, a);
            EXPECT_MAT_DIMS_EQ(a, b.size());
            EXPECT_MAT_DIMS_EQ(a, Size2i(C, R));
            EXPECT_MAT_EQ(a, b);
            EXPECT_MAT_EQ(b, a);
            b.at<float>(r, c) += 123;
            EXPECT_FALSE( Equal(a, b) );
            EXPECT_FALSE( Equal(b, a) );
        }
    }
}

/**
 * @brief test OpenCV Mat Assertions with 2 Mats of float (non-equal dims, non-equal elem. values)
 */
TEST(SparseMatAssertionsTest, Mat_NotEqDims) {

    SparseMat1f a(Mat1f::zeros(3, 2));
    Mat1i b = Mat1i::ones(2, 3);

    EXPECT_FALSE( EqualDims(a, b) );
    EXPECT_FALSE( EqualDims(b, a) );
    EXPECT_FALSE( Equal(a, b) );
    EXPECT_FALSE( Equal(b, a) );
}

/**
 * @brief repeat Mat1fNotEqDims test with cv::Size overload
 */
TEST(SparseMatAssertionsTest, Mat_NotEqDims_size) {

    SparseMat1f a(Mat1f::zeros(3, 2));
    Size2i s(3, 2);
    Mat1i b = Mat1i::ones(s);

    EXPECT_FALSE( EqualDims(a, s) );
    EXPECT_FALSE( EqualDims(a, b.size()) );
    EXPECT_FALSE( Equal(a, b) );
}

TEST(SparseMatAssertionsTest, Mat_NotEqDimsAllZeros) {

    Mat1f a = Mat1f::zeros(3, 2);
    SparseMat1f b(a);

    EXPECT_FALSE( EqualDims(b, a.t()) );
}

TEST(SparseMatAssertionsTest, Mat_NotEqDimsAllZeros_size) {

    SparseMat1f a(Mat1f::zeros(3, 2));

    EXPECT_FALSE( EqualDims(a, Size2i(3, 2)) );
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
    EXPECT_FALSE( LT<float>(Mat1f::zeros(3, 1), -1.f) );
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

/**
 * @brief repeat tests with different types of mat objects (int, float, uchar)
 */
template <class T>
class MatPODTypesTest : public testing::Test
{
};

template<class T>
struct V_
{
    static std::vector<T> values;
};

TYPED_TEST_CASE_P(MatPODTypesTest);

TYPED_TEST_P(MatPODTypesTest, TestLT_Empty) {

    std::vector<TypeParam> v = V_<TypeParam>::values;
    typedef Mat_<TypeParam> MatTP;
    EXPECT_FALSE( LT<TypeParam>(MatTP::zeros(1, 0), v[1]) );
    EXPECT_FALSE( LT<TypeParam>(MatTP::zeros(0, 1), v[1]) );
    EXPECT_FALSE( LT<TypeParam>(MatTP(), v[1]) );
}

/**
 * @brief Compare matrix against scalar value
 */
TYPED_TEST_P(MatPODTypesTest, TestLT_Value) {

    std::vector<TypeParam> v = V_<TypeParam>::values;
    typedef Mat_<TypeParam> MatTP;

    // It is necessary spell out the type when calling LT() and passing it a MatExpr (e.g. Mat1f::zeros().
    // Otherwise, the compiler cannot detect the template type and refuse to build.
    EXPECT_FALSE( LT<TypeParam>(MatTP::zeros(1, 1), v[1]) );
    EXPECT_FALSE( LT<TypeParam>(MatTP::zeros(3, 1), v[1]) );
    EXPECT_FALSE( LT<TypeParam>(MatTP(3, 1, v[4]), v[3]) );

    EXPECT_FALSE( LT<TypeParam>(MatTP::zeros(1, 1), v[1]) );
    EXPECT_FALSE( LT<TypeParam>(MatTP::zeros(1, 3), v[1]) );
    EXPECT_FALSE( LT<TypeParam>(MatTP(1, 3, v[4]), v[3]) );

    EXPECT_FALSE( LT<TypeParam>(MatTP::zeros(5, 3), v[1]) );
    EXPECT_FALSE( LT<TypeParam>(MatTP(5, 3, v[4]), v[3]) );

    EXPECT_TRUE( LT<TypeParam>(MatTP(1, 1, v[3]), v[4] ) );
    EXPECT_TRUE( LT<TypeParam>(MatTP(3, 5, v[3]), v[4] ) );
    EXPECT_TRUE( LT<TypeParam>(MatTP(3, 2, v[3]), v[4] ) );
    EXPECT_TRUE( LT<TypeParam>(MatTP(1, 3, v[3]), v[4] ) );
    EXPECT_TRUE( LT<TypeParam>(MatTP(3, 1, v[3]), v[4] ) );
    EXPECT_MAT_LT(MatTP(3, 1, v[3]), v[4]); // call the macro
}

/**
 * @brief Comapre single-valued matrices
 */
TYPED_TEST_P(MatPODTypesTest, TestLT_MatConstValue) {

    std::vector<TypeParam> v = V_<TypeParam>::values;
    typedef Mat_<TypeParam> MatTP;

    EXPECT_FALSE( LT(MatTP::zeros(1, 1), MatTP::zeros(1, 1)) );
    EXPECT_FALSE( LT(MatTP::zeros(3, 1), MatTP::zeros(3, 1)) );
    EXPECT_FALSE( LT(MatTP(3, 1, v[4]), MatTP(3, 1, v[3])) );

    EXPECT_FALSE( LT(MatTP(3, 1, v[3]), MatTP(1, 3, v[4])) ) << "Dimensions must match";

    EXPECT_TRUE( LT(MatTP(3, 1, v[3]), MatTP(3, 1, v[4])) );
    EXPECT_TRUE( LT(MatTP(1, 3, v[3]), MatTP(1, 3, v[4])) );
    EXPECT_MAT_LT(MatTP(3, 3, v[3]), MatTP(3, 3, v[4])); // call the macro
}

/**
 * @brief Fill matrices with different values and verify '<' comparison
 */
TYPED_TEST_P(MatPODTypesTest, TestLT_MatMixedValues) {

    std::vector<TypeParam> v = V_<TypeParam>::values;
    typedef Mat_<TypeParam> MatTP;

    MatTP a(2, 1), b(2, 1);
    for(int i=0; i<2; i++) { // transpose after first iteration

        a(0) = v[1];
        a(1) = v[2];

        b(0) = v[3];
        b(1) = v[4];


        EXPECT_FALSE( LT(b, a) );
        EXPECT_TRUE( LT(a, b) );
        EXPECT_MAT_LT(a, b); // call the macro

        b(0) = a(0);
        EXPECT_FALSE( LT(a, b) );

        // transpose both matrices and redo the checks
        a = a.t();
        b = b.t();
    }
}

/* Write additional type+value parameterized tests here.
 * Acquaint yourself with the values passed to along with each type.
 */

// Register test names
REGISTER_TYPED_TEST_CASE_P(MatPODTypesTest,
                           TestLT_Empty,
                           TestLT_Value,
                           TestLT_MatConstValue,
                           TestLT_MatMixedValues); ///< register additional typed_test_p (i.e. unit test) routines here

// Register values to work with inside tests, note how they're used inside the tests
template<> std::vector<float> V_<float>::values{-1.f, 0.f, 1.f, 100.f, 101.f};
template<> std::vector<int> V_<int>::values{-1, 0, 1, 100, 101};
template<> std::vector<uchar> V_<uchar>::values{0, 0, 1, 100, 101};

typedef testing::Types<float, int, uchar> PODTypes;  ///< // lists the usual suspects of matrices
INSTANTIATE_TYPED_TEST_CASE_P(MatAssertionsPODTypesTest, MatPODTypesTest, PODTypes);

} // namespace

