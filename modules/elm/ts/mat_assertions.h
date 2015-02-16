/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/**@file OpenCV Mat Assertions
  */
#ifndef _ELM_TS_MAT_ASSERTIONS_H_
#define _ELM_TS_MAT_ASSERTIONS_H_

#include "gtest/gtest.h"

#include <string>
#include <opencv2/core/core.hpp>

::testing::AssertionResult IsType(const cv::Mat& a, int mat_type);
#define EXPECT_MAT_TYPE(a, mat_type) EXPECT_TRUE( IsType(a, mat_type) )

std::string MatTypeFailureMessage(const cv::Mat& a, int mat_type);

/**
  * Check if dimensions match
  * Note:
  *     EqualDims(Mat(), Size2i(0, 0)) yields true
  *     EqualDims(Mat(), Mat()) yields true
  *     EqualDims(Mat(), Mat().size()) yields true
  */
::testing::AssertionResult EqualDims(const cv::Mat& a, const cv::Mat& b);
::testing::AssertionResult EqualDims(const cv::Mat& a, const cv::Size2i& s);
::testing::AssertionResult EqualDims(const cv::Mat& a, const cv::SparseMat& b);
::testing::AssertionResult EqualDims(const cv::SparseMat& a, const cv::Mat& b);
::testing::AssertionResult EqualDims(const cv::SparseMat& a, const cv::SparseMat& b);
::testing::AssertionResult EqualDims(const cv::SparseMat& a, const cv::Size2i& s);
#define EXPECT_MAT_DIMS_EQ(a, b) EXPECT_TRUE( EqualDims(a, b) )

/**
 * @brief Generate Mat failure message for matrix-matrix comparisons,
 * indicating matrix values at which failures occured
 * @param a
 * @param b
 * @param cmp comparison output
 * @return failure message
 */
std::string MatFailureMessageNonZero(const cv::Mat& a, const cv::Mat& b, const cv::Mat& cmp);

/**
 * @brief Generate Mat failure message for matrix-scalar comparisons,
 * indicating matrix values at which failures occured
 * @param a
 * @param cmp comparison output
 * @return failure message
 */
std::string MatFailureMessageNonZero(const cv::Mat& a, const cv::Mat& cmp);

/** @brief assert all elements in two Mat objects are equal.
  */
::testing::AssertionResult Equal(const cv::Mat& a, const cv::Mat& b);

::testing::AssertionResult Equal(const cv::SparseMat& a, const cv::Mat& b);

::testing::AssertionResult Equal(const cv::Mat& a, const cv::SparseMat& b);

::testing::AssertionResult Equal(const cv::SparseMat& a, const cv::SparseMat& b);

/** Compare two template Mat of different types.
 *  Requires per-element comparison instead, since we cannot call cv::compare with Mats of different types
 */
template <class T1, class T2>
::testing::AssertionResult Equal(const cv::Mat_<T1>& a, const cv::Mat_<T2>& b)
{
    using namespace ::testing;
    AssertionResult equal_dims = EqualDims(a, b);
    if(equal_dims != AssertionSuccess()) { return equal_dims; }

    cv::Mat1b cmp_out(a.size());
    for(size_t i=0; i<a.total(); i++) {

        cmp_out(i) = a(i) != b(i);
    }
    int n = countNonZero(cmp_out);
    if(n == 0) { return AssertionSuccess(); }
    else {
        return AssertionFailure() << MatFailureMessageNonZero(a, b, cmp_out);
    }
}
#define EXPECT_MAT_EQ(a, b) EXPECT_TRUE( Equal(a, b) )

::testing::AssertionResult Near(const cv::Mat& a, const cv::Mat& b, float tolerance);
#define EXPECT_MAT_NEAR(a, b, t) EXPECT_TRUE( Near(a, b, t) )

/**
 * @brief Assert all matrix elements are less than given value
 * @param input matrix, assertion will always fail if empty
 * @param value to compare against
 * @return assertion result
 */
template <typename T>
::testing::AssertionResult LT(const cv::Mat_<T>& m, const T &v)
{
    using namespace ::testing;
    if(m.empty()) {
         return AssertionFailure() << "Matrix operand is empty";
    }
    else {

        cv::Mat cmp_out = m >= v;
        int n = countNonZero(cmp_out);
        if(n == 0) { return AssertionSuccess(); }
        else {
            return AssertionFailure() << MatFailureMessageNonZero(m, cmp_out);
        }
    }
}

/**
 * @brief Per-element assertion that a(i) < b(i)
 * @param first matrix
 * @param second matrix
 * @return assertion result
 */
::testing::AssertionResult LT(const cv::Mat &a, const cv::Mat &b);
#define EXPECT_MAT_LT(m, v) EXPECT_TRUE( LT(m, v) )

/**
  Assert that matrix is empty
  */
::testing::AssertionResult Empty(const cv::Mat &mat);
// EXPECT_EMPTY macro already defined in ts/contrainer.h
// Should we use define guard?

#endif // _ELM_TS_MAT_ASSERTIONS_H_
