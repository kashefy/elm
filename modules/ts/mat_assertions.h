/**@file OpenCV Mat Assertions
  */
#ifndef SEM_TS_MAT_ASSERTIONS_H_
#define SEM_TS_MAT_ASSERTIONS_H_

#include "gtest/gtest.h"

#include <string>
#include <opencv2/core.hpp>

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

::testing::AssertionResult Equal(const cv::Mat& a, const cv::Mat& b);
#define EXPECT_MAT_EQ(a, b) EXPECT_TRUE( Equal(a, b) )

::testing::AssertionResult Near(const cv::Mat& a, const cv::Mat& b, float tolerance);
#define EXPECT_MAT_NEAR(a, b, t) EXPECT_TRUE( Near(a, b, t) )

/**
 * @brief Assert all matrix elements are less than given value
 * @param input matrix
 * @param value to compare against
 * @return assertion result
 */
template <typename T>
::testing::AssertionResult LT(const cv::Mat_<T>& m, const T &v)
{
    cv::Mat cmp_out = m >= v;
    int n = countNonZero(cmp_out);
    if(n == 0) { return ::testing::AssertionSuccess(); }
    else {
        return ::testing::AssertionFailure() << MatFailureMessageNonZero(m, cmp_out);
    }
}
#define EXPECT_MAT_LT(m, v) EXPECT_TRUE( LT(m, v) )

/**
  Assert that matrix is empty
  */
::testing::AssertionResult Empty(const cv::Mat &mat);
// EXPECT_EMPTY macro already defined in ts/contrainer.h
// Should we use define guard?

#endif // SEM_TS_MAT_ASSERTIONS_H_
