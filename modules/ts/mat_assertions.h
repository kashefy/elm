#ifndef SEM_TS_MAT_ASSERTIONS_H_
#define SEM_TS_MAT_ASSERTIONS_H_

#include "gtest/gtest.h"

#include <string>
#include <opencv2/core.hpp>

// OpenCV Mat Assertions

::testing::AssertionResult IsType(const cv::Mat& a, int mat_type);
#define EXPECT_MAT_TYPE(a, mat_type) EXPECT_TRUE( IsType(a, mat_type) )

std::string MatTypeFailureMessage(const cv::Mat& a, int mat_type);

::testing::AssertionResult EqualDims(const cv::Mat& a, const cv::Mat& b);
::testing::AssertionResult EqualDims(const cv::Mat& a, const cv::Size2i& s);
#define EXPECT_MAT_DIMS_EQ(a, b) EXPECT_TRUE( EqualDims(a, b) )

std::string MatFailureMessageNonZero(const cv::Mat& a, const cv::Mat& b, const cv::Mat& cmp);

::testing::AssertionResult Equal(const cv::Mat& a, const cv::Mat& b);
#define EXPECT_MAT_EQ(a, b) EXPECT_TRUE( Equal(a, b) )

::testing::AssertionResult Near(const cv::Mat& a, const cv::Mat& b, float tolerance);
#define EXPECT_MAT_NEAR(a, b, t) EXPECT_TRUE( Near(a, b, t) )

#endif // SEM_TS_MAT_ASSERTIONS_H_
