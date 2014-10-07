#ifndef SEM_MODULES_TS_H_
#define SEM_MODULES_TS_H_

#include "gtest/gtest.h"

#include <string>
#include <opencv2/core.hpp>

::testing::AssertionResult EqualDims(const cv::Mat& a, const cv::Mat& b);
#define EXPECT_MAT_DIMS_EQ(a, b) EXPECT_TRUE( EqualDims(a, b) )

std::string MatFailureMessageNonZero(const cv::Mat& a, const cv::Mat& b, const cv::Mat& cmp);

::testing::AssertionResult Equal(const cv::Mat& a, const cv::Mat& b);
#define EXPECT_MAT_EQ(a, b) EXPECT_TRUE( Equal(a, b) )

::testing::AssertionResult Near(const cv::Mat& a, const cv::Mat& b, float tolerance);
#define EXPECT_MAT_NEAR(a, b, t) EXPECT_TRUE( Near(a, b, t) )

#endif // SEM_MODULES_TS_H_
