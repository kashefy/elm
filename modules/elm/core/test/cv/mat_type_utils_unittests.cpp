/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/cv/mat_type_utils.h"

#include "gtest/gtest.h"

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;
using namespace elm;

namespace {

/**
 * @brief test utility function for getting mat type string representation
 */
TEST(CV_TypeUtilsTest, TypeToString)
{
    Mat a;
    a = Mat(1, 1, CV_32FC1);
    EXPECT_GT(MatTypeToString(a).size(), size_t(0));
    EXPECT_EQ(MatTypeToString(a), "CV_32F");

    EXPECT_GT(MatTypeToString(a).size(), size_t(0));
    EXPECT_EQ(MatTypeToString(a), MatTypeToString(Mat(1, 1, CV_32FC2)));

    a = Mat(1, 1, CV_32SC1);
    EXPECT_GT(MatTypeToString(a).size(), size_t(0));
    EXPECT_EQ(MatTypeToString(a), MatTypeToString(Mat(1, 1, CV_32SC2)));
}

TEST(CV_TypeUtilsTest, TemplateTypeToString)
{
    Mat1f a(1, 1);
    EXPECT_FALSE(MatTypeToString(a).empty());
    EXPECT_EQ(MatTypeToString(a), "CV_32F");

    EXPECT_FALSE(MatTypeToString(a).empty());
    EXPECT_EQ(MatTypeToString(a), MatTypeToString(Mat(1, 1, CV_32FC2)));

    Mat1i b(1, 1);
    EXPECT_FALSE(MatTypeToString(b).empty());
    EXPECT_EQ(MatTypeToString(b), "CV_32S");
    EXPECT_EQ(MatTypeToString(b), MatTypeToString(Mat(1, 1, CV_32SC2)));
}

/**
 * @brief test utility function for getting mat type string representation
 * with same type but different number of channels
 */
TEST(CV_TypeUtilsTest, TypeToStringChannels)
{
    Size s(1, 1);
    for(int ch=1; ch<=4; ch++) {

        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_8U, ch))),  "CV_8U");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_8S, ch))),  "CV_8S");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_16U, ch))), "CV_16U");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_16S, ch))), "CV_16S");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_32S, ch))), "CV_32S");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_32F, ch))), "CV_32F");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_64F, ch))), "CV_64F");
    }
}

TEST(CV_TypeUtilsTest, UserType)
{
    Mat_<string> m(2, 3);
    EXPECT_EQ(MatTypeToString(m), "CV_USRTYPE1");
}

} // annnonymous namespace for unit tests
