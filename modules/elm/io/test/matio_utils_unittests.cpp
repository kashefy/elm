/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matio_utils.h"

#include "gtest/gtest.h"

#include <opencv2/core/core.hpp>

#ifdef __WITH_MATIO

#include "matio.h"

#include "elm/core/exception.h"

using namespace elm;

namespace {

TEST(MATIO_UTILS_TEST, MATIOClassTOCV_TYPE)
{
    EXPECT_EQ(CV_64F, MATIOClassTOCV_TYPE(MAT_C_DOUBLE));
    EXPECT_EQ(CV_16U, MATIOClassTOCV_TYPE(MAT_C_DOUBLE));
}

TEST(MATIO_UTILS_TEST, MATIOClassTOCV_TYPE_invalid)
{
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_CELL), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_FUNCTION), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_EMPTY), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_OBJECT), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_STRUCT), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_SPARSE), ExceptionTypeError);
}

} // annonymous namespace for unit tests

#endif // __WITH_MATIO
