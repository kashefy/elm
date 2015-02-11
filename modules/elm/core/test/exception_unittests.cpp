/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/exception.h"

#include <string>

#include "gtest/gtest.h"
#include <opencv2/core.hpp>

using namespace elm;

namespace {

TEST(ExceptionTest, Throw)
{
    EXPECT_THROW(ELM_THROW_BAD_DIMS("bad dims"), ExceptionBadDims);
    EXPECT_THROW(ELM_THROW_NOT_IMPLEMENTED, ExceptionNotImpl);
    EXPECT_THROW(ELM_THROW_FILEIO_ERROR("Could not open foo.bar"), ExceptionFileIOError);
    EXPECT_THROW(ELM_THROW_VALUE_ERROR("Bad value"), ExceptionValueError);
    EXPECT_THROW(ELM_THROW_KEY_ERROR("Bad key"), ExceptionKeyError);
    EXPECT_THROW(ELM_THROW_TYPE_ERROR("Bad type"), ExceptionTypeError);
}

TEST(ExceptionTest, NotImplementedMsg)
{
    try {
        ELM_THROW_NOT_IMPLEMENTED;

    } catch(const ExceptionNotImpl &e) {

        std::string msg(e.what());
        EXPECT_NE(msg.find("Not implemented yet"), msg.npos);
    }
}

TEST(ExceptionTest, ThrowOnTrue)
{
    EXPECT_THROW(ELM_THROW_BAD_DIMS_IF(true, "bad dims"), ExceptionBadDims);
    EXPECT_NO_THROW(ELM_THROW_BAD_DIMS_IF(false, "bad dims"));
}

} // annonymous namespace for unit tests
