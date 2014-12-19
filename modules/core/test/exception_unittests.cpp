#include "core/exception.h"

#include "gtest/gtest.h"
#include <string>
#include <opencv2/core.hpp>

using namespace std;
using namespace sem;

TEST(ExceptionTest, Throw)
{
    EXPECT_THROW(SEM_THROW_BAD_DIMS("bad dims"), ExceptionBadDims);
    EXPECT_THROW(SEM_THROW_NOT_IMPLEMENTED, ExceptionNotImpl);
    EXPECT_THROW(SEM_THROW_FILEIO_ERROR("Could not open foo.bar"), ExceptionFileIOError);
    EXPECT_THROW(SEM_THROW_VALUE_ERROR("Bad value"), ExceptionValueError);
    EXPECT_THROW(SEM_THROW_KEY_ERROR("Bad key"), ExceptionKeyError);
    EXPECT_THROW(SEM_THROW_TYPE_ERROR("Bad type"), ExceptionTypeError);
}

TEST(ExceptionTest, NotImplementedMsg)
{
    try {
        SEM_THROW_NOT_IMPLEMENTED;

    } catch(const ExceptionNotImpl &e) {

        std::string msg(e.what());
        EXPECT_NE(msg.find("Not implemented yet"), msg.npos);
    }
}

