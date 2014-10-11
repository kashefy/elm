#include "core/exception.h"

#include "gtest/gtest.h"
#include <string>
#include <opencv2/core.hpp>

using namespace std;

TEST(ExceptionTest, ThrowBadDims)
{
    EXPECT_THROW(SEM_THROW_BAD_DIMS("bad dims"), ExceptionBadDims);
}

TEST(ExceptionTest, ThrowNotImplemented)
{
    EXPECT_THROW(SEM_THROW_NOT_IMPLEMENTED, ExceptionNotImpl);
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

