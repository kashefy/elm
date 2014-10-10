#include "ts/ts.h"

using namespace cv;

namespace {

TEST(ClosedIntervalAssertionTest, TestAssertion) {

    EXPECT_TRUE( InClosed(0, -1, 1) );
    EXPECT_TRUE( InClosed(0, 0, 0) );
    EXPECT_TRUE( InClosed(0, -1, 0) );
    EXPECT_TRUE( InClosed(0, 0, 1) );
    EXPECT_FALSE( InClosed(0, 1, -1) );
    EXPECT_FALSE( InClosed(0, 1, 3) );
    EXPECT_FALSE( InClosed(0, 0, -1) );
}

} // namespace

