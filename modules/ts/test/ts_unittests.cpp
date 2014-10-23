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

    // test macro
    EXPECT_IN_CLOSED(0, -1, 1);
    EXPECT_IN_CLOSED(0, 0, 0);
    EXPECT_IN_CLOSED(0, -1, 0);
    EXPECT_IN_CLOSED(0, 0, 1);
}

TEST(LClosedROpenIntervalAssertionTest, TestAssertion) {

    EXPECT_TRUE( InLClosedROpen(0, -1, 1) );
    EXPECT_FALSE( InLClosedROpen(0, 0, 0) );
    EXPECT_FALSE( InLClosedROpen(0, -1, 0) );
    EXPECT_TRUE( InLClosedROpen(0, 0, 1) );
    EXPECT_FALSE( InLClosedROpen(0, 1, -1) );
    EXPECT_FALSE( InLClosedROpen(0, 1, 3) );
    EXPECT_FALSE( InLClosedROpen(0, 0, -1) );

    // test macro
    EXPECT_IN_LCLOSED_ROPEN(0, -1, 1);
    EXPECT_IN_LCLOSED_ROPEN(0, 0, 1);
}

TEST(IntervalAssertionTest, ClosedVsOpen)
{
    EXPECT_FALSE( InClosed(0, 1, 3) || InLClosedROpen(0, 1, 3) );
    EXPECT_TRUE ( InClosed(1, 1, 3) || InLClosedROpen(1, 1, 3) );
    EXPECT_TRUE ( InClosed(3, 1, 3) && !InLClosedROpen(3, 1, 3) );
    EXPECT_FALSE( InClosed(4, 1, 3) || InLClosedROpen(4, 1, 3) );
}

class DummyTest : public ::testing::Test
{
};

TEST_F(DummyTest, FullTestName)
{
    EXPECT_EQ("DummyTest.FullTestName", std::string(FullTestName(test_info_)));
}

} // namespace

