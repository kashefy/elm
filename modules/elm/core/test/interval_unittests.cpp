#include "elm/core/interval.h"

#include "gtest/gtest.h"

TEST(ClosedIntervalTest, In) {

    EXPECT_TRUE( IntervalClosed(-1, 1).In(0) );
    EXPECT_TRUE( IntervalClosed(0, 0).In(0) );
    EXPECT_TRUE( IntervalClosed(-1, 0).In(0) );
    EXPECT_TRUE( IntervalClosed(0, 1).In(0) );
    EXPECT_FALSE( IntervalClosed(1, -1).In(0) );
    EXPECT_FALSE( IntervalClosed(1, 3).In(0) );
    EXPECT_FALSE( IntervalClosed(0, -1).In(0) );
}

TEST(LClosedROpenIntervalTest, In) {

    EXPECT_TRUE( IntervalLClosedROpen(-1, 1).In(0) );
    EXPECT_FALSE( IntervalLClosedROpen(0, 0).In(0) );
    EXPECT_FALSE( IntervalLClosedROpen(-1, 0).In(0) );
    EXPECT_TRUE( IntervalLClosedROpen(0, 1).In(0) );
    EXPECT_FALSE( IntervalLClosedROpen(1, -1).In(0) );
    EXPECT_FALSE( IntervalLClosedROpen(1, 3).In(0) );
    EXPECT_FALSE( IntervalLClosedROpen(0, -1).In(0) );
}

TEST(IntervalTest, ClosedVsOpen)
{
    float a=1, b=3;
    IntervalClosed cl(a, b);
    IntervalLClosedROpen lcl_rop(a, b);
    EXPECT_FALSE( cl.In(0) || lcl_rop.In(0) );
    EXPECT_TRUE ( cl.In(1) && lcl_rop.In(1) );
    EXPECT_FALSE( cl.In(3) && lcl_rop.In(3) );
    EXPECT_FALSE( cl.In(4) && lcl_rop.In(4) );
}
