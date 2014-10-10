#include "core/interval.h"

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

