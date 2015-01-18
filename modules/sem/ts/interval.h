/**@file interval assertions
  */
#ifndef SEM_TS_INTERVAL_H_
#define SEM_TS_INTERVAL_H_

#include "gtest/gtest.h"

/**
  assert that value x lies in closed interval [a, b]
  */
::testing::AssertionResult InClosed(float x, float a, float b);
#define EXPECT_IN_CLOSED(x, a, b) EXPECT_TRUE( InClosed(x, a, b) )

/**
  assert that value x lies in left-closed right-open interval [a, b)
  */
::testing::AssertionResult InLClosedROpen(float x, float a, float b);
#define EXPECT_IN_LCLOSED_ROPEN(x, a, b) EXPECT_TRUE( InLClosedROpen(x, a, b) )

#endif // SEM_TS_INTERVAL_H_
