#ifndef SEM_MODULES_TS_H_
#define SEM_MODULES_TS_H_

#include "gtest/gtest.h"

#include <string>
#include "ts/mat_assertions.h"

::testing::AssertionResult InClosed(float x, float a, float b);
#define EXPECT_IN_CLOSED(x, a, b) EXPECT_TRUE( InClosed(x, a, b) )

const char* FullTestName(const ::testing::TestInfo *test_info);

#endif // SEM_MODULES_TS_H_
