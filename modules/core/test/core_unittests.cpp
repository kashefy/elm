#include "core/core.h"

#include "gtest/gtest.h"
#include <string>

using namespace std;
using namespace sem;

TEST(CoreTest, GetVersion) {

    EXPECT_GT(string(GetVersion()).size(), 0);
}

