#include "sem/core/core.h"

#include "gtest/gtest.h"
#include <string>

using namespace std;
using namespace elm;

TEST(CoreTest, GetVersion) {

    EXPECT_GT(string(GetVersion()).size(), size_t(0));
}

