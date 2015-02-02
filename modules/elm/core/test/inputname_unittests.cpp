#include "elm/core/inputname.h"

#include "gtest/gtest.h"

using namespace std;
using namespace elm;

namespace {

TEST(InputNameTest, Constructor)
{
    EXPECT_EQ("foo", string(InputName("foo")));
    EXPECT_NE("bar", string(InputName("foo")));
    EXPECT_EQ("bar", string(InputName("bar")));
}

} // annonymoys namespsace for unit tests
