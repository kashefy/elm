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

TEST(InputNameTest, AssignToString)
{
    string s;
    {
        s = InputName("foo");
    }
    EXPECT_EQ("foo", s);

    {
        InputName in("bar");
        s = in;
    }
    EXPECT_EQ("bar", s);
}

} // annonymoys namespsace for unit tests
