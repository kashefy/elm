#include "elm/core/inputname.h"

#include "gtest/gtest.h"

using namespace std;
using namespace elm;

namespace {

TEST(InputNameTest, Constructor_from_string)
{
    EXPECT_EQ("foo", string(InputName(string("foo"))));
    EXPECT_NE("bar", string(InputName(string("foo"))));
    EXPECT_EQ("bar", string(InputName(string("bar"))));
}

TEST(InputNameTest, Constructor_from_const_char)
{
    EXPECT_EQ("foo", string(InputName("foo")));
    EXPECT_NE("bar", string(InputName("foo")));
    EXPECT_EQ("bar", string(InputName("bar")));
}

TEST(InputNameTest, AssignToString_operator)
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

TEST(InputNameTest, AssignToString_explicit_method)
{
    string s;
    {
        s = InputName("foo").to_string();
    }
    EXPECT_EQ("foo", s);

    {
        InputName in("bar");
        s = in.to_string();
    }
    EXPECT_EQ("bar", s);
}

TEST(InputNameTest, Operator_equals)
{
    InputName x("foo");
    InputName y("foo");

    EXPECT_TRUE(x==y);
    EXPECT_EQ(x, y);

    // switch rhs and lhs
    EXPECT_TRUE(y==x);
    EXPECT_EQ(y, x);
}

TEST(InputNameTest, Operator_non_member_equals)
{
    string lhs("foo");
    InputName rhs("foo");

    EXPECT_TRUE(lhs==rhs);
}

TEST(InputNameTest, Operator_not_equals)
{
    InputName x("foo");
    InputName y("bar");

    EXPECT_TRUE(x!=y);
    EXPECT_NE(x, y);

    // switch rhs and lhs
    EXPECT_TRUE(y!=x);
    EXPECT_NE(y, x);
}

TEST(InputNameTest, Operator_non_membernot__equals)
{
    {
        string lhs("foo");
        InputName rhs("bar");

        EXPECT_FALSE(lhs==rhs);
    }
    {
        string lhs("bar");
        InputName rhs("foo");

        EXPECT_FALSE(lhs==rhs);
    }
}

TEST(InputNameTest, Equals)
{
    InputName x("foo");
    InputName y("bar");
    InputName z("foo");

    EXPECT_TRUE(x==z);
    EXPECT_FALSE(x==y);
    EXPECT_FALSE(y==z);
    EXPECT_TRUE(x!=y);
    EXPECT_TRUE(y!=z);

    EXPECT_EQ(x, z);
    EXPECT_NE(x, y);
    EXPECT_NE(y, z);
}

} // annonymoys namespsace for unit tests
