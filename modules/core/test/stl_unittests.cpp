#include "core/stl.h"

#include "gtest/gtest.h"

#include <string>

using namespace std;
using namespace sem;

TEST(STLTest, Find)
{
    map<string, string> m;
    string s;
    EXPECT_FALSE(Find(m, "k", s));
    m["k1"] = "n1";
    EXPECT_TRUE(Find(m, "k1", s));
    EXPECT_EQ("n1", s);
    EXPECT_FALSE(Find(m, "k", s));
    EXPECT_EQ("n1", s) << "function modified object in spite of key not being found.";
}
