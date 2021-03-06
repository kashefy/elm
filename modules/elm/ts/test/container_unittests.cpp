/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/container.h"

#include <string>

using namespace std;

namespace {

TEST(Container, Empty) {

    std::vector<int> i;
    std::vector<std::string> s;
    EXPECT_TRUE( Empty(i) );
    EXPECT_TRUE( Empty(s) );

    // test macro
    EXPECT_EMPTY( i );
    EXPECT_EMPTY( s );

    i.push_back(123);
    s.push_back("foo");
    s.push_back("bar");

    EXPECT_EQ( size_t(1), i.size() );
    EXPECT_FALSE( Empty(i) );

    EXPECT_EQ( size_t(2), s.size() );
    EXPECT_FALSE( Empty(s) );

    i.pop_back();
    EXPECT_TRUE( Empty(i) );
    EXPECT_EMPTY( i );

    s.clear();
    EXPECT_TRUE( Empty(s) );
    EXPECT_EMPTY( s );
}

TEST(Container, Size) {

    EXPECT_SIZE(4, vector<int>(4, 1));
    EXPECT_SIZE(5, vector<string>(5, "foo"));
    EXPECT_SIZE(0, vector<string>());
}

} // anonymous namespace
