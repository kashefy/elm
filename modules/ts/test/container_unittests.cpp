#include "ts/container.h"

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

    EXPECT_EQ( 1, i.size() );
    EXPECT_FALSE( Empty(i) );

    EXPECT_EQ( 2, s.size() );
    EXPECT_FALSE( Empty(s) );

    i.pop_back();
    EXPECT_TRUE( Empty(i) );
    EXPECT_EMPTY( i );

    s.clear();
    EXPECT_TRUE( Empty(s) );
    EXPECT_EMPTY( s );
}

} // anonymous namespace
