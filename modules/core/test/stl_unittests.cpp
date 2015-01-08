#include "core/stl.h"

#include "ts/ts.h"

#include <string>

using namespace std;
using namespace sem;

TEST(STLFindTest, Find)
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

TEST(STLKeysTest, Empty)
{
    EXPECT_SIZE(0, Keys(map<string, string>()));

    map<string, int> m;
    m["k1"] = 100;
    m.clear();
    EXPECT_SIZE(0, Keys(m));
    EXPECT_TRUE(Keys(m).empty());
}

TEST(STLKeysTest, Size)
{
    map<int, float> m;
    for(int i=1; i<10; i++) {

        m[i] = (i)*10;
        EXPECT_SIZE(i, Keys(m));
    }
}

TEST(STLKeysTest, Keys)
{
    map<int, float> m;
    for(int i=1; i<10; i++) {

        m[i] = (i)*10;
        vector<int> v = Keys(m);
        EXPECT_SIZE(i, v);

        sort(v.begin(), v.end());
        for(int k=0, j=1; j<=i; j++, k++) {
            EXPECT_EQ(j, v[k]);
        }
    }
}



