#include "core/stl/stl.h"

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

/**
 * @brief Typed tests around Vector routines with POD types
 */
template <class T>
class STL_VectorUtils : public ::testing::Test
{
protected:
};
typedef ::testing::Types<float, int, uchar> PODTypes;
TYPED_TEST_CASE(STL_VectorUtils, PODTypes);

/**
 * @brief test utility function for populating a vector with uniformly random values
 */
TYPED_TEST(STL_VectorUtils, Push_back_randu_size)
{
    for(int n=0; n<11; n++) {

        vector<TypeParam > v;
        push_back_randu(v, n);
        EXPECT_SIZE(n, v);
    }
}



