/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/stl/stl_inl.h"

#include "elm/ts/ts.h"

#include <string>

using namespace std;
using namespace elm;

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

TYPED_TEST(STL_VectorUtils, VecToString_length)
{
    vector<TypeParam > v;
    v.push_back(static_cast<TypeParam>(1));
    v.push_back(static_cast<TypeParam>(0));
    v.push_back(static_cast<TypeParam>(40));

    EXPECT_GT(to_string(v).length(), size_t(0)) << "String is empty.";
    EXPECT_NE(to_string(v)[to_string(v).length()-1], ',') << "String ends with delimeter";
}

TYPED_TEST(STL_VectorUtils, VecToString_empty)
{
    vector<TypeParam > v;
    EXPECT_EQ(size_t(0), to_string(v).length()) << "String is not empty.";
    EXPECT_EQ(size_t(0), to_string(v, ",").length()) << "String is not empty.";
    EXPECT_EQ(size_t(0), to_string(v, "; ").length()) << "String is not empty.";
}

TYPED_TEST(STL_VectorUtils, VecToString_delim)
{
    vector<TypeParam > v;
    v.push_back(static_cast<TypeParam>(1));
    v.push_back(static_cast<TypeParam>(0));
    v.push_back(static_cast<TypeParam>(40));

    vector<string> delims;
    delims.push_back(", ");
    delims.push_back(",");
    delims.push_back("_");
    delims.push_back(";");

    for(size_t i=0; i<delims.size(); i++) {

        string vs = to_string(v, delims[i]);
        EXPECT_GT(vs.length(), size_t(0)) << "String is empty.";
        EXPECT_NE(vs[vs.length()-1], ',') << "String ends with delimeter";

        if(i>0) {
            // delimeters without spaces
            EXPECT_EQ(vs.npos, vs.find(' ')) << "contains spaces.";
        }
        else if(i==0) {

            EXPECT_EQ(to_string(v), vs) << "delim[" << i << "] is not the default delimeter.";
        }

    }

}

TEST(STL_VectorUtilsSpecialized, VecToString_length_int)
{
    vector<int> v;
    v.push_back(1);
    v.push_back(0);
    v.push_back(40);

    EXPECT_EQ(string("1, 0, 40"), to_string(v));
}

TEST(STL_VectorUtilsSpecialized, VecToString_length_float)
{
    EXPECT_EQ(string("1"), to_string(vector<float>(1, 1.f)));
    EXPECT_EQ(string("0"), to_string(vector<float>(1, 0.f)));
    EXPECT_EQ(string("-3.1"), to_string(vector<float>(1, -3.1f)).substr(0, 4));
}

