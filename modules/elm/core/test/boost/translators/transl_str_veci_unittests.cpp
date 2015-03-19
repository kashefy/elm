/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/boost/translators/transl_str_veci.h"

#include "gtest/gtest.h"

#include "elm/core/typedefs_sfwd.h"
#include "elm/ts/container.h"

using namespace elm;

namespace {

TEST(transl_VecITest, Add_to_ptree)
{
    PTree pt;
    {
        pt.add("x", 1);

        VecI v;
        v.push_back(3);
        v.push_back(1);
        v.push_back(2);

        pt.add("v", v);
    }

    VecI v = pt.get<VecI >("v");

    EXPECT_SIZE(3, v);
    EXPECT_EQ(3, v[0]);
    EXPECT_EQ(1, v[1]);
    EXPECT_EQ(2, v[2]);
}

TEST(transl_VecITest, PTree_owns_copy)
{
    PTree pt;
    {
        VecI v;
        v.push_back(3);
        v.push_back(1);
        v.push_back(2);

        pt.add("v", v);
    }

    VecI v1 = pt.get<VecI >("v");
    VecI v2 = pt.get<VecI >("v");

    for (size_t i=0; i<v2.size(); i++) {

        v2[i] *= 10;
    }

    EXPECT_NE(v2[0], v1[0]);
    EXPECT_NE(v2[0], v1[1]);
    EXPECT_NE(v2[0], v1[2]);

    EXPECT_EQ(3, v1[0]);
    EXPECT_EQ(1, v1[1]);
    EXPECT_EQ(2, v1[2]);

    EXPECT_EQ(30, v2[0]);
    EXPECT_EQ(10, v2[1]);
    EXPECT_EQ(20, v2[2]);
}

TEST(transl_VecITest, Update_value)
{
    PTree pt;
    {
        VecI v;
        v.push_back(3);
        v.push_back(1);
        v.push_back(2);

        pt.add("v", v);
    }
    {
        VecI v;
        v.push_back(-3);
        v.push_back(-1);
        v.push_back(-2);

        pt.put("v", v);
    }

    VecI v = pt.get<VecI >("v");

    EXPECT_SIZE(3, v);
    EXPECT_EQ(-3, v[0]);
    EXPECT_EQ(-1, v[1]);
    EXPECT_EQ(-2, v[2]);
}

TEST(transl_VecITest, Add_two)
{
    PTree pt;
    {
        VecI v;
        v.push_back(3);
        v.push_back(1);
        v.push_back(2);

        pt.put("v1", v);
    }
    {
        VecI v;
        v.push_back(-3);
        v.push_back(-1);
        v.push_back(-2);

        pt.put("v2", v);
    }

    VecI v1 = pt.get<VecI >("v1");

    EXPECT_EQ(3, v1[0]);
    EXPECT_EQ(1, v1[1]);
    EXPECT_EQ(2, v1[2]);

    VecI v2 = pt.get<VecI >("v2");

    EXPECT_EQ(-3, v2[0]);
    EXPECT_EQ(-1, v2[1]);
    EXPECT_EQ(-2, v2[2]);
}

} // annonymous namespace for tests
