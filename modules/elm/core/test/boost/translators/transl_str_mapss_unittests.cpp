/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/boost/translators/transl_str_mapss.h"

#include "gtest/gtest.h"

#include "elm/core/typedefs_sfwd.h"
#include "elm/ts/container.h"

using namespace elm;

namespace {

TEST(transl_MapSSTest, Add_to_ptree)
{
    PTree pt;
    {
        pt.add("x", 1);

        MapSS m;
        m["foo"] = "bar";
        m["x"] = "y";

        pt.add("m", m);
    }

    MapSS m = pt.get<MapSS >("m");

    EXPECT_SIZE(2, m);
    EXPECT_EQ("bar", m["foo"]);
    EXPECT_EQ("y", m["x"]);
    EXPECT_EQ(m.end(), m.find("y"));
}

TEST(transl_MapSSTest, PTree_owns_copy)
{
    PTree pt;
    {
        MapSS m;
        m["foo"] = "bar";
        m["x"] = "y";

        pt.add("m", m);
    }

    MapSS m1 = pt.get<MapSS >("m");
    MapSS m2 = pt.get<MapSS >("m");
\
    for(MapSS::const_iterator itr=m2.begin(); itr != m2.end(); ++itr) {

        m2[itr->first] += itr->second;
    }

    EXPECT_NE(m2["foo"], m1["foo"]);
    EXPECT_NE(m2["x"], m1["x"]);

    EXPECT_EQ("bar", m1["foo"]);
    EXPECT_EQ("y", m1["x"]);

    EXPECT_EQ("barbar", m2["foo"]);
    EXPECT_EQ("yy", m2["x"]);
}

TEST(transl_MapSSTest, Update_value)
{
    PTree pt;
    {
        MapSS m;
        m["foo"] = "bar";
        m["x"] = "y";

        pt.put("m", m);
    }
    {
        MapSS m;
        m["foo"] = "xbar";
        m["x"] = "xy";

        pt.put("m", m);
    }

    MapSS m = pt.get<MapSS >("m");

    EXPECT_SIZE(2, m);
    EXPECT_EQ("xbar", m["foo"]);
    EXPECT_EQ("xy", m["x"]);
    EXPECT_EQ(m.end(), m.find("y"));
}

TEST(transl_MapSSTest, Add_two)
{
    PTree pt;
    {
        MapSS m;
        m["foo"] = "bar";
        m["x"] = "y";

        pt.add("m1", m);
    }
    {
        MapSS m;
        m["foo"] = "xbar";
        m["x"] = "xy";

        pt.add("m2", m);
    }

    MapSS m1 = pt.get<MapSS >("m1");

    EXPECT_SIZE(2, m1);
    EXPECT_EQ("bar", m1["foo"]);
    EXPECT_EQ("y", m1["x"]);
    EXPECT_EQ(m1.end(), m1.find("y"));

    MapSS m2 = pt.get<MapSS >("m2");

    EXPECT_SIZE(2, m2);
    EXPECT_EQ("xbar", m2["foo"]);
    EXPECT_EQ("xy", m2["x"]);
    EXPECT_EQ(m2.end(), m2.find("y"));
}

} // annonymous namespace for tests

