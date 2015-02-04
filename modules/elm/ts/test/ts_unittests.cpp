/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/ts.h"

using namespace elm;

namespace {

class DummyTest : public ::testing::Test
{
};

TEST_F(DummyTest, FullTestName)
{
    EXPECT_EQ("DummyTest.FullTestName", std::string(FullTestName(test_info_)));
}

} // annonymous namespace around tests

