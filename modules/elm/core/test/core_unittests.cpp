/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/core.h"

#include "gtest/gtest.h"
#include <string>

using namespace std;
using namespace elm;

TEST(CoreTest, GetVersion) {

    EXPECT_GT(string(GetVersion()).size(), size_t(0));
}

