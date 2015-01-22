/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/ts.h"

using namespace std;
using namespace testing;

const char* elm::FullTestName(const TestInfo *test_info)
{
    string s1(test_info->test_case_name());
    string s2(test_info->name());
    return (s1+"."+s2).c_str();
}
