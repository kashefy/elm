/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/interval.h"

#include "elm/core/interval.h"

using namespace testing;

AssertionResult InClosed(float x, float a, float b) {

    if(IntervalClosed(a, b).In(x)) { return AssertionSuccess(); }
    else { return AssertionFailure() << a << "<=" << x << "<=" << b << " Failed."; }
}

AssertionResult InLClosedROpen(float x, float a, float b) {

    if(IntervalLClosedROpen(a, b).In(x)) { return AssertionSuccess(); }
    else { return AssertionFailure() << a << "<=" << x << "<" << b << " Failed."; }
}
