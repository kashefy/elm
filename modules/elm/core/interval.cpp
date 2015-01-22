/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/interval.h"

base_Interval::base_Interval(float a, float b)
    : a_(a), b_(b)
{
}

IntervalClosed::IntervalClosed(float a, float b)
    :base_Interval(a, b)
{
}

bool IntervalClosed::In(float x) const
{
    return x >= a_ && x <= b_;
}

IntervalLClosedROpen::IntervalLClosedROpen(float a, float b)
    : base_Interval(a, b)
{
}

bool IntervalLClosedROpen::In(float x) const
{
    return x >= a_ && x < b_;
}
