#include "sem/ts/interval.h"

#include "sem/core/interval.h"

using namespace testing;

AssertionResult InClosed(float x, float a, float b) {

    if(IntervalClosed(a, b).In(x)) { return AssertionSuccess(); }
    else { return AssertionFailure() << a << "<=" << x << "<=" << b << " Failed."; }
}

AssertionResult InLClosedROpen(float x, float a, float b) {

    if(IntervalLClosedROpen(a, b).In(x)) { return AssertionSuccess(); }
    else { return AssertionFailure() << a << "<=" << x << "<" << b << " Failed."; }
}
