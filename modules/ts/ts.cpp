#include "ts/ts.h"

#include "core/interval.h"
#include "core/mat_utils.h"

using namespace std;
using namespace testing;

AssertionResult InClosed(float x, float a, float b) {

    if(IntervalClosed(a, b).In(x)) { return AssertionSuccess(); }
    else { return AssertionFailure() << a << "<=" << x << "<=" << b << " Failed."; }
}
