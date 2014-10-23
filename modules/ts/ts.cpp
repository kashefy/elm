#include "ts/ts.h"

#include "core/interval.h"
#include "core/mat_utils.h"

using namespace std;
using namespace testing;

AssertionResult InClosed(float x, float a, float b) {

    if(IntervalClosed(a, b).In(x)) { return AssertionSuccess(); }
    else { return AssertionFailure() << a << "<=" << x << "<=" << b << " Failed."; }
}

AssertionResult InLClosedROpen(float x, float a, float b) {

    if(IntervalLClosedROpen(a, b).In(x)) { return AssertionSuccess(); }
    else { return AssertionFailure() << a << "<=" << x << "<" << b << " Failed."; }
}

const char* FullTestName(const TestInfo *test_info)
{
    string s1(test_info->test_case_name());
    string s2(test_info->name());
    return (s1+"."+s2).c_str();
}
