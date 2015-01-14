#include "ts/ts.h"

using namespace std;
using namespace testing;

const char* sem::FullTestName(const TestInfo *test_info)
{
    string s1(test_info->test_case_name());
    string s2(test_info->name());
    return (s1+"."+s2).c_str();
}
