#include "encoding/base_filterbank.h"

#include "ts/ts.h"

#include <iostream>

using namespace std;

TEST(IterTest, test)
{
    base_FilterBank range;

    cout << "NumberRange::iterator:" << endl;
    for (base_FilterBank::iterator iter = range.begin(); iter != range.end(); iter++)
        cout << *iter << endl;

    cout << "NumberRange::cumulative_iterator:" << endl;
    for (base_FilterBank::cumulative_iterator iter = range.begin(); iter != range.end(); iter++)
        cout << *iter << endl;
}
