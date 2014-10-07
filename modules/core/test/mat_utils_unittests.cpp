#include "core/mat_utils.h"

#include "gtest/gtest.h"
#include "ts/ts.h"
#include <string>

using namespace std;
using namespace sem;

TEST(MatUtilsTest, CumSum_zeros) {

    int N = 10;
    for(int r=0; r<N; r++) {

        for(int c=0; c<N; c++) {

            MatF in = MatF::zeros(r, c);
            MatF out;
            sem::CumSum(in, out);
            EXPECT_EQ(in.total(), out.total());
        }
    }
}

