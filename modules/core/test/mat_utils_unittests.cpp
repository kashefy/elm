#include "core/mat_utils.h"

#include "gtest/gtest.h"
#include "ts/ts.h"
#include <string>

using namespace std;
using namespace sem;

/**
 * @brief test calculation of cumulative sum
 */
TEST(MatUtilsTest, CumSum_zeros) {

    int N = 10;
    for(int r=1; r<N; r++) {

        for(int c=1; c<N; c++) {

            MatF in = MatF::zeros(r, c);
            MatF out;
            sem::CumSum(in, out);
            EXPECT_MAT_EQ(in, out);
        }
    }
}

/**
 * @brief calculcate cumulative sum on matrix of ones.
 */
TEST(MatUtilsTest, CumSum_ones) {

    int N = 5;
    for(int r=1; r<N; r++) {

        for(int c=1; c<N; c++) {

            MatF in = MatF::ones(r, c);
            MatF out;
            sem::CumSum(in, out);

            // check output
            EXPECT_MAT_DIMS_EQ(in, out);
            for(int i=0; i<r*c; i++) {
                EXPECT_FLOAT_EQ( out(i), i+1 );
            }
            EXPECT_FLOAT_EQ( out(out.rows-1, out.cols-1), out.rows*out.cols ) << "Expectig sum value in last element.";
        }
    }
}

/**
 * @brief calculcate cumulative sum on matrix of negative ones.
 */
TEST(MatUtilsTest, CumSum_ones_neg) {

    int N = 5;
    for(int r=1; r<N; r++) {

        for(int c=1; c<N; c++) {

            MatF in = -MatF::ones(r, c);
            MatF out;
            sem::CumSum(in, out);

            // check output
            EXPECT_MAT_DIMS_EQ(in, out);
            for(int i=0; i<r*c; i++) {
                EXPECT_FLOAT_EQ( out(i), -(i+1) );
            }
            EXPECT_FLOAT_EQ( out(out.rows-1, out.cols-1), -out.rows*out.cols ) << "Expectig sum value in last element.";
        }
    }
}

/**
 * @brief calculcate cumulative sum on empty input matrix. Empty in, empty out.
 */
TEST(MatUtilsTest, CumSum_empty) {

    MatF in = MatF::zeros(0, 0);
    MatF out;
    sem::CumSum(in, out);
    EXPECT_EQ(out.total(), 0);
}

