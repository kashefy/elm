#include "elm/core/cv/neighborhood.h"

#include "elm/core/exception.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

/**
 * @brief test calculation of neighborhood mean and variance with empty input
 */
TEST(NeighMeanVarTest, Empty)
{
    Mat1f in, neigh_mean, neigh_var;
    ASSERT_NO_THROW(NeighMeanVar(in, 3, neigh_mean, neigh_var));
    EXPECT_MAT_DIMS_EQ(neigh_mean, Mat1f());
    EXPECT_MAT_DIMS_EQ(neigh_var, Mat1f());
}

/**
 * @brief test dimensions of neighborhood mean and variance matrices
 */
TEST(NeighMeanVarTest, Dims)
{
    const int N=6;
    for(int r=1; r<N; r++) {
        for(int c=1; c<N; c++) {

            Mat1f in(r, c), neigh_mean, neigh_var;
            ASSERT_NO_THROW(NeighMeanVar(in, 3, neigh_mean, neigh_var));
            EXPECT_MAT_DIMS_EQ(neigh_mean, Size2i(c, r));
            EXPECT_MAT_DIMS_EQ(neigh_var, Size2i(c, r));
        }
    }
}

/**
 * @brief test values of neighborhood mean and variance matrices with constant single-value input
 */
TEST(NeighMeanVarTest, ZeroVariance)
{
    Mat1f in, neigh_mean, neigh_var;
    ASSERT_NO_THROW(NeighMeanVar(Mat1f::zeros(10, 10), 3, neigh_mean, neigh_var));
    EXPECT_MAT_EQ(neigh_mean, Mat1f::zeros(10, 10)) << "All-zero input did not yield zero mean.";
    EXPECT_MAT_EQ(neigh_var, Mat1f::zeros(10, 10)) << "All-zero input did not yield zero variance.";

    const int N=6;
    for(float v=-10; v<=10; v+=5) {
        for(int r=1; r<N; r++) {
            for(int c=1; c<N; c++) {

                in = Mat1f(r ,c, v);
                ASSERT_NO_THROW(NeighMeanVar(in, 3, neigh_mean, neigh_var));
                EXPECT_MAT_EQ(neigh_mean, Mat1f(r ,c, v)) << "Incorrect mean for constant input";
                EXPECT_MAT_EQ(neigh_var, Mat1f::zeros(r, c)) << "Non-zero variance for constant input";
            }
        }
    }
}

/**
 * @brief test calculation neighborhood mean and variance matrices with different radii and zero variance input
 */
TEST(NeighMeanVarTest, ZeroVariance_Radius)
{
    Mat1f in, neigh_mean, neigh_var;
    for(int radius=1; radius<=10; radius++) {

        ASSERT_NO_THROW(NeighMeanVar(Mat1f::zeros(10, 10), radius, neigh_mean, neigh_var));
        EXPECT_MAT_EQ(neigh_mean, Mat1f::zeros(10, 10)) << "All-zero input did not yield zero mean.";
        EXPECT_MAT_EQ(neigh_var, Mat1f::zeros(10, 10)) << "All-zero input did not yield zero variance.";

        const int N=6;
        for(float v=-10; v<=10; v+=5) {
            for(int r=1; r<N; r++) {
                for(int c=1; c<N; c++) {

                    in = Mat1f(r ,c, v);
                    ASSERT_NO_THROW(NeighMeanVar(in, radius, neigh_mean, neigh_var));
                    EXPECT_MAT_EQ(neigh_mean, Mat1f(r ,c, v)) << "Incorrect mean for constant input";
                    EXPECT_MAT_EQ(neigh_var, Mat1f::zeros(r, c)) << "Non-zero variance for constant input";
                }
            }
        }
    }
}

/**
 * @brief test neighborhood calculations with radius 1
 */
TEST(NeighMeanVarTest, Radius_zero)
{
    const int N=5;
    for(int i=0; i<N; i++) {

        Mat1f in(5, 5, randu<float>()), neigh_mean, neigh_var;
        NeighMeanVar(in, 0, neigh_mean, neigh_var);
        EXPECT_MAT_NEAR(neigh_mean, in, 1e-7) << "All-zero input did not yield zero mean.";
        EXPECT_MAT_EQ(neigh_var, Mat1f::zeros(5, 5)) << "Did not yield zero variance with radius==1.";
    }
}

/**
 * @brief test neighborhood calculations with radius 1
 */
TEST(NeighMeanVarTest, Radius_negative)
{
    for(int radius=-1; radius<-10; radius--) {

        Mat1f in(5, 5, randu<float>()), neigh_mean, neigh_var;
        EXPECT_THROW(NeighMeanVar(in, radius, neigh_mean, neigh_var), ExceptionValueError);
    }
}

/**
 * @brief test calculation neighborhood mean and variance matrices
 */
TEST(NeighMeanVarTest, Values)
{
    const int N=9;
    const int RADIUS=1;
    const int DISTANCE=RADIUS*2+1;
    Mat1f in(N, N, 0.f);
    for(int r=0; r<N; r++) {
        for(int c=0; c<N; c++) {

            if(r % DISTANCE == 0 && c % DISTANCE == 0) {

                in(r, c) = 1.f;
            }
        }
    }

    Mat1f neigh_mean, neigh_var;
    NeighMeanVar(in, RADIUS, neigh_mean, neigh_var);

    // check mean values
    float sub_area = static_cast<float>(DISTANCE*DISTANCE);

    // at top-left corner
    EXPECT_FLOAT_EQ(4.f/sub_area, neigh_mean(0, 0)) << "Unexpected mean value for mean at TL corner";

    // top row, excluding top-left and top-right corners
    EXPECT_MAT_EQ(neigh_mean.row(0).colRange(1, neigh_mean.cols-1),
                  Mat1f(1, neigh_mean.cols-2, 2.f/sub_area)) << "Unexpected mean values in top-row";

    // left border row, excluding TL and BL corners
    EXPECT_MAT_EQ(neigh_mean.col(0).rowRange(1, neigh_mean.cols-1),
                  Mat1f(neigh_mean.cols-2, 1, 2.f/sub_area)) << "Unexpected mean values in most-left column";

    // most-right border
    EXPECT_MAT_EQ(neigh_mean.col(neigh_mean.cols-1),
                  Mat1f::zeros(neigh_mean.cols, 1)) << "Unexpected mean values in most-right column";

    // bottom border
    EXPECT_MAT_EQ(neigh_mean.row(neigh_mean.rows-1),
                  Mat1f::zeros(1, neigh_mean.rows)) << "Unexpected mean values in bottom row";

    // remaining area
    EXPECT_MAT_EQ(neigh_mean(Range(1, neigh_mean.rows-1), Range(1, neigh_mean.cols-1)),
                  Mat1f(neigh_mean.rows-2, neigh_mean.cols-2, 1.f/sub_area))
            << "Unexpected mean  values in the central area.";


    // check variance values
    // at top-left corner
    EXPECT_FLOAT_EQ(0.24691357f, neigh_var(0, 0)) << "Unexpected variance value at TL corner";

    // top row, excluding top-left and top-right corners
    EXPECT_MAT_EQ(neigh_var.row(0).colRange(1, neigh_var.cols-1),
                  Mat1f(1, neigh_var.cols-2, 0.17283951f)) << "Unexpected variance values in top-row";

    // left border row, excluding TL and BL corners
    EXPECT_MAT_EQ(neigh_var.col(0).rowRange(1, neigh_var.cols-1),
                  Mat1f(neigh_var.cols-2, 1, 0.17283951f)) << "Unexpected variance values in most-left column";

    // most-right border
    EXPECT_MAT_EQ(neigh_var.col(neigh_var.cols-1),
                  Mat1f::zeros(neigh_var.cols, 1)) << "Unexpected variance values in most-right column";

    // bottom border
    EXPECT_MAT_EQ(neigh_var.row(neigh_var.rows-1),
                  Mat1f::zeros(1, neigh_var.rows)) << "Unexpected variance values in bottom row";

    // remaining area
    EXPECT_MAT_EQ(neigh_var(Range(1, neigh_var.rows-1), Range(1, neigh_var.cols-1)),
                  Mat1f(neigh_var.rows-2, neigh_var.cols-2, 0.09876544f))
            << "Unexpected variance values in the central area.";
}

/**
 * @brief test calculation neighborhood mean and variance matrices with zero-padding
 */
TEST(NeighMeanVarTest, ZeroPadding)
{
    const int N=9;
    const int RADIUS=1;
    const int DISTANCE=RADIUS*2+1;
    Mat1f in(N, N, 0.f);
    for(int r=0; r<N; r++) {
        for(int c=0; c<N; c++) {

            if(r % DISTANCE == 0 && c % DISTANCE == 0) {

                in(r, c) = 1.f;
            }
        }
    }

    Mat1f neigh_mean, neigh_var;
    NeighMeanVar(in, RADIUS, neigh_mean, neigh_var, cv::BORDER_CONSTANT, Scalar_<float>(0.f));

    // check mean values
    float sub_area = static_cast<float>(DISTANCE*DISTANCE);

    // most-right border
    EXPECT_MAT_EQ(neigh_mean.col(neigh_mean.cols-1),
                  Mat1f::zeros(neigh_mean.cols, 1)) << "Unexpected mean values in most-right column";

    // bottom border
    EXPECT_MAT_EQ(neigh_mean.row(neigh_mean.rows-1),
                  Mat1f::zeros(1, neigh_mean.rows)) << "Unexpected mean values in bottom row";

    // remaining area
    EXPECT_MAT_EQ(neigh_mean(Range(0, neigh_mean.rows-1), Range(0, neigh_mean.cols-1)),
                  Mat1f(neigh_mean.rows-1, neigh_mean.cols-1, 1.f/sub_area))
            << "Unexpected mean  values in the central area.";


    // check variance values
    // most-right border
    EXPECT_MAT_EQ(neigh_var.col(neigh_var.cols-1),
                  Mat1f::zeros(neigh_var.cols, 1)) << "Unexpected variance values in most-right column";

    // bottom border
    EXPECT_MAT_EQ(neigh_var.row(neigh_var.rows-1),
                  Mat1f::zeros(1, neigh_var.rows)) << "Unexpected variance values in bottom row";

    // remaining area
    EXPECT_MAT_EQ(neigh_var(Range(0, neigh_var.rows-1), Range(0, neigh_var.cols-1)),
                  Mat1f(neigh_var.rows-1, neigh_var.cols-1, 0.09876544f))
            << "Unexpected variance values in the central area.";
}

} // annonymous namespace for test cases
