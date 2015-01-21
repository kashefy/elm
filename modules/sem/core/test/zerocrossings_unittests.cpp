#include "sem/core/zerocrossings.h"

#include "sem/core/exception.h"
#include "sem/ts/ts.h"

using namespace cv;

/**
 * @brief class for testing computing of zero-crossings usig sobel algo.
 */
class ZeroCrossingsDiffTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = ZeroCrossingsDiff();
    }

    virtual void DrawLinesVertical(Mat1f &test_src, Mat1f &expected_dst)
    {
        test_src = Mat1f(9, 9, 0.1f);

        // define a vertical line with +ve values
        Mat line = Mat1f::ones(test_src.rows, 1);

        // define a vertical line with -ve values
        Mat line_neg = -line;

        // apply +ve line to matrix, will not register as zero-crossing
        line.copyTo(test_src.col(test_src.cols/2));

        // apply -ve vert. line in two places
        // should register 4 vertical zero crossings
        line_neg.copyTo(test_src.col(1*test_src.cols/4));
        line_neg.copyTo(test_src.col(3*test_src.cols/4));

        // draw expected matrix with zero crossings
        expected_dst = Mat1f::zeros(test_src.size());
        Mat1f expected_row = Mat1f::zeros(1, test_src.cols);
        expected_row.colRange(test_src.cols/4, test_src.cols/4+2) = 1.f;
        expected_row.colRange(3*test_src.cols/4, 3*test_src.cols/4+2) = 1.f;
        repeat(expected_row, test_src.rows, 1, expected_dst);
    }

    virtual void DrawLinesHorizontal(Mat1f &test_src, Mat1f &expected_dst)
    {
        DrawLinesVertical(test_src, expected_dst);

        flip(test_src.t(), test_src, 1);
        flip(expected_dst.t(), expected_dst, 1);
        expected_dst.col(expected_dst.cols-2).copyTo(expected_dst.col(expected_dst.cols-1));

        //std::cout<<test_src<<std::endl;
        //std::cout<<expected_dst<<std::endl;
    }

    ZeroCrossingsDiff to_; ///< test object
};

TEST_F(ZeroCrossingsDiffTest, Empty)
{
    Mat1f src, dst;
    EXPECT_NO_THROW(to_(src, dst));
    EXPECT_TRUE(dst.empty());
}

TEST_F(ZeroCrossingsDiffTest, Unfiorm)
{
    for(int r=1; r<10; r++) {

        for(int c=1; c<10; c++) {

            for(float v=-1.f; v<=1.f; v++) {

                Mat1f src(r, c, v), dst;
                to_(src, dst);
                EXPECT_MAT_EQ(dst, Mat1f::zeros(src.size())) << "No zero-crossing for constant image";
            }
        }
    }
}

TEST_F(ZeroCrossingsDiffTest, PosLinePosBackground)
{
    Mat1f src(9, 9, 0.1f), dst;

    // define a vertical line with +ve values
    Mat line = Mat1f::ones(src.rows, 1);
    // apply +ve line to matrix, will not register as zero-crossing
    line.copyTo(src.col(src.cols/2));

    to_(src, dst);

    EXPECT_MAT_EQ(dst, Mat1f::zeros(src.size()));
}

TEST_F(ZeroCrossingsDiffTest, LinesVertical)
{
    Mat1f src, dst, expected_dst;
    DrawLinesVertical(src, expected_dst);

    to_(src, dst);

    EXPECT_FALSE(dst.empty());
    EXPECT_MAT_DIMS_EQ(src, dst);
    EXPECT_MAT_EQ(dst, expected_dst);
}

TEST_F(ZeroCrossingsDiffTest, LinesHorizontal)
{
    Mat1f src, dst, expected_dst;
    DrawLinesHorizontal(src, expected_dst);

    to_(src, dst);

    EXPECT_FALSE(dst.empty());
    EXPECT_MAT_DIMS_EQ(src, dst);
    EXPECT_MAT_EQ(dst, expected_dst);
}

/**
 * @brief Test with row matrix and column matrix input
 */
TEST_F(ZeroCrossingsDiffTest, RowMatColMat)
{
    // Row matrix
    Mat1f src(1, 9, 0.1f), dst;
    src(1*src.cols/4) = -1.f;
    src(3*src.cols/4) = -1.f;

    to_(src, dst);

    EXPECT_FALSE(dst.empty());
    EXPECT_MAT_DIMS_EQ(src, dst);

    Mat1f expected_dst(src.size(), 0.f);
    expected_dst.colRange(src.cols/4, src.cols/4+2) = 1.f;
    expected_dst.colRange(3*src.cols/4, 3*src.cols/4+2) = 1.f;
    EXPECT_MAT_EQ(dst, expected_dst);

    // Column matrix
    to_(src.t(), dst);
    EXPECT_MAT_EQ(dst, expected_dst.t());
}

TEST_F(ZeroCrossingsDiffTest, SingleElement)
{
    Mat1f dst;
    EXPECT_NO_THROW(to_(Mat1f(1, 1, 0.5f), dst));
    EXPECT_MAT_EQ(dst, Mat1f::zeros(1, 1));
}
