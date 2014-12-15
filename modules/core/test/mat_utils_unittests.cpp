#include "core/mat_utils.h"

#include "gtest/gtest.h"
#include "ts/ts.h"
#include <string>

using namespace std;
using namespace cv;
using namespace sem;

TEST(MatUtilsTest, ConvertTo8U_zeros)
{
    const Size2i SIZE(3, 3);
    Mat src = Mat1i::zeros(SIZE);
    EXPECT_MAT_EQ(Mat_<uchar>::zeros(SIZE), ConvertTo8U(src));
    src = Mat1f::zeros(SIZE);
    EXPECT_MAT_EQ(Mat_<uchar>::zeros(SIZE), ConvertTo8U(src));
}

/**
 * @brief Since we cannot distinguish min and max values
 * An input of ones behaves the same as input of zeros
 */
TEST(MatUtilsTest, ConvertTo8U_ones)
{
    const Size2i SIZE(3, 3);
    Mat src = Mat1i::ones(SIZE);
    EXPECT_MAT_EQ(Mat_<uchar>::zeros(SIZE), ConvertTo8U(src));
    src = Mat1f::ones(SIZE);
    EXPECT_MAT_EQ(Mat_<uchar>::zeros(SIZE), ConvertTo8U(src));
}

/**
 * @brief Test conversion+scaling to 8U (uchar) mats with unsigned integer input
 */
TEST(MatUtilsTest, ConvertTo8U_uint)
{
    Mat1i src(3, 3);
    src(0) = 0;
    src(1) = 255;
    for(size_t i=2; i<src.total(); i++) {

        // large numbers lead to overflow errors and are difficult to assert
        src(i) = randu<uint>() % 256;
    }

    Mat result = ConvertTo8U(src);

    EXPECT_MAT_DIMS_EQ(src, result) << "Dimensions changed";
    EXPECT_MAT_TYPE(result, CV_8U) << "Not unsigned chars";
    EXPECT_EQ(src.channels(), result.channels()) << "No. of channels changed";
}
/**
 * @brief Test conversion+scaling to 8U (uchar) mats with float matrix input
 */
TEST(MatUtilsTest, ConvertTo8U_float)
{
    Mat1f src(3, 3);

    // large numbers lead to overflow and are difficult to assert
    randn(src, 0., 255.);
    src.setTo(0.f, abs(src) > 255);

    int src_max_idx[2] = {-1, -1};
    double src_min_val, src_max_val;
    minMaxIdx(src, &src_min_val, 0, 0, src_max_idx);
    src_max_val = Mat1f(src-src_min_val)(src_max_idx[0], src_max_idx[1]);

    cv::Mat result = ConvertTo8U(src);

    EXPECT_MAT_DIMS_EQ(src, result) << "Dimensions changed";
    EXPECT_MAT_TYPE(result, CV_8U) << "Not unsigned chars";
    EXPECT_EQ(src.channels(), result.channels()) << "No. of channels changed";

    Mat1f result_f;
    result.convertTo(result_f, CV_32F, src_max_val/255., src_min_val);

    EXPECT_NEAR(sum(src)(0), sum(result_f)(0), src_max_val/10.); // allow for 10% error
}

/**
 * @brief test calculation of cumulative sum
 */
TEST(MatUtilsTest, CumSum_zeros) {

    int N = 10;
    for(int r=1; r<N; r++) {

        for(int c=1; c<N; c++) {

            Mat1f in = Mat1f::zeros(r, c);
            Mat1f out;
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

            Mat1f in = Mat1f::ones(r, c);
            Mat1f out;
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

            Mat1f in = -Mat1f::ones(r, c);
            Mat1f out;
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

    Mat1f in = Mat1f::zeros(0, 0);
    Mat1f out;
    sem::CumSum(in, out);
    EXPECT_EQ(out.total(), size_t(0));
}

/**
 * @brief test utility function for getting mat type string representation
 */
TEST(MatUtilsTest, TypeToString)
{
    Mat a;
    a = Mat(1, 1, CV_32FC1);
    EXPECT_GT(MatTypeToString(a).size(), size_t(0));
    EXPECT_EQ(MatTypeToString(a), "CV_32F");

    EXPECT_GT(MatTypeToString(a).size(), size_t(0));
    EXPECT_EQ(MatTypeToString(a), MatTypeToString(Mat(1, 1, CV_32FC2)));

    a = Mat(1, 1, CV_32SC1);
    EXPECT_GT(MatTypeToString(a).size(), size_t(0));
    EXPECT_EQ(MatTypeToString(a), MatTypeToString(Mat(1, 1, CV_32SC2)));
}

TEST(MatUtilsTest, TemplateTypeToString)
{
    Mat1f a(1, 1);
    EXPECT_FALSE(MatTypeToString(a).empty());
    EXPECT_EQ(MatTypeToString(a), "CV_32F");

    EXPECT_FALSE(MatTypeToString(a).empty());
    EXPECT_EQ(MatTypeToString(a), MatTypeToString(Mat(1, 1, CV_32FC2)));

    Mat1i b(1, 1);
    EXPECT_FALSE(MatTypeToString(b).empty());
    EXPECT_EQ(MatTypeToString(b), "CV_32S");
    EXPECT_EQ(MatTypeToString(b), MatTypeToString(Mat(1, 1, CV_32SC2)));
}

/**
 * @brief test utility function for getting mat type string representation
 * with same type but different number of channels
 */
TEST(MatUtilsTest, TypeToStringChannels)
{
    Size s(1, 1);
    for(int ch=1; ch<=4; ch++) {

        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_8U, ch))),  "CV_8U");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_8S, ch))),  "CV_8S");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_16U, ch))), "CV_16U");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_16S, ch))), "CV_16S");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_32S, ch))), "CV_32S");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_32F, ch))), "CV_32F");
        EXPECT_EQ(MatTypeToString(Mat(s, CV_MAKETYPE(CV_64F, ch))), "CV_64F");
    }
}

/**
 * @brief test ARange initialization with empty range
 */
TEST(MatARangeTest, Empty)
{
    EXPECT_TRUE(ARange_(0, 0, 0).empty());
    EXPECT_TRUE(ARange_(1, 3, 0).empty());
    EXPECT_TRUE(ARange_(10, 3, 0).empty());
    EXPECT_TRUE(ARange_(10, 30, 50).empty());
}

/**
 * @brief test invalid ARange initialization
 */
TEST(MatARangeTest, Invalid)
{
    EXPECT_THROW(ARange_(1, 3, -1), ExceptionValueError);
    EXPECT_THROW(ARange_(1, -3, 1), ExceptionValueError);
}

/**
 * @brief test Arange initialization with [x, y) range where y > x
 */
TEST(MatARangeTest, Increasing)
{
    { // integers
        Mat1i x = ARange_(1000, 1010, 1);

        EXPECT_EQ(x.rows, 1) << "Expecting row vector.";
        EXPECT_MAT_DIMS_EQ(x, Mat(1, 10, CV_32SC1));
        EXPECT_MAT_TYPE(x, CV_32S);

        for(int i=0; i<static_cast<int>(x.total()); i++) {

            EXPECT_EQ(x(i), 1000+i);
        }
    }

    { // uchar
        Mat x = ARange_<unsigned char>(0, 5, 2);

        EXPECT_EQ(x.rows, 1) << "Expecting row vector.";
        EXPECT_MAT_DIMS_EQ(x, Mat(1, 3, CV_32SC1));
        EXPECT_MAT_TYPE(x, CV_8U);

        for(int i=0, j=0; i<static_cast<int>(x.total()); i++, j+=2) {
            EXPECT_EQ(x.at<unsigned char>(i), static_cast<unsigned char>(j));
        }
    }

    { // floats
        Mat x = ARange_<float>(-1.f, 1.f, 0.5f);
        EXPECT_EQ(x.rows, 1) << "Expecting row vector.";
        EXPECT_MAT_DIMS_EQ(x, Size(4, 1));

        float j = -1.f;
        for(int i=0; i<static_cast<int>(x.total()); i++) {
            EXPECT_FLOAT_EQ(x.at<float>(i), j);
            j += 0.5f;
        }
    }
}

/**
 * @brief test Arange initialization with [x, y) range where y < x
 */
TEST(MatARangeTest, Decreasing)
{
    { // integers
        Mat1i x = ARange_(1010, 1000, -1);

        EXPECT_EQ(x.rows, 1) << "Expecting row vector.";
        EXPECT_MAT_DIMS_EQ(x, Mat(1, 10, CV_32SC1));
        EXPECT_MAT_TYPE(x, CV_32S);

        for(int i=0; i<static_cast<int>(x.total()); i++) {

            EXPECT_EQ(x(i), 1010-i);
        }
    }

    { // floats
        Mat x = ARange_<float>(1.f, -1.f, -0.5f);
        EXPECT_EQ(x.rows, 1) << "Expecting row vector.";
        EXPECT_MAT_DIMS_EQ(x, Size(4, 1));

        float j = 1.f;
        for(int i=0; i<static_cast<int>(x.total()); i++) {
            EXPECT_FLOAT_EQ(x.at<float>(i), j);
            j -= 0.5f;
        }
    }
}

/**
 * @brief test convinience function for converting a point struct to a matrix
 */
TEST(Point2MatTest, Point2Mat)
{
    const int N=10;
    for(int i=0; i<N; i++) {

        int x = randu<int>();
        int y = randu<int>();
        Mat m = Point2Mat(Point2i(x, y));
        EXPECT_MAT_DIMS_EQ(m, Size2i(2, 1)) << "not a row matrix with 2 columns";
        EXPECT_EQ(x, m.at<int>(0)) << "Unexpected value for x coordinate";
        EXPECT_EQ(y, m.at<int>(1)) << "Unexpected value for y coordinate";
    }
}

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


