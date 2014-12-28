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

TEST(ElementsAtLoc, EmptyVector)
{
    EXPECT_NO_THROW(ElementsAt(VecMat1f(), 0, 0));
    EXPECT_EMPTY(ElementsAt(VecMat1f(), 0, 0));

    EXPECT_NO_THROW(ElementsAt(VecMat1f(), 999, 999));
    EXPECT_EMPTY(ElementsAt(VecMat1f(), 999, 999));

    EXPECT_NO_THROW(ElementsAt(VecMat1f(), 999, 0));
    EXPECT_EMPTY(ElementsAt(VecMat1f(), 999, 0));

    EXPECT_NO_THROW(ElementsAt(VecMat1f(), 0, 999));
    EXPECT_EMPTY(ElementsAt(VecMat1f(), 0, 999));
}

TEST(ElementsAt, ElementsAt)
{
    const int SIZE=10;
    const int ROWS=3;
    const int COLS=4;
    VecMat1f data;
    for(int i=0; i<SIZE; i++) {

        Mat1f tmp(ROWS, COLS);
        randn(tmp, 0., 1.);
        data.push_back(tmp);
    }

    for(int r=0; r<ROWS; r++) {
        for(int c=0; c<COLS; c++) {

            Mat1f elements_expected(1, static_cast<int>(data.size()));
            int k=0;
            for(VecMat1fCIter itr=data.begin();
                itr != data.end();
                itr++, k++) {

                elements_expected(k) = (*itr)(r, c);
            }
            Mat1f elements_actual = sem::ElementsAt(data, r, c);

            EXPECT_MAT_DIMS_EQ(elements_actual, Size(static_cast<int>(data.size()), 1)) << "Expecting a row matrix";
            EXPECT_MAT_EQ(elements_expected, elements_actual) << "Unexpected element values";
        }
    }
}

TEST(ElementsAt, Invalid)
{
    const int SIZE=2;
    const int ROWS=3;
    const int COLS=4;
    VecMat1f data;
    for(int i=0; i<SIZE; i++) {

        Mat1f tmp(ROWS, COLS);
        randn(tmp, 0., 1.);
        data.push_back(tmp);
    }

    EXPECT_NO_THROW(ElementsAt(data, 0, 0));
    EXPECT_THROW(ElementsAt(data, -1, 0), ExceptionBadDims);
    EXPECT_THROW(ElementsAt(data, 0, -1), ExceptionBadDims);
    EXPECT_THROW(ElementsAt(data, -1, 0), ExceptionBadDims);
    EXPECT_THROW(ElementsAt(data, -1, -1), ExceptionBadDims);
    EXPECT_THROW(ElementsAt(data, ROWS, 0), ExceptionBadDims);
    EXPECT_THROW(ElementsAt(data, 0, COLS), ExceptionBadDims);
}

/**
 * @brief test reshape routine with empty input
 */
TEST(ReshapeVecMatTest, Empty)
{
    // empty vector
    EXPECT_EMPTY(Reshape(VecMat1f())) << "Empty input yielded non-empty output. That's odd.";

    // vector filled with empty matrices
    for(int i=1; i<10; i++) {

        EXPECT_EMPTY(Reshape(VecMat1f(i, Mat1f())));
    }
}

/**
 * @brief test dimensions of reshaped input
 */
TEST(ReshapeVecMatTest, Dims)
{
    for(int i=1; i<=10; i++) {

        for(int r=1; r<7; r++) {

            for(int c=1; c<7; c++) {

                EXPECT_MAT_DIMS_EQ(Reshape(VecMat1f(i, Mat1f::zeros(r, c))), Size(i, r*c))
                        << "Unexpected dims returend from reshappign vector of matrices";
            }
        }
    }
}

/**
 * @brief test values after reshape
 */
TEST(ReshapeVecMatTest, Values)
{
    const int N=10; ///< no. of matrices in vector
    const int ROWS=3;
    const int COLS=4;
    VecMat1f in;
    for(int i=0; i<N; i++) {

        Mat1f m(ROWS, COLS);
        randn(m, 0, 100);
        in.push_back(m);
    }

    Mat1f result = Reshape(in);

    // inspect result
    for(int i=0; i<N; i++) {

        for(int r=0; r<ROWS; r++) {

            for(int c=0; c<COLS; c++) {

                EXPECT_FLOAT_EQ(in[i](r, c), result(r*COLS+c, i))
                        << "Value mismatch between initial input and reshaped output";
            }
        }
    }
}

TEST(Mat_ToVec_Test, Empty)
{
    EXPECT_EMPTY(Mat_ToVec_<float>(Mat1f()));
    EXPECT_EMPTY(Mat_ToVec_<float>(Mat1i()));
    EXPECT_EMPTY(Mat_ToVec_<float>(Mat1b()));
}

/**
 * @brief test conversion to vector with two-dimensional matrix as input
 */
TEST(Mat_ToVec_Test, TwoDimensional_Landscape)
{
    const int ROWS=3;
    const int COLS=4;
    Mat1f in(ROWS, COLS);
    randn(in, 0, 100);
    VecF out = Mat_ToVec_(in);
    EXPECT_SIZE(in.total(), out) << "Not all elements acccounted for";

    // check values
    for(int r=0; r<ROWS; r++) {

        for(int c=0; c<COLS; c++) {

            EXPECT_FLOAT_EQ(in(r, c), out[r*COLS+c]) << "value mismatch";
        }
    }
}

TEST(Mat_ToVec_Test, TwoDimensional_Portrait)
{
    const int ROWS=4;
    const int COLS=3;
    Mat1f in(ROWS, COLS);
    randn(in, 0, 100);
    VecF out = Mat_ToVec_(in);
    EXPECT_SIZE(in.total(), out) << "Not all elements acccounted for";

    // check values
    for(int r=0; r<ROWS; r++) {

        for(int c=0; c<COLS; c++) {

            EXPECT_FLOAT_EQ(in(r, c), out[r*COLS+c]) << "value mismatch";
        }
    }
}

/**
 * @brief test conversion with one-dimensional matrix
 */
TEST(Mat_ToVec_Test, OneDimensional)
{
    const int SIZE=5;
    Mat1i in(SIZE, 1);
    randn(in, 0, 100);

    ASSERT_GT(SIZE, 0) << "This test needs zero-sized input";

    std::vector<int> out = Mat_ToVec_<int>(in);
    EXPECT_SIZE(SIZE, out) << "Not all elements acccounted for";
    EXPECT_SIZE(SIZE, Mat_ToVec_<int>(in.t())) << "Not all elements acccounted for";

    // check values
    for(int i=0; i<SIZE; i++) {

        EXPECT_EQ(in(i), out[i]) << "Value mismatch at i=" << i;
    }
}


/**
 * @brief A setup for repeating tests with different types of mat objects (int, float, uchar)
 */
template <class T>
class MatPODTypesTest : public testing::Test
{
};

/**
 * @brief the struct below enables defining values to be used inside the tests
 * These values are set below once per type.
 */
template<class T>
struct V_
{
    static std::vector<T> values;
};

TYPED_TEST_CASE_P(MatPODTypesTest);

/**
 * @brief pass invalid input and check exception thrown
 * Multi-channels matrices not supported.
 */
TYPED_TEST_P(MatPODTypesTest, Invalid_MultiChannel) {

    std::vector<TypeParam> v = V_<TypeParam>::values;

    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >::zeros(1, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >::ones(1, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >::zeros(10, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >::ones(10, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >(10, 1, randu<TypeParam>()), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >(1, 10, randu<TypeParam>()), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >(1, 10, v[0]), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >(3, 4, v[0]), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >(4, 3, v[0]), v[0]), ExceptionBadDims );

    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >::zeros(1, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >::ones(1, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >::zeros(10, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >::ones(10, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >(10, 1, randu<TypeParam>()), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >(1, 10, randu<TypeParam>()), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >(1, 10, v[0]), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >(3, 4, v[0]), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >(4, 3, v[0]), v[0]), ExceptionBadDims );

    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >::zeros(1, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >::ones(1, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >::zeros(10, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >::ones(10, 1), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >(10, 1, randu<TypeParam>()), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >(1, 10, randu<TypeParam>()), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >(1, 10, v[0]), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >(3, 4, v[0]), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >(4, 3, v[0]), v[0]), ExceptionBadDims );
}

/**
 * @brief pass invalid empty input and check exception thrown
 * Multi-channels matrices not supported.
 */
TYPED_TEST_P(MatPODTypesTest, Invalid_MultiChannelEmpty) {

    std::vector<TypeParam> v = V_<TypeParam>::values;
    typedef Mat_<TypeParam> MatTP;

    // effectively single-channel
    EXPECT_NO_THROW( find_first_of(Mat_<Vec<TypeParam, 1> >(), v[0]) );

    EXPECT_NO_THROW( find_first_of(Mat_<Vec<TypeParam, 1> >::zeros(0, 0), v[0]) );
    EXPECT_NO_THROW( find_first_of(Mat_<Vec<TypeParam, 1> >::zeros(1, 0), v[0]) );
    EXPECT_NO_THROW( find_first_of(Mat_<Vec<TypeParam, 1> >::zeros(0, 1), v[0]) );

    EXPECT_NO_THROW( find_first_of(Mat_<Vec<TypeParam, 1> >::ones(0, 0), v[0]) );
    EXPECT_NO_THROW( find_first_of(Mat_<Vec<TypeParam, 1> >::ones(1, 0), v[0]) );
    EXPECT_NO_THROW( find_first_of(Mat_<Vec<TypeParam, 1> >::ones(0, 1), v[0]) );

    // 2 channels
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >(), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >::zeros(0, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >::zeros(1, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >::zeros(0, 1), v[0]), ExceptionBadDims );

    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >::ones(0, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >::ones(1, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 2> >::ones(0, 1), v[0]), ExceptionBadDims );

    // 3 channels
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >(), v[0]), ExceptionBadDims );

    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >::zeros(0, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >::zeros(1, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >::zeros(0, 1), v[0]), ExceptionBadDims );

    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >::ones(0, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >::ones(1, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 3> >::ones(0, 1), v[0]), ExceptionBadDims );

    // 4 channels
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >(), v[0]), ExceptionBadDims );

    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >::zeros(0, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >::zeros(1, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >::zeros(0, 1), v[0]), ExceptionBadDims );

    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >::ones(0, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >::ones(1, 0), v[0]), ExceptionBadDims );
    EXPECT_THROW( find_first_of(Mat_<Vec<TypeParam, 4> >::ones(0, 1), v[0]), ExceptionBadDims );
}

/**
 * @brief pass invalid input and check exception thrown
 * Only continuous matrices supported for now.
 */
TYPED_TEST_P(MatPODTypesTest, Invalid_NonContinuous) {

    std::vector<TypeParam> v = V_<TypeParam>::values;
    typedef Mat_<TypeParam> MatTP;

    // copy vector of values into matrix object
    MatTP v_mat(1, static_cast<int>(v.size()));
    for(size_t i=0; i<v.size(); i++) v_mat(i) = v[i];
    v_mat = v_mat.reshape(1, 2);

    /* provoke non-continuous input by taking out column ranges
     */
    // col() yielding single columns
    for(int c=0; c<v_mat.cols; c++) {

        EXPECT_THROW( find_first_of(v_mat.col(c), v[0]), ExceptionTypeError );
        EXPECT_THROW( find_first_of(v_mat.col(c), v[0]), ExceptionTypeError );
    }

    // colRange()
    for(int c=1; c<v_mat.cols-1; c++) {

        EXPECT_THROW( find_first_of(v_mat.colRange(c, v_mat.cols), v[0]), ExceptionTypeError );
    }

    for(int c=1; c<v_mat.cols; c++) {

        EXPECT_THROW( find_first_of(v_mat.colRange(0, c), v[0]), ExceptionTypeError );
    }

    // Scenarios where we don't expect failures
    for(size_t i=0; i<v.size(); i++) {

        // colRange() that spans all cols should pass as continuous
        EXPECT_NO_THROW( find_first_of(v_mat.colRange(0, v_mat.cols), v[i]) ) << "Spanning all columns enables matrix continuity.";
        int index;
        EXPECT_TRUE( find_first_of(v_mat.colRange(0, v_mat.cols), v[i], index) );
        EXPECT_EQ(static_cast<int>(i), index);

        /* Now we'll take out row ranges, all should pass as continuous
         */
        // row() yielding single rows
        for(int r=0; r<v_mat.rows; r++) {

            EXPECT_NO_THROW( find_first_of(v_mat.row(r), v[i]) );
            EXPECT_NO_THROW( find_first_of(v_mat.row(r), v[i]) );
        }

        // rowRange()
        for(int r=1; r<v_mat.rows; r++) {

            EXPECT_NO_THROW( find_first_of(v_mat.rowRange(0, r), v[i]) );
        }

        for(int r=0; r<v_mat.rows-1; r++) {

            EXPECT_NO_THROW( find_first_of(v_mat.rowRange(r, v_mat.rows), v[i]) );
        }
    }
}


/**
 * @brief test finding elements in empty matrices
 */
TYPED_TEST_P(MatPODTypesTest, FindFirstOf_Empty) {

    std::vector<TypeParam> v = V_<TypeParam>::values;
    typedef Mat_<TypeParam> MatTP;
    for(size_t i=0; i<v.size(); i++) {

        EXPECT_FALSE( find_first_of(MatTP::zeros(1, 0), v[i]) );
        EXPECT_FALSE( find_first_of(MatTP::zeros(0, 1), v[i]) );
        EXPECT_FALSE( find_first_of(MatTP(), v[i]) );
    }
}

/**
 * @brief We don't expect to find anything.
 * This test assumes that values[4] is within range and its value is unique.
 * Matrix is reshaped and transposed.
 */
TYPED_TEST_P(MatPODTypesTest, FindFirstOf_NotFound) {

    std::vector<TypeParam> v = V_<TypeParam>::values;
    typedef Mat_<TypeParam> MatTP;

    MatTP v_mat(1, static_cast<int>(v.size()));
    for(size_t i=0; i<v.size(); i++) v_mat(i) = v[i];

    TypeParam target = v[4];
    v_mat(4) = v[0];
    for(size_t i=0; i<v.size(); i++) {

        EXPECT_FALSE( find_first_of(v_mat.reshape(1, 1), target) );
        EXPECT_FALSE( find_first_of(v_mat.reshape(1, 2), target) );
        EXPECT_FALSE( find_first_of(v_mat.t(), target) );
    }
}

/**
 * @brief We expect to find something.
 * Matrix is reshaped and transposed.
 */
TYPED_TEST_P(MatPODTypesTest, FindFirstOf_Found) {

    std::vector<TypeParam> v = V_<TypeParam>::values;
    typedef Mat_<TypeParam> MatTP;

    MatTP v_mat(1, static_cast<int>(v.size()));
    for(size_t i=0; i<v.size(); i++) v_mat(i) = v[i];

    int index;
    for(size_t i=0; i<v.size(); i++) {

        EXPECT_TRUE( find_first_of(v_mat.reshape(1, 1), v[i]) );
        EXPECT_TRUE( find_first_of(v_mat.reshape(1, 2), v[i]) );
        EXPECT_TRUE( find_first_of(v_mat.t(), v[i]) );

        index = -1;
        EXPECT_TRUE( find_first_of(v_mat.reshape(1, 1), v[i], index) );
        EXPECT_EQ(static_cast<int>(i), index);

        index = -1;
        EXPECT_TRUE( find_first_of(v_mat.reshape(1, 2), v[i], index) );
        EXPECT_EQ(static_cast<int>(i), index);

        index = -1;
        EXPECT_TRUE( find_first_of(v_mat.t(), v[i], index) );
        EXPECT_EQ(static_cast<int>(i), index);
    }
}

/**
 * @brief One value is duplicated and placed into previous index
 * Matrix is reshaped and transposed.
 */
TYPED_TEST_P(MatPODTypesTest, FindFirstOf_Duplicate) {

    std::vector<TypeParam> v = V_<TypeParam>::values;
    typedef Mat_<TypeParam> MatTP;

    MatTP v_mat(1, static_cast<int>(v.size()));
    for(size_t i=0; i<v.size(); i++) v_mat(i) = v[i];

    int index_actual = -2;
    for(size_t i=1; i<v.size(); i++) {

        // make a mat copy of the vector
        MatTP v_mat(1, static_cast<int>(v.size()));
        for(size_t i=0; i<v.size(); i++) v_mat(i) = v[i];

        int index_expected = abs(randu<int>() % static_cast<int>(i));
        v_mat(index_expected) = v[i]; // duplicate somewhere before

        index_actual = -1;
        EXPECT_TRUE( find_first_of(v_mat.reshape(1, 1), v[i], index_actual) );
        EXPECT_EQ(index_expected, index_actual);

        index_actual = -1;
        EXPECT_TRUE( find_first_of(v_mat.reshape(1, 2), v[i], index_actual) );
        EXPECT_EQ(index_expected, index_actual);

        index_actual = -1;
        EXPECT_TRUE( find_first_of(v_mat.t(), v[i], index_actual) );
        EXPECT_EQ(index_expected, index_actual);
    }

    // sanity check that we actually iterated throug the loop
    ASSERT_GE(index_actual, 0) << "Test didn't really do anything.";
}

/* Write additional type+value parameterized tests here.
 * Acquaint yourself with the values passed to along with each type.
 */

// Register test names
REGISTER_TYPED_TEST_CASE_P(MatPODTypesTest,
                           Invalid_NonContinuous,
                           Invalid_MultiChannel,
                           Invalid_MultiChannelEmpty,
                           FindFirstOf_Empty,
                           FindFirstOf_NotFound,
                           FindFirstOf_Found,
                           FindFirstOf_Duplicate); ///< register additional typed_test_p (i.e. unit test) routines here

///< Register values to work with inside tests, note how they're used inside the tests
template<> std::vector<float> V_<float>::values{-1.f, 0.f, 1.f, 100.f, 101.f, 200.f};
template<> std::vector<int> V_<int>::values{-1, 0, 1, 100, 101, 200};
template<> std::vector<uchar> V_<uchar>::values{255, 0, 1, 100, 101, 200};

typedef testing::Types<float, int, uchar> PODTypes;  ///< // lists the usual suspects of matrices
INSTANTIATE_TYPED_TEST_CASE_P(MatUtilsPODTypesTest, MatPODTypesTest, PODTypes);
