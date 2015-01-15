#include "core/cv/mat_utils.h"
#include "core/cv/mat_utils_inl.h"

#include "ts/ts.h"

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

TEST(Mat2PointTest, Mat2Point2i)
{
    const int N=10;
    for(int i=0; i<N; i++) {

        int x = randu<int>();
        int y = randu<int>();
        Mat1i m = Mat1i(1, 2);
        m(0) = x;
        m(1) = y;

        // twice to transpose the second time around

        for(int j=0; j<2; j++) {

            Point2i p = Mat2Point2i(m);
            EXPECT_EQ(x, p.x) << "Unexpected value for x coordinate";
            EXPECT_EQ(y, p.y) << "Unexpected value for y coordinate";

            m = m.t();
        }
    }
}

TEST(Mat2PointTest, Mat2Point3i)
{
    const int N=10;
    for(int i=0; i<N; i++) {

        int x = randu<int>();
        int y = randu<int>();
        int z = randu<int>();

        Mat1i m = Mat1i(1, 3);
        m(0) = x;
        m(1) = y;
        m(2) = z;

        /* Iterate twice to transpose the second time around.
         * This way we test row as well as column Mat input.
         */
        for(int j=0; j<2; j++) {

            Point3i p = Mat2Point3_(m);
            EXPECT_EQ(x, p.x) << "Unexpected value for x coordinate";
            EXPECT_EQ(y, p.y) << "Unexpected value for y coordinate";
            EXPECT_EQ(z, p.z) << "Unexpected value for z coordinate";

            m = m.t();
        }
    }
}

TEST(Mat2PointTest, Mat2Point3f)
{
    const int N=10;
    for(int i=0; i<N; i++) {

        Mat1f m = Mat1f(1, 3);
        randu(m, 0.f, 100.f);

        /* Iterate twice to transpose the second time around.
         * This way we test row as well as column Mat input.
         */
        for(int j=0; j<2; j++) {

            Point3f p = Mat2Point3_(m);
            EXPECT_EQ(m(0), p.x) << "Unexpected value for x coordinate";
            EXPECT_EQ(m(1), p.y) << "Unexpected value for y coordinate";
            EXPECT_EQ(m(2), p.z) << "Unexpected value for z coordinate";

            m = m.t();
        }
    }
}

TEST(Mat2PointTest, Mat2Point2i_redundnantElems)
{
    const int N=10;
    for(int i=0; i<N; i++) {

        Mat1i m = Mat1i(1, 10);
        randu(m, 0, 100);
        int x = m(0);
        int y = m(1);

        // twice to transpose the second time around
        Point2i p = Mat2Point2i(m);
        EXPECT_EQ(x, p.x) << "Unexpected value for x coordinate";
        EXPECT_EQ(y, p.y) << "Unexpected value for y coordinate";
    }
}

TEST(Mat2PointTest, Mat2Point3i_redundnantElems)
{
    const int N=10;
    for(int i=0; i<N; i++) {

        Mat1i m = Mat1i(1, 10);
        randu(m, 0, 100);

        // twice to transpose the second time around
        Point3i p = Mat2Point3_(m);
        EXPECT_EQ(m(0), p.x) << "Unexpected value for x coordinate";
        EXPECT_EQ(m(1), p.y) << "Unexpected value for y coordinate";
        EXPECT_EQ(m(2), p.z) << "Unexpected value for z coordinate";
    }
}

TEST(Mat2PointTest, Mat2Point3f_redundnantElems)
{
    const int N=10;
    for(int i=0; i<N; i++) {

        Mat1f m = Mat1f(1, 10);
        randu(m, 0.f, 100.f);

        // twice to transpose the second time around
        Point3f p = Mat2Point3_(m);
        EXPECT_EQ(m(0), p.x) << "Unexpected value for x coordinate";
        EXPECT_EQ(m(1), p.y) << "Unexpected value for y coordinate";
        EXPECT_EQ(m(2), p.z) << "Unexpected value for z coordinate";
    }
}

TEST(Mat2PointTest, Invalid)
{
    // sanity check on what's returned from Mat::total()
    ASSERT_EQ(size_t(1), Mat1i(1, 1).total());
    ASSERT_EQ(size_t(2), Mat1i(1, 2).total());
    ASSERT_EQ(size_t(2), Mat1i(2, 1).total());
    ASSERT_EQ(size_t(0), Mat1i(1, 0).total());
    ASSERT_EQ(size_t(0), Mat1i(0, 1).total());
    ASSERT_EQ(size_t(0), Mat1i(0, 0).total());

    ASSERT_EQ(size_t(1), Mat1f(1, 1).total());
    ASSERT_EQ(size_t(2), Mat1f(1, 2).total());
    ASSERT_EQ(size_t(2), Mat1f(2, 1).total());
    ASSERT_EQ(size_t(0), Mat1f(1, 0).total());
    ASSERT_EQ(size_t(0), Mat1f(0, 1).total());
    ASSERT_EQ(size_t(0), Mat1f(0, 0).total());

    EXPECT_THROW(Mat2Point2i(Mat1i()),        ExceptionBadDims);
    EXPECT_THROW(Mat2Point2i(Mat1i(0, 0)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point2i(Mat1i(1, 0)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point2i(Mat1i(0, 1)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point2i(Mat1i(1, 1)),    ExceptionBadDims);

    EXPECT_THROW(Mat2Point3_(Mat1f()),        ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1f(0, 0)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1f(1, 0)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1f(0, 1)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1f(1, 1)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1f(2, 1)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1f(1, 2)),    ExceptionBadDims);

    EXPECT_THROW(Mat2Point3_(Mat1i()),        ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1i(0, 0)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1i(1, 0)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1i(0, 1)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1i(1, 1)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1i(2, 1)),    ExceptionBadDims);
    EXPECT_THROW(Mat2Point3_(Mat1i(1, 2)),    ExceptionBadDims);
}

/**
 * @brief A setup for repeating tests with different types of mat objects (int, float, uchar)
 * @todo convert to TYPED_TESTS
 */
template <class T>
class MatPODTypesTest : public testing::Test
{
protected:
};

// define some helper routines and classes

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
TYPED_TEST_P(MatPODTypesTest, FindFirstOf_Invalid_MultiChannel) {

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
TYPED_TEST_P(MatPODTypesTest, FindFirstOf_Invalid_MultiChannelEmpty) {

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
TYPED_TEST_P(MatPODTypesTest, FindFirstOf_Invalid_NonContinuous) {

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

/**
 * @brief Keep tabs on what happens when assigning a Mat of M channels to a Mat of N channels or generic Mat
 */
TYPED_TEST_P(MatPODTypesTest, InterChannelCasting)
{
    ASSERT_GE(V_<float>::values.size(), size_t(3)) << "This test requires at least 3 values to be defined.";

    Mat_<TypeParam > m1(1, 3);

    for(int i=0; i<m1.cols; i++) {

        m1(i) = V_<TypeParam >::values[i];
    }

    typedef Vec<TypeParam, 3 > Vec3TP;
    Mat_<Vec3TP > m3 = m1;

    EXPECT_EQ(1, m1.channels()) << "Unexpected no. of channels.";
    EXPECT_EQ(3, m3.channels()) << "Unexpected no. of channels.";
    for(int i=0; i<m1.cols; i++) {

        EXPECT_EQ(m1(i), m3(0)[i]);
        EXPECT_EQ(V_<TypeParam>::values[i], m3(0)[i]) << "Mismatch at i=" << i;
    }

    Mat mm = m1;
    EXPECT_EQ(1, mm.channels()) << "Unexpected no. of channels.";
    for(int i=0; i<m1.cols; i++) {

        EXPECT_EQ(m1(i), mm.at<TypeParam >(i));
        EXPECT_EQ(V_<TypeParam>::values[i], mm.at<TypeParam >(i)) << "Mismatch at i=" << i;
    }

    Mat mm3 = m3;
    EXPECT_EQ(3, mm3.channels()) << "Unexpected no. of channels.";
    for(int i=0; i<m1.cols; i++) {

        EXPECT_EQ(m3(0)[i], mm3.at<Vec3TP >(0)[i]);
        EXPECT_EQ(V_<TypeParam>::values[i], mm3.at<Vec3TP >(0)[i]) << "Mismatch at i=" << i;
    }
}

/* Write additional type+value parameterized tests here.
 * Acquaint yourself with the values passed to along with each type.
 */

// Register test names
REGISTER_TYPED_TEST_CASE_P(MatPODTypesTest,
                           FindFirstOf_Invalid_NonContinuous,
                           FindFirstOf_Invalid_MultiChannel,
                           FindFirstOf_Invalid_MultiChannelEmpty,
                           FindFirstOf_Empty,
                           FindFirstOf_NotFound,
                           FindFirstOf_Found,
                           FindFirstOf_Duplicate,
                           InterChannelCasting); ///< register additional typed_test_p (i.e. unit test) routines here

///< Register values to work with inside tests, note how they're used inside the tests
template<> std::vector<float> V_<float>::values{-1.f, 0.f, 1.f, 100.f, 101.f, 200.f};
template<> std::vector<int> V_<int>::values{-1, 0, 1, 100, 101, 200};
template<> std::vector<uchar> V_<uchar>::values{255, 0, 1, 100, 101, 200};

typedef testing::Types<float, int, uchar> PODTypes;  ///< lists the usual suspects of matrices
INSTANTIATE_TYPED_TEST_CASE_P(MatUtilsPODTypesTest, MatPODTypesTest, PODTypes);

/**
 * @brief Keep tabs on overload call precedence
 */
class MatOverloadCallPrecedenceTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        const int MAT_ROWS = abs(randu<int>()) % 10 + 1;
        const int MAT_COLS = abs(randu<int>()) % 100 + 1;

        mi_ = Mat_<int>(MAT_ROWS, MAT_COLS);
        randn(mi_, 0, 100);  // nothing too large so that it can be fully represented in other type

        mf_ = Mat_<float>(MAT_ROWS, MAT_COLS);
        for(size_t i=0; i<static_cast<size_t>(MAT_ROWS*MAT_COLS); i++) {

            mf_(i) = static_cast<float>(mi_(i));
        }
    }

    Mat_<int> mi_;
    Mat_<float> mf_;
};

enum OverloadId
{
    _MAT,
    _1I,
    _TT,
    _TEmptyEmpty,
    _2F,
    _TF,
    _TI
};

OverloadId foo(const Mat &a, const Mat &b) { return _MAT; }

OverloadId foo(const Mat1i &a, const Mat1f &b) { return _1I; }

template <class T1, class T2>
OverloadId foo(const Mat_<T1> &a, const Mat_<T2> &b) { return _TT; }

template <>
OverloadId foo(const Mat1i &a, const Mat1f &b) { return _TEmptyEmpty; }

OverloadId foo(const Mat1f &a, const Mat1i &b) { return _2F; }
template <class T1>

OverloadId foo(const Mat_<T1> &a, const Mat1f &b) { return _TF; }

template <class T1>
OverloadId foo(const Mat_<T1> &a, const Mat1i &b) { return _TI; }

TEST_F(MatOverloadCallPrecedenceTest, SanityCheck)
{
    ASSERT_NE(_MAT, _TF);
    ASSERT_NE(_MAT, _1I);
    ASSERT_NE(_1I, _TF);

    EXPECT_EQ(foo(Mat1f(), Mat1f()), foo(Mat_f(), Mat_f()));
    EXPECT_NE(foo(Mat1i(), Mat1f()), foo(Mat_f(), Mat_f()));

    /* comment those out unless you want to see the warning:
        warning: ISO C++ says that these are ambiguous,
        even though the worst conversion for the first
        is better than the worst conversion for the second: [enabled by default]
        */
//    EXPECT_NE(foo(Mat1i(), Mat1f()), foo(Mat(), Mat_f()));
//    EXPECT_NE(foo(Mat1i(), Mat1i()), foo(Mat_f(), Mat()));

}

TEST_F(MatOverloadCallPrecedenceTest, CheckId)
{
    EXPECT_MAT_EQ(mi_, mf_) << "Matrices are not equal";

    EXPECT_EQ(_1I, foo(mi_, mf_));

    EXPECT_EQ(_MAT, foo(Mat(), Mat()));

    EXPECT_EQ(_TF, foo(Mat_f(), Mat_f()));
    EXPECT_EQ(_1I, foo(Mat1i(), Mat1f()));
    EXPECT_EQ(_TI, foo(Mat1i(), Mat1i()));
    EXPECT_EQ(_TF, foo(Mat1f(), Mat1f()));
    EXPECT_EQ(_2F, foo(Mat1f(), Mat1i()));

    /* comment those out unless you want to see the warning:
        warning: ISO C++ says that these are ambiguous,
        even though the worst conversion for the first
        is better than the worst conversion for the second: [enabled by default]
        */
//    EXPECT_EQ(_MAT, foo(Mat(), Mat_f()));
//    EXPECT_EQ(_MAT, foo(Mat_f(), Mat()));

}
