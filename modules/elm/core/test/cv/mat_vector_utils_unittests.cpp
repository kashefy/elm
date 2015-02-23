/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/cv/mat_vector_utils.h"
#include "elm/core/cv/mat_vector_utils_inl.h"

#include "elm/core/stl/stl_inl.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

TEST(Mat_ToVec_Test, Empty)
{
    EXPECT_EMPTY(Mat_ToVec_<float>(Mat1f()));
    EXPECT_EMPTY(Mat_ToVec_<float>(Mat1i()));
    EXPECT_EMPTY(Mat_ToVec_<float>(Mat1b()));
    EXPECT_EMPTY(Mat_ToVec_<float>(Mat()));
}

TEST(Mat_ToVec_Test, Invalid)
{
    EXPECT_THROW(Mat_ToVec_<float>(Mat1f::zeros(3, 4).col(0)), ExceptionTypeError);
    EXPECT_THROW(Mat_ToVec_<float>(Mat1f(3, 4, 11.f).colRange(0, 2)), ExceptionTypeError);
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
    VecF out = Mat_ToVec_<float>(in);
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
            for(VecMat1f::const_iterator itr=data.begin();
                itr != data.end();
                itr++, k++) {

                elements_expected(k) = (*itr)(r, c);
            }
            Mat1f elements_actual = elm::ElementsAt(data, r, c);

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

/**
 * @brief test utility function for extracting lower triangular part of matrix
 */
class TrilTest : public ::testing::Test
{
protected:
};

TEST_F(TrilTest, Empty)
{
    VecMat1f vec_mat;

    int n;
    n = tril(Mat1f(), vec_mat);
    EXPECT_SIZE(0, vec_mat);
    EXPECT_EQ(0, n);

    n = tril(Mat1f(0, 0), vec_mat);
    EXPECT_SIZE(0, vec_mat);
    EXPECT_EQ(0, n);

    n = tril(Mat1f(0, 3), vec_mat);
    EXPECT_SIZE(0, vec_mat);
    EXPECT_EQ(0, n);

    n = tril(Mat1f(3, 0), vec_mat);
    EXPECT_SIZE(0, vec_mat);
    EXPECT_EQ(0, n);
}

TEST_F(TrilTest, Dims)
{
    int n_expected = 0;
    for(int i=1; i<10; i++) {

        Mat1f m(i, i, i);
        VecMat1f vec_mat;
        int n = tril(m, vec_mat);

        EXPECT_EQ(n_expected, n);

        EXPECT_EQ(vec_mat.size(), size_t(i-1));
        for(size_t r=0; r<vec_mat.size(); r++) {

            EXPECT_MAT_DIMS_EQ(vec_mat[r], Size2i(r+1, 1));
        }

        n_expected += i;
    }
}

TEST_F(TrilTest, Values)
{
    Mat1f m(4, 4);
    randn(m, 0.f, 100.f);

    VecMat1f vec_mat;
    int n = tril(m, vec_mat);
    EXPECT_EQ(vec_mat.size(), size_t(m.rows-1));
    EXPECT_EQ(6, n);

    for(size_t r=0; r<vec_mat.size(); r++) {

        EXPECT_MAT_EQ(vec_mat[r], m.row(r+1).colRange(0, r+1));
    }
}

/**
 * @brief Typed tests around Vector routines with POD types
 */
template <class T>
class CV_MatVectorUtils : public ::testing::Test
{
protected:
};
typedef ::testing::Types<float, int, uchar> PODTypes;
TYPED_TEST_CASE(CV_MatVectorUtils, PODTypes);

TYPED_TEST(CV_MatVectorUtils, Mat_ToVec_ThreeDimensional)
{
    const int ROWS=2, COLS=3, PLANES=4;

    ASSERT_GT(ROWS*COLS*PLANES, 0) << "Useless test if there aren't any elements.";

    int d[3] = {ROWS, COLS, PLANES};
    Mat in = Mat(3, d, CV_MAKETYPE(DataType<TypeParam>::depth, 1));

    randn(in, 0, 100);
    vector<TypeParam > out = Mat_ToVec_<TypeParam >(in);
    EXPECT_EQ(size_t(ROWS*COLS*PLANES), in.total()) << "Not all elements acccounted for.";
    EXPECT_SIZE(in.total(), out) << "Not all elements acccounted for.";

    // check values
    int i=0;
    for(int r=0; r<ROWS; r++) {
        for(int c=0; c<COLS; c++) {
            for(int p=0; p<PLANES; p++) {

                EXPECT_TRUE(in.at<TypeParam>(r, c, p) == out[i++])
                        << "value mismatch at (" << r << "," << c << "," << p << ")";
            }
        }
    }
}

TYPED_TEST(CV_MatVectorUtils, Mat_ToVec_Invalid_NonContinuous_Mat_)
{
    typedef Mat_<TypeParam> MatTP;

    // copy vector of values into matrix object
    MatTP m(2, 3);
    randn(m, 0, 100);

    /* provoke non-continuous input by taking out column ranges
     */
    // col() yielding single columns
    for(int c=0; c<m.cols; c++) {

        EXPECT_THROW( Mat_ToVec_(m.col(c)), ExceptionTypeError );
        EXPECT_THROW( Mat_ToVec_(m.col(c)), ExceptionTypeError );
    }

    // colRange()
    for(int c=1; c<m.cols-1; c++) {

        EXPECT_THROW( Mat_ToVec_<TypeParam >(m.colRange(c, m.cols)), ExceptionTypeError );
    }

    for(int c=1; c<m.cols; c++) {

        EXPECT_THROW( Mat_ToVec_<TypeParam >(m.colRange(0, c)), ExceptionTypeError );
    }
}

// STL vector to template Mat_ conversions
/**
 * @brief Empty in empty out
 */
TYPED_TEST(CV_MatVectorUtils, Vec_TRowMat_Empty)
{
    vector<TypeParam > v;
    EXPECT_TRUE(Vec_ToRowMat_<TypeParam >(v).empty());
}

/**
 * @brief test that a row matrix is produced with the correct no. of cols
 */
TYPED_TEST(CV_MatVectorUtils, Vec_TRowMat_AlwaysRowMat)
{
    for(int s=1; s<10; s++) {

        vector<TypeParam > v(s, randu<TypeParam >());

        Mat_<TypeParam > m = Vec_ToRowMat_<TypeParam >(v);
        ASSERT_FALSE(m.empty()) << "Why is this matrix empty?";
        ASSERT_EQ(1, m.rows)    << "Expecting row matrix. Expecting matrix with a single row";
        ASSERT_EQ(s, m.cols);
        ASSERT_EQ(s, static_cast<int>(m.total()));
        EXPECT_MAT_DIMS_EQ(m, Size2i(static_cast<int>(v.size()), 1)) << "Encountered unexpected size";
    }
}

/**
 * @brief test resulting matrix element values
 */
TYPED_TEST(CV_MatVectorUtils, Vec_TRowMat_Values)
{
    for(int s=1; s<10; s++) {

        vector<TypeParam > v;
        push_back_randu<TypeParam >(v, s);

        Mat_<TypeParam > m = Vec_ToRowMat_<TypeParam >(v);
        EXPECT_MAT_DIMS_EQ(m, Size2i(static_cast<int>(v.size()), 1)) << "Encountered unexpected size";

        // check values
        for(int i=0; i<s; i++) {
            EXPECT_EQ(v[i], m(i));
            EXPECT_EQ(v[i], m(0, i));
        }
    }
}

/**
 * @brief Test that no copying happened and that both point to the same memory chunk.
 *Q: Who owns the data? A: the source vector
 */
TYPED_TEST(CV_MatVectorUtils, Vec_TRowMat_NoCopy)
{
    const int SIZE=3;

    // initialize a vector with random values
    vector<TypeParam > v;
    push_back_randu<TypeParam >(v, SIZE);

    Mat_<TypeParam > m = Vec_ToRowMat_<TypeParam >(v);

    // modify each element in one and see it reflect in the other
    for(int i=0; i<SIZE; i++) {

        // check before doing anything
        EXPECT_EQ(v[i], m(i));
        EXPECT_EQ(v[i], m(0, i));

        // modify in vector
        v[i] = randu<TypeParam >();
        EXPECT_EQ(v[i], m(i));
        EXPECT_EQ(v[i], m(0, i));

        // modify in matrix
        m(i) = randu<TypeParam >();
        EXPECT_EQ(v[i], m(i));
        EXPECT_EQ(v[i], m(0, i));
    }
}

/**
 * @brief Q: Who owns the data? A: the source vector
 *
 * We'll test for:
 * 1) If the matrix goes out of scope or is released -> vector is not affected
 * 2) If the vector goes out of scope matrix values become undetermined
 */
TYPED_TEST(CV_MatVectorUtils, Vec_TRowMat_Ownership)
{
    const int SIZE=3;

    Mat_<TypeParam > m_outer;
    Mat_<TypeParam > m_bckp;
    vector<TypeParam > bckp;
    {
        // initialize a vector with random values
        vector<TypeParam > v;
        push_back_randu(v, SIZE);

        // make an explicit back up of the vector values
        bckp = v;

        for(int i=0; i<SIZE; i++) {
            EXPECT_EQ(v[i], bckp[i]);
        }

        // test ownership after mat release
        {
            Mat_<TypeParam > m_inner = Vec_ToRowMat_(v);
            m_inner.copyTo(m_bckp);
            m_inner.release(); // will not affect the data

            for(int i=0; i<SIZE; i++) {
                EXPECT_EQ(v[i], bckp[i]);
            }
        }

        // check again after mat going out of scope
        for(int i=0; i<SIZE; i++) {
            EXPECT_EQ(v[i], bckp[i]);
        }

        m_outer = Vec_ToRowMat_(v);
        for(int i=0; i<SIZE; i++) {
            EXPECT_EQ(v[i], m_outer(i));
        }
    }

    for(int i=0; i<SIZE; i++) {

        EXPECT_EQ(bckp[i], m_bckp(i));
    }

    Mat cmp_out;
    compare(m_outer, m_bckp, cmp_out, CMP_NE);
    int n = countNonZero(cmp_out); // n==0 -> equal
    EXPECT_NE(n, 0) << "They turned out to be equal.";
}

/**
 * @brief test assignment to regular non-template Mat object
 */
TYPED_TEST(CV_MatVectorUtils, Vec_TRowMat_AssignToMat)
{
    const int SIZE=3;

    // initialize a vector with random values
    vector<TypeParam > v;
    push_back_randu<TypeParam >(v, SIZE);

    Mat m = Vec_ToRowMat_<TypeParam >(v);

    EXPECT_MAT_DIMS_EQ(m, Size2i(static_cast<int>(v.size()), 1)) << "Encountered unexpected size";
    EXPECT_MAT_TYPE(m, DataType<TypeParam >::depth) << "Unexpected mat type";

    // check values
    for(int i=0; i<SIZE; i++) {
        EXPECT_EQ(v[i], m.at<TypeParam >(i));
        EXPECT_EQ(v[i], m.at<TypeParam >(0, i));
    }
}


} // annonymous namespace for unit tests
