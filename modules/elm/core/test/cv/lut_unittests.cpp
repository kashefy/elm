/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/cv/lut.h"

#include "elm/ts/mat_assertions.h"

using cv::Mat1i;
using namespace elm;

namespace {

/**
 * @brief test case around ELM's alternate LUT class
 */
class LUTTest : public ::testing::Test
{
protected:
};

TEST_F(LUTTest, Apply_empty_table)
{
    {
        Mat1i m0 = Mat1i::ones(2, 3);
        Mat1i m = m0.clone();
        LUT to;
        to.apply(m);
        EXPECT_MAT_EQ(m0, m);
    }
    {
        Mat1i m0 = Mat1i::zeros(2, 3);
        Mat1i m = m0.clone();
        elm::LUT to;
        to.apply(m);
        EXPECT_MAT_EQ(m0, m);
    }
}

TEST_F(LUTTest, Apply_empty_table_empty_mat)
{
    LUT to;
    Mat1i m;
    EXPECT_NO_THROW(to.apply(m));
}

TEST_F(LUTTest, Apply)
{
    const int N = 6;
    int data[N] = {1, 2,
                   1, 4,
                   6, 1};
    Mat1i m = Mat1i(3, 2, data).clone();

    LUT to(N);

    to.insert(1);
    to.insert(2);
    to.insert(4);
    to.insert(6);

    EXPECT_EQ(2, to.update(2, 4)) << "not returning smaller value.";

    to.apply(m);

    int data2[N] = {1, 2,
                    1, 2,
                    6, 1};
    Mat1i expected = Mat1i(3, 2, data2).clone();
    EXPECT_MAT_EQ(expected, m);
}

TEST_F(LUTTest, Update)
{
    const int N = 6;
    int data[N] = {1, 2,
                   1, 4,
                   6, 1};
    Mat1i m = Mat1i(3, 2, data).clone();

    LUT to(N);

    to.insert(2);
    to.insert(4);

    EXPECT_EQ(2, to.update(2, 4)) << "not returning smaller value.";
    EXPECT_EQ(2, to.update(4, 2)) << "not returning smaller value.";
    EXPECT_EQ(4, to.update(4, 4));
}

TEST_F(LUTTest, Apply_chained)
{
    const int N = 6;
    int data[N] = {1, 2,
                   1, 4,
                   6, 1};
    Mat1i m = Mat1i(3, 2, data).clone();

    LUT to;
    to.Capacity(N);

    to.insert(1);
    to.insert(2);
    to.insert(4);
    to.insert(6);

    to.update(2, 4);
    to.update(2, 6);

    to.apply(m);

    int data2[N] = {1, 2,
                    1, 2,
                    2, 1};

    Mat1i expected = Mat1i(3, 2, data2).clone();
    EXPECT_MAT_EQ(expected, m);
}

/**
 * @brief Cover scenario of values replaced by lower as well as higher ones
 */
TEST_F(LUTTest, Apply_to_higher_lower)
{
    const int N = 6;
    int data[N] = {1, 2,
                   5, 4,
                   6, 1};
    Mat1i m = Mat1i(3, 2, data).clone();

    LUT to;
    to.Capacity(N);

    to.insert(1);
    to.insert(2);
    to.insert(4);
    to.insert(5);
    to.insert(6);

    to.update(2, 6);
    to.update(1, 2);

    to.apply(m);

    int data2[N] = {1, 1,
                    5, 4,
                    1, 1};

    Mat1i expected = Mat1i(3, 2, data2).clone();
    EXPECT_MAT_EQ(expected, m);
}

TEST_F(LUTTest, Assign_chained_empty_mat)
{
    Mat1i m;

    LUT to;
    to.Capacity(100);

    to.insert(1);
    to.insert(2);
    to.insert(4);
    to.insert(6);

    to.update(2, 4);
    to.update(2, 6);

    EXPECT_NO_THROW(to.apply(m));
    EXPECT_TRUE(m.empty());
}


} // annonymous namespace for tests
