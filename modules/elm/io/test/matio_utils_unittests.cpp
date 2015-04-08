/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/io/matio_utils.h"

#ifdef __WITH_MATIO

#include "gtest/gtest.h"

#include <opencv2/core/core.hpp>

#include "matio.h"

#include "elm/core/exception.h"
#include "elm/ts/container.h"

using namespace elm;

namespace {

TEST(MATIO_UTILS_TEST, MATIOClassTOCV_TYPE)
{
    EXPECT_EQ(CV_8S, MATIOClassTOCV_TYPE(MAT_C_CHAR));
    EXPECT_EQ(CV_64F, MATIOClassTOCV_TYPE(MAT_C_DOUBLE));
    EXPECT_EQ(CV_8U, MATIOClassTOCV_TYPE(MAT_C_UINT8));
    EXPECT_EQ(CV_16U, MATIOClassTOCV_TYPE(MAT_C_UINT16));
    EXPECT_EQ(CV_8S, MATIOClassTOCV_TYPE(MAT_C_INT8));
    EXPECT_EQ(CV_16S, MATIOClassTOCV_TYPE(MAT_C_INT16));
    EXPECT_EQ(CV_32S, MATIOClassTOCV_TYPE(MAT_C_INT32));
    EXPECT_EQ(CV_32F, MATIOClassTOCV_TYPE(MAT_C_SINGLE));
}

/**
 * @brief types that don't have a direct mapping but could still be cast
 */
TEST(MATIO_UTILS_TEST, MATIOClassTOCV_TYPE_not_supported)
{
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_UINT32), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_UINT64), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_INT64), ExceptionTypeError);
}

TEST(MATIO_UTILS_TEST, MATIOClassTOCV_TYPE_invalid)
{
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_CELL), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_FUNCTION), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_EMPTY), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_OBJECT), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_STRUCT), ExceptionTypeError);
    EXPECT_THROW(MATIOClassTOCV_TYPE(MAT_C_SPARSE), ExceptionTypeError);
}

TEST(MATIO_UTILS_TEST, Ind2sub_size)
{
    for(int d=1; d<10; d++) {

        int *SIZES = new int[d];

        for(int i=0; i<d; i++) {

            SIZES[i] = 1;
        }

        std::vector<int> subs;
        ind2sub(0, d, SIZES, subs);
        EXPECT_SIZE(d, subs);

        delete [] SIZES;
    }
}

TEST(MATIO_UTILS_TEST, Ind2sub_first_el)
{
    const int DIMS=3;
    const int SIZES[DIMS] = {4, 3, 2};

    std::vector<int> subs;
    ind2sub(0, DIMS, SIZES, subs);

    for(int i=0; i<DIMS; i++) {

        EXPECT_EQ(0, subs[i]);
    }
}

TEST(MATIO_UTILS_TEST, Ind2sub_last_el)
{
    const int DIMS=3;
    const int SIZES[DIMS] = {4, 3, 2};

    std::vector<int> subs;
    ind2sub(4*3*2-1, DIMS, SIZES, subs);

    for(int i=0; i<DIMS; i++) {

        EXPECT_EQ(SIZES[i]-1, subs[i]);
    }
}

TEST(MATIO_UTILS_TEST, Ind2sub_1d)
{
    const int DIMS=1;
    const int SIZES[DIMS] = {4};

    std::vector<int> subs;
    ind2sub(2, DIMS, SIZES, subs);

    EXPECT_SIZE(1, subs);
    EXPECT_EQ(2, subs[0]);
}


TEST(MATIO_UTILS_TEST, Ind2sub_2d)
{
    const int DIMS=2;
    const int SIZES[DIMS] = {4, 3};

    std::vector<int> subs;
    ind2sub(7, DIMS, SIZES, subs);

    EXPECT_SIZE(2, subs);
    EXPECT_EQ(2, subs[0]);
    EXPECT_EQ(1, subs[1]);
}

TEST(MATIO_UTILS_TEST, Ind2sub_3d)
{
    const int DIMS=3;
    const int SIZES[DIMS] = {4, 3, 2};

    std::vector<int> subs;

    ind2sub(19, DIMS, SIZES, subs);

    EXPECT_SIZE(3, subs);
    EXPECT_EQ(3, subs[0]);
    EXPECT_EQ(0, subs[1]);
    EXPECT_EQ(1, subs[2]);
}

TEST(MATIO_UTILS_TEST, Sub2ind_first_el)
{
    const int DIMS=3;
    const int SIZES[DIMS] = {4, 3, 2};

    int subs[DIMS] = {0, 0, 0};
    EXPECT_EQ(0, sub2ind(DIMS, SIZES, subs));
}

TEST(MATIO_UTILS_TEST, Sub2ind_last_el)
{
    const int DIMS=3;
    const int SIZES[DIMS] = {4, 3, 2};

    int subs[DIMS] = {3, 2, 1};
    EXPECT_EQ(4*3*2-1, sub2ind(DIMS, SIZES, subs));
}

TEST(MATIO_UTILS_TEST, Sub2ind_1d)
{
    const int DIMS=1;
    const int SIZES[DIMS] = {4};

    int subs[DIMS] = {2};
    EXPECT_EQ(2, sub2ind(DIMS, SIZES, subs));
}

TEST(MATIO_UTILS_TEST, Sub2ind_2d)
{
    const int DIMS=2;
    const int SIZES[DIMS] = {4, 3};

    int subs[DIMS] = {2, 1};
    EXPECT_EQ(7, sub2ind(DIMS, SIZES, subs));
}

TEST(MATIO_UTILS_TEST, Sub2ind_3d)
{
    const int DIMS=3;
    const int SIZES[DIMS] = {4, 3, 2};

    int subs[DIMS] = {3, 0, 1};
    EXPECT_EQ(19, sub2ind(DIMS, SIZES, subs));
}

} // annonymous namespace for unit tests

#endif // __WITH_MATIO
