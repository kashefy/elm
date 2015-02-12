/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/cv/sparsemat_utils.h"

#include "elm/ts/mat_assertions.h"

using namespace cv;
using namespace elm;

namespace {

TEST(SparseMatUtils, Total)
{
    EXPECT_EQ(static_cast<size_t>(0), total(SparseMat()));

    int sz[2] = {3, 4};
    EXPECT_EQ(static_cast<size_t>(12), total(SparseMat1f(2, sz)));

    for(int dims=1; dims<10; dims++) {

        int prod = 1;

        int *sz = new int[dims];
        for(int i=0; i<dims; i++) {

            sz[i] = i+1;
            prod *= (i+1);
        }

        EXPECT_EQ(static_cast<size_t>(prod), total(SparseMat1f(dims, sz)));

        delete [] sz;
    }
}

} // annonymous namespace for unit tests
