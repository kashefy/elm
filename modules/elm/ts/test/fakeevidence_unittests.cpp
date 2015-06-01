/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/fakeevidence.h"

#include "elm/ts/mat_assertions.h"

namespace {

TEST(FakeEvidenceTest, Dims) {

    for(int d=1; d<10; d++) {

        FakeEvidence to(d); // test object

        EXPECT_MAT_DIMS_EQ(to.next(1), cv::Size2i(d, 1));
    }
}

} // annonymous namespace for unit tests
