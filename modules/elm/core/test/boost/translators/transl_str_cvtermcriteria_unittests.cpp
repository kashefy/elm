/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/boost/translators/transl_str_cvtermcriteria.h"

#include "gtest/gtest.h"

#include <elm/core/typedefs_fwd.h>

using namespace elm;

namespace {

TEST(transl_CVTermCriteriaTest, Add_to_ptree)
{
    PTree pt;
    {
        pt.add("x", 1);

        CvTermCriteria c;
        c.epsilon = 0.01;
        c.max_iter = 100;
        c.type = CV_TERMCRIT_EPS | CV_TERMCRIT_ITER;

        pt.add("crit", c);
    }

    CvTermCriteria c = pt.get<CvTermCriteria>("crit");

    EXPECT_EQ(0.01, c.epsilon);
    EXPECT_EQ(100, c.max_iter);
    EXPECT_EQ(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, c.type);
}

TEST(transl_CVTermCriteriaTest, PTree_owns_copy)
{
    PTree pt;
    {
        CvTermCriteria c;
        c.epsilon = 0.01;
        c.max_iter = 100;
        c.type = CV_TERMCRIT_EPS | CV_TERMCRIT_ITER;

        pt.add("crit", c);
    }

    CvTermCriteria c1 = pt.get<CvTermCriteria>("crit");
    CvTermCriteria c2 = pt.get<CvTermCriteria>("crit");

    c2.epsilon  = 0.1;
    c2.max_iter = 10;
    c2.type     = CV_TERMCRIT_NUMBER;

    EXPECT_NE(c2.epsilon, c1.epsilon);
    EXPECT_NE(c2.max_iter, c1.max_iter);
    EXPECT_NE(c2.type, c1.type);

    EXPECT_EQ(0.01, c1.epsilon);
    EXPECT_EQ(100, c1.max_iter);
    EXPECT_EQ(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, c1.type);
}

TEST(transl_CVTermCriteriaTest, Update_value)
{
    PTree pt;
    {
        CvTermCriteria c;
        c.epsilon = 0.01;
        c.max_iter = 100;
        c.type = CV_TERMCRIT_EPS | CV_TERMCRIT_ITER;

        pt.add("crit", c);
    }
    {
        CvTermCriteria c;
        c.epsilon = 0.1;
        c.max_iter = 1000;
        c.type = CV_TERMCRIT_ITER;

        pt.put("crit", c);
    }

    CvTermCriteria c = pt.get<CvTermCriteria>("crit");

    EXPECT_EQ(0.1, c.epsilon);
    EXPECT_EQ(1000, c.max_iter);
    EXPECT_EQ(CV_TERMCRIT_ITER, c.type);
    EXPECT_NE(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, c.type);
}

} // annonymous namespace for tests
