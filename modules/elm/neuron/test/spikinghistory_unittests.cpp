/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/neuron/spikinghistory.h"

#include "elm/ts/ts.h"

using namespace cv;

namespace {

class SpikingHistoryTest : public ::testing::Test
{
protected:
    SpikingHistoryTest()
        : to_(5, 3),
          len_(-1),
          dims_(0)
    {
    }

    virtual void SetUp()
    {
        len_  = 5;
        dims_ = 3;
        to_ = SpikingHistory(dims_, len_);
        to_.Update(Mat::ones(1, dims_, CV_8UC1));
    }

    SpikingHistory to_; ///< test object
    int len_;           ///< history length
    int dims_;          ///< dimensions
};

TEST_F(SpikingHistoryTest, InitialHistory)
{
    EXPECT_MAT_EQ(SpikingHistory(dims_, len_).History(),  Mat1i::zeros(1, dims_)) << "History mismatch";
}

TEST_F(SpikingHistoryTest, Reset)
{
    Mat1i expected = Mat1i::zeros(1, dims_);
    for(int i=0; i<len_*3; i++) {

        to_.Reset();
        EXPECT_MAT_EQ(to_.History(), expected) << "History mismatch at i=" << i;
    }
}

TEST_F(SpikingHistoryTest, Reset_index)
{
    for(int i=0; i<len_*3; i++) {

        EXPECT_MAT_EQ(to_.History(), Mat1i(1, dims_, len_)) << "History not initialized properly for this iteration.";

        int index = abs(randu<int>()) % dims_;
        to_.Reset(index);

        Mat1i expected = Mat1i(1, dims_, len_);
        expected(index) = 0;

        EXPECT_MAT_EQ(to_.History(), expected) << "History mismatch at i=" << i;

        to_.Update(Mat::ones(1, dims_, CV_8UC1));
    }
}

TEST_F(SpikingHistoryTest, Advance)
{
    Mat1i expected = Mat1i::zeros(1, dims_)+len_;
    for(int i=0; i<len_*3; i++) {

        to_.Advance();
        if(i < len_) {

            expected -= 1;
            EXPECT_MAT_EQ(to_.History(), expected) << "History mismatch at i=" << i;
        }
        else {

            EXPECT_MAT_EQ(to_.History(), Mat1i::zeros(1, dims_)) << "History mismatch at i=" << i;
        }
    }
}

TEST_F(SpikingHistoryTest, AdvanceThenReset)
{
    to_.Advance();
    to_.Advance();

    Mat1i expected = Mat1i::zeros(1, dims_)+len_;

    EXPECT_MAT_EQ(to_.History(), expected-2) << "History mismatch before Reset";

    to_.Reset();

    EXPECT_MAT_EQ(to_.History(), Mat1i::zeros(1, dims_)) << "History mismatch after Reset";
}

TEST_F(SpikingHistoryTest, RecentIndex)
{
    for(int i=0; i<len_*3; i++) {

        Mat1i h = to_.History();
        for(int index=0; index<h.cols; index++) {

            if(i < len_) { EXPECT_TRUE  (to_.Recent(index)); }
            else         { EXPECT_FALSE (to_.Recent(index)); }
        }
        to_.Advance();
    }
}

TEST_F(SpikingHistoryTest, Recent)
{
    for(int i=0; i<len_*3; i++) {

        Mat1i h = to_.History();
        for(int index=0; index<h.cols; index++) {

            if(i < len_) {

                EXPECT_TRUE(to_.Recent(index));
                EXPECT_GE(sum(to_.Recent())[0], h.cols);
            }
            else {
                EXPECT_FALSE(to_.Recent(index));
                EXPECT_EQ(sum(to_.Recent())[0], 0);
            }
        }
        to_.Advance();
    }
}

TEST_F(SpikingHistoryTest, Update)
{
    RNG rng;
    int n = 0;
    while(n++ < len_*3) {

        to_.Advance();
        int index = int(rng.uniform(1, dims_));

        Mat1i spikes = Mat1i::zeros(1, dims_);
        spikes(index) = 1;
        Mat mask = spikes > 0;

        to_.Update(mask);

        EXPECT_TRUE(to_.Recent(index));

        if(n-1 < len_-1)    { EXPECT_TRUE  (to_.Recent(0)); }
        else                { EXPECT_FALSE (to_.Recent(0)); }
    }
}

/**
 * @brief test around ColRange functionality of SpikingHistory
 * Applying SpikingHistory methods on a history subset
 */
class SpikingHistoryColRangeTest : public SpikingHistoryTest
{
protected:
    SpikingHistoryColRangeTest()
        : SpikingHistoryTest(),
          to_cr_(5, 3)
    {
    }

    virtual void SetUp()
    {
        SpikingHistoryTest::SetUp();

        start_ = 1;
        end_   = dims_-1;

        // check if test is applicable with this set up
        ASSERT_GT(end_, start_)
                << "Increase dims for test to be applicable, must allow extracting a column range of > 0 cols.";

        to_cr_ = to_.ColRange(start_, end_);
    }

    SpikingHistory to_cr_;  ///< test object column range

    int start_;         ///< colRange start
    int end_;           ///< colRange end
};

/**
 * @brief test behavior when requesting invalid column ranges (e.g. out of bounds, negative indices,...)
 * Error messages from OpenCV exceptions appear in console.
 */
TEST_F(SpikingHistoryColRangeTest, InvalidRange)
{
    EXPECT_NO_THROW(to_.ColRange(0, 0));

    EXPECT_THROW(to_.ColRange(0, dims_+1),       cv::Exception);
    EXPECT_THROW(to_.ColRange(dims_+1, dims_+2), cv::Exception);
    EXPECT_THROW(to_.ColRange(2, 1),             cv::Exception);
}

TEST_F(SpikingHistoryColRangeTest, Dims)
{
    EXPECT_GT(to_cr_.History().cols, 0) << "No cols in sub history";
    EXPECT_FALSE(to_cr_.History().empty()) << "Sub history must not be empty";

    EXPECT_MAT_DIMS_EQ(to_cr_.History(), to_.History().colRange(start_, end_)) << "Column range dims mismatch";
}

TEST_F(SpikingHistoryColRangeTest, Values)
{
    const int N=30;

    Mat1b evidence(1, dims_);

    for(int i=0; i<N; i++) {

        randu(evidence, 0, 2);

        to_.Update(evidence > 0);

        if(i==10) {
            to_.Reset();
        }

        EXPECT_GT(to_cr_.History().cols, 0) << "No cols in sub history";
        EXPECT_FALSE(to_cr_.History().empty()) << "Sub history must not be empty";

        EXPECT_MAT_EQ(to_cr_.History(), to_.History().colRange(start_, end_)) << "Column range mismatch";

        to_.Advance();
    }
}

TEST_F(SpikingHistoryColRangeTest, SingleCol)
{
    const int N=30;

    start_ = 0;
    end_ = 1;
    to_cr_ = to_.ColRange(start_, end_);

    Mat1b evidence(1, dims_);

    for(int i=0; i<N; i++) {

        randu(evidence, 0, 2);

        to_.Update(evidence > 0);

        if(i==10) {
            to_.Reset();
        }

        EXPECT_EQ(to_cr_.History().cols, 1) << "Expecting a single column.";
        EXPECT_FALSE(to_cr_.History().empty()) << "Sub history must not be empty";

        EXPECT_MAT_EQ(to_cr_.History(), to_.History().colRange(start_, end_)) << "Column range mismatch";

        to_.Advance();
    }
}

/** @brief test that manipulation of sub history reflects on global history
 */
TEST_F(SpikingHistoryColRangeTest, ManipulateSubHistory)
{
    const int N=30;
    Mat1b evidence(1, end_-start_);

    ASSERT_GT(start_, 0) << "This test is only applicable if the column range starts after the first column.";

    to_.Reset();

    for(int i=0; i<N; i++) {

        randu(evidence, 0, 2);

        to_cr_.Update(evidence > 0);

        if(i==10) {
            to_cr_.Reset();
        }

        EXPECT_MAT_EQ(to_cr_.History(), to_.History().colRange(start_, end_)) << "Column range mismatch";

        EXPECT_MAT_EQ(to_.History().colRange(0, start_), Mat1i::zeros(1, start_)) << "Remaining columns have been modified.";

        to_cr_.Advance();
    }
}

} // annonymous namespace for test cases
