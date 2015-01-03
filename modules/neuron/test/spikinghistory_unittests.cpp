#include "neuron/spikinghistory.h"

#include "ts/ts.h"

using namespace cv;

namespace {

class SpikingHistoryTest : public ::testing::Test
{
protected:
    SpikingHistoryTest()
        : to_(5, 3)
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
        std::cout<<index<<std::endl;
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

} // annonymous namespace for test cases
