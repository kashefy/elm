#include "neuron/spikinghistory.h"

#include "ts/ts.h"

class SpikingHistoryTest : public testing::Test
{
protected:
    SpikingHistoryTest()
        : to_(5, 3)
    {
    }

    virtual void SetUp()
    {
        len_ = 5;
        dims_ = 3;
        to_ = SpikingHistory(dims_, len_);
    }

    SpikingHistory to_; ///< test object
    int len_;           ///< history length
    int dims_;          ///< dimensions
};

TEST_F(SpikingHistoryTest, InitialHistory)
{
    MatI h = to_.History();
    EXPECT_MAT_EQ(h,  MatI::zeros(1, dims_)+len_) << "History mismatch";
}

TEST_F(SpikingHistoryTest, Reset)
{
    MatI expected = MatI::zeros(1, dims_)+len_;
    for(int i=0; i<len_*3; i++) {

        EXPECT_MAT_EQ(to_.History(), expected) << "History mismatch at i=" << i;
        to_.Reset();
    }
}

TEST_F(SpikingHistoryTest, Advance)
{
    MatI expected = MatI::zeros(1, dims_)+len_;
    for(int i=0; i<len_*3; i++) {

        to_.Advance();
        if(i < len_) {

            expected -= 1;
            EXPECT_MAT_EQ(to_.History(), expected) << "History mismatch at i=" << i;
        }
        else {

            EXPECT_MAT_EQ(to_.History(), MatI::zeros(1, dims_)) << "History mismatch at i=" << i;
        }
    }
}

TEST_F(SpikingHistoryTest, AdvanceThenReset)
{
    to_.Advance();
    to_.Advance();

    MatI expected = MatI::zeros(1, dims_)+len_;

    EXPECT_MAT_EQ(to_.History(), expected-2) << "History mismatch before Reset";

    to_.Reset();

    EXPECT_MAT_EQ(to_.History(), expected) << "History mismatch after Reset";
}

TEST_F(SpikingHistoryTest, Recent)
{
    for(int i=0; i<len_*3; i++) {

        MatI h = to_.History();
        for(int index=0; index<h.cols; index++) {

            if(i < len_) { EXPECT_TRUE  (to_.Recent(index)); }
            else         { EXPECT_FALSE (to_.Recent(index)); }
        }
        to_.Advance();
    }
}

TEST_F(SpikingHistoryTest, Update)
{
    cv::RNG rng;
    int n = 0;
    while(n++ < len_*3) {

        to_.Advance();
        int index = int(rng.uniform(1, dims_));

        MatI spikes = MatI::zeros(1, dims_);
        spikes(index) = 1;
        cv::Mat mask = spikes != 0;

        to_.Update(mask);

        EXPECT_TRUE(to_.Recent(index));

        if(n-1 < len_-1)    { EXPECT_TRUE  (to_.Recent(0)); }
        else                { EXPECT_FALSE (to_.Recent(0)); }
    };
}
