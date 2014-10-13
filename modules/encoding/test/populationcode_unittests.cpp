#include "encoding/populationcode.h"

#include "ts/ts.h"

using cv::Mat1f;

namespace {

class MutexPopulationCodeTest : public testing::Test
{
protected:
    void virtual SetUp()
    {
        in_ = Mat1f::ones(3, 1);
        to_.State(in_);
    }

    MutexPopulationCode to_; ///< test object
    Mat1f in_;
};

TEST_F(MutexPopulationCodeTest, PopCode_dims) {

    Mat1f pc = to_.PopCode();
    EXPECT_MAT_DIMS_EQ( pc, Mat1f(in_.rows*2, 1) );
}

TEST_F(MutexPopulationCodeTest, PopCode_ones) {

    Mat1f pc = to_.PopCode();
    for(int i=0; i<pc.rows; i++) {

        EXPECT_FLOAT_EQ(pc(i), i%2==0? 0.f : 1.f);
    }
}

TEST_F(MutexPopulationCodeTest, PopCode_zeros) {

    in_ = Mat1f::zeros(in_.rows, in_.cols);
    to_.State(in_);
    Mat1f pc = to_.PopCode();

    for(int i=0; i<pc.rows; i++) {

        EXPECT_FLOAT_EQ(pc(i), i%2==0? 1.f : 0.f);
    }
}

} // namespace
