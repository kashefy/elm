#include "encoding/populationcode.h"

#include "ts/ts.h"

namespace {

class MutexPopulationCodeTest : public testing::Test
{
protected:
    void virtual SetUp()
    {
        in_ = MatF::ones(3, 1);
        to_.State(in_);
    }

    MutexPopulationCode to_; ///< test object
    MatF in_;
};

TEST_F(MutexPopulationCodeTest, PopCode_dims) {

    MatF pc = to_.PopCode();
    EXPECT_MAT_DIMS_EQ( pc, MatF(in_.rows*2, 1) );
}

TEST_F(MutexPopulationCodeTest, PopCode_ones) {

    MatF pc = to_.PopCode();
    for(int i=0; i<pc.rows; i++) {

        EXPECT_FLOAT_EQ(pc(i), i%2==0? 0.f : 1.f);
    }
}

TEST_F(MutexPopulationCodeTest, PopCode_zeros) {

    in_ = MatF::zeros(in_.rows, in_.cols);
    to_.State(in_);
    MatF pc = to_.PopCode();

    for(int i=0; i<pc.rows; i++) {

        EXPECT_FLOAT_EQ(pc(i), i%2==0? 1.f : 0.f);
    }
}

} // namespace
