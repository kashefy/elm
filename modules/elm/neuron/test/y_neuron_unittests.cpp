/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/neuron/neuron.h"

#include "elm/core/exception.h"
#include "elm/ts/ts.h"

using cv::Mat1f;
using namespace elm;

namespace {

class YNeuronTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        pc_ = Mat1f(10, 1);
        for(int i=0; i<pc_.rows; i++) {

            pc_(i) = static_cast<float>(i%2);
        }

        frequency_ = 40;
        delta_t_msec_ = 1.f;

        to_ = YNeuron();
        to_.init(frequency_, delta_t_msec_);
    }

    Mat1f pc_;           ///< population codes
    float frequency_;
    float delta_t_msec_;

    YNeuron to_;    ///< test object
};

TEST_F(YNeuronTest, PopCodeZero)
{
    const int N = 1000;
    for(int i=0; i<N; i++) {

        EXPECT_EQ(to_.State(0), 0);
    }
}

TEST_F(YNeuronTest, InvalidState)
{
    EXPECT_THROW(to_.State(2.f), ExceptionNotImpl);
    EXPECT_THROW(to_.State(0.5f), ExceptionNotImpl);
    EXPECT_THROW(to_.State(0.1f), ExceptionNotImpl);

    EXPECT_NO_THROW(to_.State(-1.f));
    EXPECT_NO_THROW(to_.State(0.f));
    EXPECT_NO_THROW(to_.State(1.f));
}

TEST_F(YNeuronTest, FreqInf)
{
    const int N = 100;

    to_.init(1.f, 1000.f);

    for(int i=0; i<N; i++) {

        EXPECT_EQ(to_.State(1), 1);
        EXPECT_EQ(to_.State(0), 0);
    }
}

TEST_F(YNeuronTest, FreqZero)
{
    const int N = 100;

    to_.init(0.f, 1000.f);

    for(int i=0; i<N; i++) {

        EXPECT_EQ(to_.State(1), 0);
        EXPECT_EQ(to_.State(0), 0);
    }
}

TEST_F(YNeuronTest, DeltaTZero)
{
    const int N = 100;

    to_.init(1000.f, 0.f);

    for(int i=0; i<N; i++) {

        EXPECT_EQ(to_.State(1), 0);
        EXPECT_EQ(to_.State(0), 0);
    }
}

TEST_F(YNeuronTest, FreqZeroDeltaTZero)
{
    const int N = 100;
    to_.init(0.f, 0.f);

    for(int i=0; i<N; i++) {

        EXPECT_EQ(to_.State(1), 0);
        EXPECT_EQ(to_.State(0), 0);
    }
}

} // namespace

