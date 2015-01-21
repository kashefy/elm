#include "sem/eval/reversecorr.h"

#include "sem/core/exception.h"
#include "sem/ts/ts.h"

using namespace cv;
using namespace sem;

class STATest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = STA();
    }

    STA to_;    ///< test object
};

TEST_F(STATest, EmptyInput)
{
    EXPECT_THROW(to_.Add(Mat()), ExceptionBadDims);
    EXPECT_THROW(to_.Add(std::vector<float>()), ExceptionBadDims);
}

TEST_F(STATest, NothingAdded)
{
    EXPECT_TRUE(to_.Compute().empty());
}

TEST_F(STATest, Zeros)
{
    const int N=10;
    for(int i=0; i<N; i++) {

        to_.Add(Mat1f::zeros(3, 3));
    }

    EXPECT_MAT_EQ(to_.Compute(), Mat1f::zeros(3, 3));
}

TEST_F(STATest, SpikeTriggeredAvg)
{
    const int N=100;
    for(int i=0; i<N; i++) {

        Mat1f in(1, 3);
        in(0) = 0;
        in(1) = i%2==0? 1 : 0;
        in(2) = 1;
        to_.Add(in);
    }

    float data[3] = {0.f, 0.5f, 1.f};
    Mat1f expected_sta(1, 3, data);
    EXPECT_MAT_EQ(to_.Compute(), expected_sta);
    EXPECT_MAT_EQ(to_.Compute(), expected_sta);
}

TEST_F(STATest, SpikeTriggeredAvg_int)
{
    const int N=100;
    for(int i=0; i<N; i++) {

        Mat1i in(1, 3);
        in(0) = 0;
        in(1) = i%2==0? 1 : 0;
        in(2) = 1;
        to_.Add(in);
    }

    float data[3] = {0.f, 0.5f, 1.f};
    Mat1f expected_sta(1, 3, data);
    EXPECT_MAT_EQ(to_.Compute(), expected_sta);
    EXPECT_MAT_EQ(to_.Compute(), expected_sta);
}

TEST_F(STATest, SpikeTriggeredAvg_vecf)
{
    const int N=100;
    for(int i=0; i<N; i++) {

        std::vector<float> in;
        in.push_back(0);
        in.push_back(i%2==0? 1 : 0);
        in.push_back(1);
        to_.Add(in);
    }

    float data[3] = {0.f, 0.5f, 1.f};
    Mat1f expected_sta(1, 3, data);
    EXPECT_MAT_EQ(to_.Compute(), expected_sta);
    EXPECT_MAT_EQ(to_.Compute(), expected_sta);
}

TEST_F(STATest, NonZero)
{
    const int N=10;
    for(int i=0; i<N; i++) {

        Mat1f in(1, 3);
        in(0) = 1;
        in(1) = 0.5;
        in(2) = 5;
        to_.Add(in);
    }

    EXPECT_MAT_EQ(to_.Compute(), Mat1f::ones(1, 3));
}

TEST_F(STATest, DimChange)
{
    to_.Add(Mat1f(2, 2, 1.f));
    EXPECT_THROW(to_.Add(Mat1f(1, 4, 1.f)), ExceptionBadDims);
    EXPECT_THROW(to_.Add(Mat1f(4, 1, 1.f)), ExceptionBadDims);
    EXPECT_THROW(to_.Add(Mat1f()), ExceptionBadDims);
    EXPECT_MAT_DIMS_EQ(to_.Compute(), Size(2, 2)) << "STA dims don't match that of initial input.";
}
