#include "elm/core/signal.h"

#include "elm/core/exception.h"
#include "elm/ts/signal_tp_.h"
#include "elm/ts/ts.h"

using namespace std;
using namespace cv;
using namespace elm;

INSTANTIATE_TYPED_TEST_CASE_P(SignalTP_FtData_Tests, Signal_TP_, FeatureData);

template<>
std::vector<FeatureData> FtV_<FeatureData>::values{FeatureData(Mat1f(1, 1, 0.f)),
            FeatureData(Mat1f(1, 1, 1.f)),
            FeatureData(Mat1f(1, 1, 100.f))};

/**
 * @brief Additional coverage for main Signal class with FeatureData template
 */
class SignalTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = Signal();

        VecS in;
        in.push_back("foo");
        in.push_back("bar");

        for(uint i=0; i<in.size(); i++) {

            to_.Append(in[i], Mat1f());
        }

        to_.Append(in[0], Mat1f());
    }

    Signal to_; ///<test object
    VecS in_;   ///< test feature names
};

TEST_F(SignalTest, GetFeatures)
{
    EXPECT_THROW(to_["wrong"], ExceptionKeyError);
    EXPECT_THROW(to_["Foo"], ExceptionKeyError);
    EXPECT_THROW(to_["FOO"], ExceptionKeyError);

    EXPECT_SIZE(2, to_["foo"]);
    EXPECT_SIZE(1, to_["bar"]);

    EXPECT_MAT_EQ(to_["bar"][0], Mat1f());
}

TEST_F(SignalTest, MostRecentMat)
{
    EXPECT_THROW(to_.MostRecentMat("wrong"), ExceptionKeyError);
    EXPECT_THROW(to_.MostRecentMat("Foo"), ExceptionKeyError);
    EXPECT_THROW(to_.MostRecentMat("FOO"), ExceptionKeyError);

    for(int i=0; i<3; i++) {

        EXPECT_SIZE(2+i, to_["foo"]);
        to_.Append("foo", Mat1f::ones(1, 1)+i);
        EXPECT_SIZE(2+i+1, to_["foo"]);
        EXPECT_MAT_EQ(to_.MostRecentMat("foo"), Mat1f::ones(1, 1)+i);
    }
}


