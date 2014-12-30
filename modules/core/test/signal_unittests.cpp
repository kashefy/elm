#include "core/signal.h"

#include "core/exception.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;
using namespace sem;

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

TEST_F(SignalTest, Constructor)
{
    EXPECT_NO_THROW(Signal());
}

TEST_F(SignalTest, FeatureNames)
{
    EXPECT_EMPTY(Signal().FeatureNames()) << "Signal should initially be empty";

    VecS feature_names = to_.FeatureNames();
    EXPECT_EQ(size_t(2), feature_names.size());

    // linear search to see if feature names were added
    for(uint i=0; i<in_.size(); i++) {

        bool found = false;
        for(uint j=0; j<feature_names.size() && !found; j++) {

            found = feature_names[j] == in_[i];
        }
        EXPECT_TRUE(found) << "Could not find added name " << in_[i];
    }
}

TEST_F(SignalTest, GetFeatures)
{
    EXPECT_THROW(to_["wrong"], ExceptionKeyError);
    EXPECT_THROW(to_["Foo"], ExceptionKeyError);
    EXPECT_THROW(to_["FOO"], ExceptionKeyError);

    EXPECT_SIZE(2, to_["foo"]);
    EXPECT_SIZE(1, to_["bar"]);

    EXPECT_MAT_EQ(to_["bar"][0], Mat1f());
}

TEST_F(SignalTest, MostRecent)
{
    EXPECT_THROW(to_.MostRecent("wrong"), ExceptionKeyError);
    EXPECT_THROW(to_.MostRecent("Foo"), ExceptionKeyError);
    EXPECT_THROW(to_.MostRecent("FOO"), ExceptionKeyError);


    for(int i=0; i<3; i++) {

        EXPECT_SIZE(2+i, to_["foo"]);
        to_.Append("foo", Mat1f::ones(1, 1)+i);
        EXPECT_SIZE(2+i+1, to_["foo"]);
        EXPECT_MAT_EQ(to_.MostRecent("foo"), Mat1f::ones(1, 1)+i);
    }
}


