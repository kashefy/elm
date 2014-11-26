#include "core/signal.h"

#include "core/exception.h"
#include "ts/ts.h"

using namespace std;
using namespace cv;

class SignalTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = Signal();
    }

    Signal to_; ///<test object
};

TEST_F(SignalTest, Constructor)
{
    EXPECT_NO_THROW(Signal());
}

TEST_F(SignalTest, FeatureNames)
{
    EXPECT_EMPTY(to_.FeatureNames()) << "Signal should initially be empty";

    VecS in;
    in.push_back("foo");
    in.push_back("bar");

    for(uint i=0; i<in.size(); i++) {

        to_.Append(in[i], Mat1f());
    }

    VecS feature_names = to_.FeatureNames();
    EXPECT_EQ(2, feature_names.size());

    // linear search to see if feature names were added
    for(uint i=0; i<in.size(); i++) {

        bool found = false;
        for(uint j=0; j<feature_names.size() && !found; j++) {

            found = feature_names[j] == in[i];
        }
        EXPECT_TRUE(found) << "Could not find added name " << in[i];
    }
}


