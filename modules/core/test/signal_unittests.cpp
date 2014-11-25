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



