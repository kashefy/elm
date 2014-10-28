#include "encoding/intensitycontrast.h"

#include "core/exception.h"
#include "ts/ts.h"

class IntensityConstrastRetGangTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        radius_ = 13;
        scale_ = 1.f;
        to_ = RetGang();
        to_.Init(radius_, scale_);
    }

    RetGang to_;    ///< test object
    int radius_;
    int scale_;
};

TEST_F(IntensityConstrastRetGangTest, Init)
{
    EXPECT_THROW(to_.Init(0, 1.f), ExceptionBadDims);
    EXPECT_THROW(to_.Init(-1, 1.f), ExceptionBadDims);
    EXPECT_THROW(to_.Init(-20, 1.f), ExceptionBadDims);
}
