#include "core/area.h"

#include "gtest/gtest.h"
#include "core/exception.h"

class TrapzTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = Trapz();
    }

    Trapz to_; ///< test object
};

TEST_F(TrapzTest, BadDims) {

    MatF x;
    EXPECT_THROW(to_(x, MatF::zeros(1, 2)), ExceptionBadDims);
    EXPECT_THROW(to_(MatF::zeros(1, 2), x), ExceptionBadDims);

    EXPECT_THROW(to_(x, x), ExceptionBadDims);

    EXPECT_THROW(to_(MatF::zeros(1, 1), MatF::zeros(1, 2)), ExceptionBadDims);
    EXPECT_THROW(to_(MatF::zeros(1, 2), MatF::zeros(1, 1)), ExceptionBadDims);

    EXPECT_NO_THROW(to_(MatF::zeros(1, 2), MatF::zeros(1, 2)));
    EXPECT_NO_THROW(to_(MatF::zeros(1, 2), MatF::zeros(2, 1)));
    EXPECT_NO_THROW(to_(MatF::zeros(2, 1), MatF::zeros(2, 1)));
    EXPECT_NO_THROW(to_(MatF::zeros(2, 1), MatF::zeros(1, 2)));

    EXPECT_NO_THROW(to_(MatF::zeros(10, 1), MatF::zeros(10, 1)));
}

TEST_F(TrapzTest, Trapezoidal) {

    float values[2] = {0.f, 1.f};
    MatF x(1, 2, values);
    MatF y = x.clone();

    EXPECT_FLOAT_EQ(to_(x, y), 0.5f);
    EXPECT_FLOAT_EQ(to_(x, MatF::zeros(1, 2)), 0.f);
    EXPECT_FLOAT_EQ(to_(x, MatF::ones(1, 2)), 1.f);
}

