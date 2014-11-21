#include "core/layerconfig.h"

#include "gtest/gtest.h"
#include "core/exception.h"

using namespace std;

class LayerConfigTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = LayerConfig();
    }

    LayerConfig to_; ///< test object
};

TEST_F(LayerConfigTest, Input) {

    to_.Input("k1", "n1");
    to_.Input("k2", "n2");
    to_.Input("k2", "n22");

    EXPECT_EQ("n1", to_.Input("k1"));
    EXPECT_EQ("n22", to_.Input("k2"));
}

TEST_F(LayerConfigTest, Input_WrongKey) {

    EXPECT_THROW(to_.Input("k1"), ExceptionKeyError);
    to_.Input("k1", "n1");
    EXPECT_EQ("n1", to_.Input("k1"));
}

TEST_F(LayerConfigTest, Output) {

    to_.Output("k1", "n1");
    to_.Output("k2", "n2");
    to_.Output("k2", "n22");

    EXPECT_EQ("n1", to_.Output("k1"));
    EXPECT_EQ("n22", to_.Output("k2"));
}

TEST_F(LayerConfigTest, Output_WrongKey) {

    EXPECT_THROW(to_.Output("k1"), ExceptionKeyError);
    to_.Output("k1", "n1");
    EXPECT_EQ("n1", to_.Output("k1"));
}

TEST_F(LayerConfigTest, Params) {

    PTree p1, p2;
    p1.put("k1", "n1");
    p1.put<int>("k2", 2);
    p1.put<int>("k2", 22);

    to_.Params(p1);

    p2 = to_.Params();

    EXPECT_EQ("n1", p2.get<string>("k1"));
    EXPECT_EQ(22, p2.get<int>("k2"));
}
