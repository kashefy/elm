/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
//#include "elm/core/layerinputnames.h"
#include "elm/core/layerionames.h"

#include "gtest/gtest.h"

#include <boost/optional.hpp>

#include "elm/core/exception.h"
#include "elm/core/inputname.h"

using namespace std;
using namespace elm;

template <class TType>
class LayerInputNamesTypedTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = TType();
    }

    TType to_; ///< test object
};
typedef ::testing::Types<LayerInputNames,LayerIONames> LayerInputNamesTypes;

TYPED_TEST_CASE(LayerInputNamesTypedTest, LayerInputNamesTypes);

TYPED_TEST(LayerInputNamesTypedTest, Input) {

    this->to_.Input("k1", "n1");
    this->to_.Input("k2", "n2");
    this->to_.Input("k2", "n22");

    EXPECT_EQ("n1", this->to_.Input("k1").to_string());
    EXPECT_EQ("n22", this->to_.Input("k2").to_string());
}

TYPED_TEST(LayerInputNamesTypedTest, Input_WrongKey) {

    EXPECT_THROW(this->to_.Input("k1"), ExceptionKeyError);
    this->to_.Input("k1", "n1");
    EXPECT_EQ("n1", this->to_.Input("k1").to_string());
}

TYPED_TEST(LayerInputNamesTypedTest, InputOpt) {

    EXPECT_THROW(this->to_.Input("k1"), ExceptionKeyError);
    EXPECT_FALSE(bool(this->to_.InputOpt("k1")));
    this->to_.Input("k1", "n1");
    EXPECT_TRUE(bool(this->to_.InputOpt("k1")));
    EXPECT_EQ("n1", this->to_.InputOpt("k1").get());

    EXPECT_THROW(this->to_.Input("k2"), ExceptionKeyError);
    EXPECT_FALSE(this->to_.InputOpt("k2"));
}

TYPED_TEST(LayerInputNamesTypedTest, InputMap) {

    this->to_.Input("k1", "n1");
    this->to_.Input("k2", "n2");
    this->to_.Input("k2", "n22");

    MapSS inputs = this->to_.InputMap();
    EXPECT_EQ(static_cast<size_t>(2), inputs.size());
    EXPECT_EQ(inputs["k1"], "n1");
    EXPECT_EQ(inputs["k2"], "n22");
}
