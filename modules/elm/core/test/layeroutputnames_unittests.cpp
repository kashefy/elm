/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
//#include "elm/core/layeroutputnames.h"
#include "elm/core/layerionames.h"

#include "gtest/gtest.h"

#include <boost/optional.hpp>

#include "elm/core/exception.h"
#include "elm/core/inputname.h"

using namespace std;
using namespace elm;

template <class TType>
class LayerOutputNamesTypedTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = TType();
    }

    TType to_; ///< test object
};
typedef ::testing::Types<LayerOutputNames, LayerIONames> LayerOutputNamesTypes;

TYPED_TEST_CASE(LayerOutputNamesTypedTest, LayerOutputNamesTypes);

TYPED_TEST(LayerOutputNamesTypedTest, Output) {

    this->to_.Output("k1", "n1");
    this->to_.Output("k2", "n2");
    this->to_.Output("k2", "n22");

    EXPECT_EQ("n1", this->to_.Output("k1"));
    EXPECT_EQ("n22", this->to_.Output("k2"));
}

TYPED_TEST(LayerOutputNamesTypedTest, Output_WrongKey) {

    EXPECT_THROW(this->to_.Output("k1"), ExceptionKeyError);
    this->to_.Output("k1", "n1");
    EXPECT_EQ("n1", this->to_.Output("k1"));
}

TYPED_TEST(LayerOutputNamesTypedTest, OutputOpt) {

    EXPECT_THROW(this->to_.Output("k1"), ExceptionKeyError);
    this->to_.Output("k1", "n1");
    EXPECT_TRUE(this->to_.OutputOpt("k1") != 0);
    EXPECT_EQ("n1", this->to_.OutputOpt("k1").get());

    EXPECT_THROW(this->to_.Output("k2"), ExceptionKeyError);
    EXPECT_FALSE(this->to_.OutputOpt("k2"));
}
