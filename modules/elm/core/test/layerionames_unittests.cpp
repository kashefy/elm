/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/layerionames.h"

#include "gtest/gtest.h"

#include <boost/optional.hpp>

#include "elm/core/exception.h"
#include "elm/core/inputname.h"

using namespace std;
using namespace elm;

class LayerIONamesTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        to_ = LayerIONames();
    }

    LayerIONames to_; ///< test object
};

/**
 * @brief Test that input is not mixed as output
 */
TEST_F(LayerIONamesTest, Input) {

    this->to_.Input("k1", "n1");
    this->to_.Input("k2", "n2");
    this->to_.Input("k2", "n22");

    EXPECT_EQ("n1", this->to_.Input("k1").to_string());
    EXPECT_EQ("n22", this->to_.Input("k2").to_string());
    EXPECT_THROW(this->to_.Output("k1"), ExceptionKeyError) << "Output mixing with input";
    EXPECT_THROW(this->to_.Output("k2"), ExceptionKeyError) << "Output mixing with input";
}

/**
 * @brief Test that output is not mixed as input
 */
TEST_F(LayerIONamesTest, Output) {

    to_.Output("k1", "n1");
    to_.Output("k2", "n2");
    to_.Output("k2", "n22");

    EXPECT_EQ("n1", to_.Output("k1"));
    EXPECT_EQ("n22", to_.Output("k2"));
    EXPECT_THROW(to_.Input("k1"), ExceptionKeyError) << "Output mixing with input";
    EXPECT_THROW(to_.Input("k2"), ExceptionKeyError) << "Output mixing with input";
}
