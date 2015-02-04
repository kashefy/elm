/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/layerconfig.h"

#include "gtest/gtest.h"

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

TEST_F(LayerIONamesTest, Input) {

    to_.Input("k1", "n1");
    to_.Input("k2", "n2");
    to_.Input("k2", "n22");

    EXPECT_EQ("n1", to_.Input("k1").to_string());
    EXPECT_EQ("n22", to_.Input("k2").to_string());
    EXPECT_THROW(to_.Output("k1"), ExceptionKeyError) << "Output mixing with input";
    EXPECT_THROW(to_.Output("k2"), ExceptionKeyError) << "Output mixing with input";
}

TEST_F(LayerIONamesTest, Input_WrongKey) {

    EXPECT_THROW(to_.Input("k1"), ExceptionKeyError);
    to_.Input("k1", "n1");
    EXPECT_EQ("n1", to_.Input("k1").to_string());
}

TEST_F(LayerIONamesTest, InputOpt) {

    EXPECT_THROW(to_.Input("k1"), ExceptionKeyError);
    EXPECT_FALSE(bool(to_.InputOpt("k1")));
    to_.Input("k1", "n1");
    EXPECT_TRUE(bool(to_.InputOpt("k1")));
    EXPECT_EQ("n1", to_.InputOpt("k1").get());

    EXPECT_THROW(to_.Input("k2"), ExceptionKeyError);
    EXPECT_FALSE(to_.InputOpt("k2"));
}

TEST_F(LayerIONamesTest, Output) {

    to_.Output("k1", "n1");
    to_.Output("k2", "n2");
    to_.Output("k2", "n22");

    EXPECT_EQ("n1", to_.Output("k1"));
    EXPECT_EQ("n22", to_.Output("k2"));
    EXPECT_THROW(to_.Input("k1"), ExceptionKeyError) << "Output mixing with input";
    EXPECT_THROW(to_.Input("k2"), ExceptionKeyError) << "Output mixing with input";
}

TEST_F(LayerIONamesTest, Output_WrongKey) {

    EXPECT_THROW(to_.Output("k1"), ExceptionKeyError);
    to_.Output("k1", "n1");
    EXPECT_EQ("n1", to_.Output("k1"));
}

TEST_F(LayerIONamesTest, OutputOpt) {

    EXPECT_THROW(to_.Output("k1"), ExceptionKeyError);
    to_.Output("k1", "n1");
    EXPECT_TRUE(to_.OutputOpt("k1") != 0);
    EXPECT_EQ("n1", to_.OutputOpt("k1").get());

    EXPECT_THROW(to_.Output("k2"), ExceptionKeyError);
    EXPECT_FALSE(to_.OutputOpt("k2"));
}
