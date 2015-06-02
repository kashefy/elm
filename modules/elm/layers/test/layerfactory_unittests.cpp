/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layerfactory.h"

#include "gtest/gtest.h"

#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/saliencyitti.h" ///< to have layer dervied classes to test with
#include "elm/layers/weightedsum.h"  ///< to have layer dervied classes to test with
#include "elm/ts/mat_assertions.h"

using std::string;
using namespace elm;

namespace {

/**
 * @brief class or testing LayerFactory's static methods
 */
class LayerFactoryStaticTest : public ::testing::Test
{
protected:
    static void SetUpTestCase() {

        LayerFactory();
    }
};

TEST_F(LayerFactoryStaticTest, CreateLayerPtrShared)
{
    {
        LayerShared ptr = LayerFactory::CreateShared("LayerY");
        EXPECT_TRUE(bool(ptr));
    }
    {
        LayerShared ptr = LayerFactory::CreateShared("WeightedSum");
        EXPECT_TRUE(bool(ptr));
    }
}

TEST_F(LayerFactoryStaticTest, CreateLayerPtrShared_WrongType)
{
    EXPECT_THROW(LayerFactory::CreateShared("Blahbla"), ExceptionTypeError);
}

TEST_F(LayerFactoryStaticTest, CreateLayerPtrShared_UniqueInstancesSameType)
{
    const string TYPE="WeightedSum";

    LayerShared ptr1 = LayerFactory::CreateShared(TYPE);
    LayerShared ptr2 = LayerFactory::CreateShared(TYPE);

    EXPECT_NE(ptr1, ptr2);
}

TEST_F(LayerFactoryStaticTest, CreateLayerPtrShared_WithConfig)
{
    PTree params;
    params.put(WeightedSum::PARAM_A, 0.2f);
    params.put(WeightedSum::PARAM_B, 0.3f);
    LayerConfig config;
    config.Params(params);

    const string NAME_STIMULUS = "in";
    const string NAME_RESPONSE = "out";

    config.Input(WeightedSum::KEY_INPUT_STIMULUS, NAME_STIMULUS);
    config.Output(WeightedSum::KEY_OUTPUT_RESPONSE, NAME_RESPONSE);

    LayerShared ptr = LayerFactory::CreateShared("WeightedSum", config, config);
    EXPECT_TRUE(bool(ptr));

    // populate signal with input feature
    Signal signal;
    signal.Append("in", cv::Mat1f::ones(1, 2));
    EXPECT_TRUE(signal.Exists(NAME_STIMULUS));
    EXPECT_FALSE(signal.Exists(NAME_RESPONSE));

    // apply signal to layer
    ptr->Activate(signal);
    ptr->Response(signal);

    // check response
    EXPECT_TRUE(signal.Exists(NAME_RESPONSE));
    EXPECT_MAT_DIMS_EQ(signal.MostRecentMat1f(NAME_RESPONSE), cv::Size2i(1, 1));
    EXPECT_FLOAT_EQ(signal.MostRecentMat1f(NAME_RESPONSE).at<float>(0), 0.5f);
}

TEST_F(LayerFactoryStaticTest, Exists)
{
    EXPECT_FALSE(LayerFactory::Exists("Blahbla"));
    EXPECT_TRUE(LayerFactory::Exists("WeightedSum"));
}

TEST_F(LayerFactoryStaticTest, InitLayerPtrShared_WithConfig)
{
    PTree params;
    params.put(WeightedSum::PARAM_A, 0.2f);
    params.put(WeightedSum::PARAM_B, 0.3f);
    LayerConfig config;
    config.Params(params);

    const string NAME_STIMULUS = "in";
    const string NAME_RESPONSE = "out";

    config.Input(WeightedSum::KEY_INPUT_STIMULUS, NAME_STIMULUS);
    config.Output(WeightedSum::KEY_OUTPUT_RESPONSE, NAME_RESPONSE);

    LayerShared ptr(new WeightedSum);
    LayerFactory::Init(ptr, config, config);
    EXPECT_TRUE(bool(ptr));

    // populate signal with input feature
    Signal signal;
    signal.Append("in", cv::Mat1f::ones(1, 2));
    EXPECT_TRUE(signal.Exists(NAME_STIMULUS));
    EXPECT_FALSE(signal.Exists(NAME_RESPONSE));

    // apply signal to layer
    ptr->Activate(signal);
    ptr->Response(signal);

    // check response
    EXPECT_TRUE(signal.Exists(NAME_RESPONSE));
    EXPECT_MAT_DIMS_EQ(signal.MostRecentMat1f(NAME_RESPONSE), cv::Size2i(1, 1));
    EXPECT_FLOAT_EQ(signal.MostRecentMat1f(NAME_RESPONSE).at<float>(0), 0.5f);
}

TEST_F(LayerFactoryStaticTest, InitLayerPtrShared_WithConfig_invalid_ptr)
{
    PTree params;
    params.put(WeightedSum::PARAM_A, 0.2f);
    params.put(WeightedSum::PARAM_B, 0.3f);
    LayerConfig config;
    config.Params(params);

    const string NAME_STIMULUS = "in";
    const string NAME_RESPONSE = "out";

    config.Input(WeightedSum::KEY_INPUT_STIMULUS, NAME_STIMULUS);
    config.Output(WeightedSum::KEY_OUTPUT_RESPONSE, NAME_RESPONSE);

    LayerShared ptr;
    ASSERT_FALSE(bool(ptr));
    EXPECT_THROW(LayerFactory::Init(ptr, config, config), ExceptionValueError);

    ptr.reset(new WeightedSum);
    EXPECT_TRUE(bool(ptr));
    EXPECT_NO_THROW(LayerFactory::Init(ptr, config, config));
}

class LayerFactoryTest : public ::testing::Test
{
};

TEST_F(LayerFactoryStaticTest, Constructor)
{
    EXPECT_NO_THROW(LayerFactory to);
}

} // annonymous namespace
