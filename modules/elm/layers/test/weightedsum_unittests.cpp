/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/weightedsum.h"

#include "gtest/gtest.h"

#include <memory>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/ts/layer_assertions.h"
#include "elm/ts/mat_assertions.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(WeightedSum);

class WeightedSumTest : public ::testing::Test
{
public:
    static const string NAME_STIMULUS;
    static const string NAME_RESPONSE;

protected:
    virtual void SetUp()
    {
        to_.reset(new WeightedSum());

        config_ = LayerConfig();

        // params
        PTree params;
        params.add(WeightedSum::PARAM_A, -1.f);
        params.add(WeightedSum::PARAM_B, 2.f);
        config_.Params(params);

        // IO
        config_.Input(WeightedSum::KEY_INPUT_STIMULUS, NAME_STIMULUS);
        config_.Output(WeightedSum::KEY_OUTPUT_RESPONSE, NAME_RESPONSE);

        to_.reset(new WeightedSum(config_));
    }

    unique_ptr<base_Layer> to_; ///< test object
    LayerConfig config_;        ///< default config for tests
};
const string WeightedSumTest::NAME_STIMULUS = "in";
const string WeightedSumTest::NAME_RESPONSE = "out";

TEST_F(WeightedSumTest, Reset_EmptyConfig)
{
    EXPECT_THROW(to_->Reset(LayerConfig()), boost::property_tree::ptree_bad_path);
}

TEST_F(WeightedSumTest, Activate)
{
    Signal signal;
    // feed input into signal object
    EXPECT_FALSE(signal.Exists(NAME_STIMULUS));
    signal.Append(NAME_STIMULUS, Mat1f::ones(3, 2));
    EXPECT_TRUE(signal.Exists(NAME_STIMULUS));

    // compute response
    EXPECT_FALSE(signal.Exists(NAME_RESPONSE));
    to_->Activate(signal);
    to_->Response(signal);
    EXPECT_TRUE(signal.Exists(NAME_RESPONSE)) << "Resonse missing";

    // Check response dimensions
    Mat1f response = signal[NAME_RESPONSE][0];
    EXPECT_MAT_DIMS_EQ(response, Size(1, 3));

    // Check response values
    float a = config_.Params().get<float>(WeightedSum::PARAM_A);
    float b = config_.Params().get<float>(WeightedSum::PARAM_B);
    for(int r=0; r<response.rows; r++) {

        EXPECT_EQ(a+b, response(r));
    }
}

} // annonymous namespace for test cases and fixtures
