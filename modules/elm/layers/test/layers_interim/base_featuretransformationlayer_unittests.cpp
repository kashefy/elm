/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layers_interim/base_featuretransformationlayer.h"

#include <memory>

#include "gtest/gtest.h"

#include "elm/core/layerionames.h"
#include "elm/core/signal.h"
#include "elm/ts/layer_assertions.h"
#include "elm/ts/mat_assertions.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

const string NAME_IN_M  = "in";
const string NAME_OUT_M = "out";

/** @brief class deriving from intermediate base_FeatureTransformationLayer for test purposes
  */
class DummyFeatureTransformationLayer : public base_FeatureTransformationLayer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {}

    void Activate(const Signal &signal) {

        m_ = signal.MostRecentMat1f(name_input_)*2.f;
    }

    DummyFeatureTransformationLayer() {}
};

class FeatureTransformationLayerTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.reset(new DummyFeatureTransformationLayer());

        LayerIONames io;
        io.Input(DummyFeatureTransformationLayer::KEY_INPUT_STIMULUS, NAME_IN_M);
        io.Output(DummyFeatureTransformationLayer::KEY_OUTPUT_RESPONSE, NAME_OUT_M);

        to_->IONames(io);

        sig_.Append(NAME_IN_M, Mat1f(3, 4, 1.f));

    }

    virtual void TearDown()
    {
        sig_.Clear();
    }

    // members:
    shared_ptr<base_Layer> to_; ///< test object
    Signal sig_;
};

TEST_F(FeatureTransformationLayerTest, Sanity)
{
    EXPECT_EQ(base_FeatureTransformationLayer::KEY_INPUT_STIMULUS,
              DummyFeatureTransformationLayer::KEY_INPUT_STIMULUS);

    EXPECT_EQ(base_FeatureTransformationLayer::KEY_OUTPUT_RESPONSE,
              DummyFeatureTransformationLayer::KEY_OUTPUT_RESPONSE);

}

TEST_F(FeatureTransformationLayerTest, ActivateAndResponse)
{
    to_->Activate(sig_);
    to_->Response(sig_);

    EXPECT_MAT_EQ(sig_.MostRecentMat1f(NAME_IN_M)*2,
                  sig_.MostRecentMat1f(NAME_OUT_M));
}

} // annonymous namespace for test cases and test fixtures
