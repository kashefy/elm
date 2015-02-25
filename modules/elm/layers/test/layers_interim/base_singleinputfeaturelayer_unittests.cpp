/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layers_interim/base_singleinputfeaturelayer.h"

#include <memory>

#include "gtest/gtest.h"

#include "elm/core/layerionames.h"
#include "elm/core/signal.h"
#include "elm/ts/layer_assertions.h"

using namespace std;
using namespace cv;
using namespace elm;

namespace {

const string NAME_IN_M  = "in";
const string NAME_OUT_M = "out";

/** @brief class deriving from intermediate base_SingleInputFeatureLayer for test purposes
  */
class DummySingleInputFeatureLayer : public base_SingleInputFeatureLayer
{
public:
    static const string KEY_OUTPUT_M;

    void Clear() {}

    void Reconfigure(const LayerConfig &config) {}

    void OutputNames(const LayerOutputNames &io) {

        name_out_ = io.Output(KEY_OUTPUT_M);
    }

    void Activate(const Signal &signal) {

        m_ = signal.MostRecentMat1f(name_input_)*2.f;
    }

    void Response(Signal &signal) {

        signal.Append(name_out_, m_);
    }

    DummySingleInputFeatureLayer() {}

protected:
    string name_out_;

    Mat1f m_;
};
const string DummySingleInputFeatureLayer::KEY_OUTPUT_M = "m_out";

class SingleInputFeatureLayerTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.reset(new DummySingleInputFeatureLayer());

        LayerIONames io;
        io.Input(DummySingleInputFeatureLayer::KEY_INPUT_STIMULUS, NAME_IN_M);
        io.Output(DummySingleInputFeatureLayer::KEY_OUTPUT_M, NAME_OUT_M);

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

TEST_F(SingleInputFeatureLayerTest, Sanity)
{
    EXPECT_EQ(base_SingleInputFeatureLayer::KEY_INPUT_STIMULUS,
              DummySingleInputFeatureLayer::KEY_INPUT_STIMULUS);
}

TEST_F(SingleInputFeatureLayerTest, ActivateAndResponse)
{
    to_->Activate(sig_);
    to_->Response(sig_);

    EXPECT_MAT_EQ(sig_.MostRecentMat1f(NAME_IN_M)*2,
                  sig_.MostRecentMat1f(NAME_OUT_M));
}

} // annonymous namespace for test cases and test fixtures
