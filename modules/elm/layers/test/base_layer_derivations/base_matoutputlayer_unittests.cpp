/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/base_layer_derivations/base_matoutputlayer.h"

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

/** @brief class deriving from base_MatOutputLayer for test purposes
  */
class DummyMatOutputLayer : public base_MatOutputLayer
{
public:
    static const string KEY_INPUT_M;

    virtual void Clear() {}

    virtual void Reconfigure(const LayerConfig &config) {}

    virtual void IONames(const LayerIONames &io) {

        name_in_ = io.Input(KEY_INPUT_M);
        base_MatOutputLayer::IONames(io);
    }

    virtual void Activate(const Signal &signal) {

        m_ = signal.MostRecentMat(name_in_)*2.f;
    }

    DummyMatOutputLayer() {}

protected:
    string name_in_;
};
const string DummyMatOutputLayer::KEY_INPUT_M = "m_in";

class MatOutputLayerTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.reset(new DummyMatOutputLayer());

        LayerIONames io;
        io.Input(DummyMatOutputLayer::KEY_INPUT_M, NAME_IN_M);
        io.Output(DummyMatOutputLayer::KEY_OUTPUT_M, NAME_OUT_M);

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

TEST_F(MatOutputLayerTest, Sanity)
{
    EXPECT_EQ(base_MatOutputLayer::KEY_OUTPUT_M,
              DummyMatOutputLayer::KEY_OUTPUT_M);
}

TEST_F(MatOutputLayerTest, ActivateAndResponse)
{
    to_->Activate(sig_);
    to_->Response(sig_);

    EXPECT_MAT_EQ(sig_.MostRecentMat(NAME_IN_M)*2,
                  sig_.MostRecentMat(NAME_OUT_M));
}

} // annonymous namespace for test cases and test fixtures
