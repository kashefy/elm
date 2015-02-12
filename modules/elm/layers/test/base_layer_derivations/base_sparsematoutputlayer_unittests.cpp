/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/base_layer_derivations/base_sparsematoutputlayer.h"

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

/** @brief class deriving from base_SparseMatOutputLayer for test purposes
  */
class DummySparseMatOutputLayer : public base_SparseMatOutputLayer
{
public:
    static const string KEY_INPUT_M;

    void Clear() {}

    void Reconfigure(const LayerConfig &config) {}

    void InputNames(const LayerInputNames &io) {

        name_in_ = io.Input(KEY_INPUT_M);
    }

    void Activate(const Signal &signal) {

        m_ = signal.MostRecentMat1f(name_in_)*2.f;
    }

    DummySparseMatOutputLayer() {}

protected:
    string name_in_;
};
const string DummySparseMatOutputLayer::KEY_INPUT_M = "m_in";

class SparseMatOutputLayerTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.reset(new DummySparseMatOutputLayer());

        LayerIONames io;
        io.Input(DummySparseMatOutputLayer::KEY_INPUT_M, NAME_IN_M);
        io.Output(DummySparseMatOutputLayer::KEY_OUTPUT_RESPONSE, NAME_OUT_M);

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

TEST_F(SparseMatOutputLayerTest, Sanity)
{
    EXPECT_EQ(base_SparseMatOutputLayer::KEY_OUTPUT_RESPONSE,
              DummySparseMatOutputLayer::KEY_OUTPUT_RESPONSE);
}

TEST_F(SparseMatOutputLayerTest, ActivateAndResponse)
{
    to_->Activate(sig_);
    to_->Response(sig_);

    EXPECT_MAT_EQ(sig_.MostRecentMat1f(NAME_IN_M)*2,
                  sig_.MostRecentMat1f(NAME_OUT_M));

    Mat1f dense;
    sig_.MostRecent(NAME_IN_M).get<SparseMat1f>().convertTo(dense, CV_32FC1);

    EXPECT_MAT_EQ(dense*2,
                  sig_.MostRecent(NAME_OUT_M).get<SparseMat1f>());
}

} // annonymous namespace for test cases and test fixtures
