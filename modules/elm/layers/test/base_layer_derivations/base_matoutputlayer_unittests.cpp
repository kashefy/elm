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

        m_ = signal.MostRecentMat(name_in_);
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
    }

    shared_ptr<base_Layer> to_; ///< test object
};

TEST_F(MatOutputLayerTest, Test)
{
    cout<<base_MatOutputLayer::KEY_OUTPUT_M<<std::endl;
    cout<<DummyMatOutputLayer::KEY_OUTPUT_M<<std::endl;
}

} // annonymous namespace for test cases and test fixtures
