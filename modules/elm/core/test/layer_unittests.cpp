/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/base_Layer.h"

#include "gtest/gtest.h"

#include <memory>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"

using namespace std;
using std::shared_ptr;
using std::unique_ptr;
using namespace cv;
using namespace elm;

/** class for deriving from base Layer for test purposes
  */
class DummyChildLayer : public base_Layer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {}

    void InputNames(const LayerIONames &io) {}

    void OutputNames(const LayerIONames &io);

    void Activate(const Signal &signal) {}

    void Response(Signal &signal) {}

    DummyChildLayer() {}
};

class LayerTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.reset(new DummyChildLayer());
    }

    shared_ptr<base_Layer> to_; ///< test object
};

TEST_F(LayerTest, Reset)
{
    EXPECT_THROW(to_->Reset(LayerConfig()), ExceptionNotImpl);
}

TEST_F(LayerTest, Reset_unique_ptr)
{
    unique_ptr<base_Layer> to(new DummyChildLayer());
    EXPECT_THROW(to_->Reset(LayerConfig()), ExceptionNotImpl);
}
