/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layers_interim/base_LearningLayer.h"

#include "gtest/gtest.h"

#include <memory>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"

using namespace std;
using std::shared_ptr;
using namespace cv;
using namespace elm;

/** class for deriving from base Layer for test purposes
  */
class DummyChildLearningLayer : public base_LearningLayer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {}

    void InputNames(const LayerInputNames &io) {}

    void OutputNames(const LayerOutputNames &io) {}

    void Activate(const Signal &signal) {}

    void Response(Signal &signal) {}

    void  Learn() {}

    void Learn(const cv::Mat1f& features, const cv::Mat1f &labels) {}

    DummyChildLearningLayer()
        : base_LearningLayer() {}
};

class LearningLayerTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.reset(new DummyChildLearningLayer());
    }

    shared_ptr<base_Layer> to_; ///< test object
};

TEST_F(LearningLayerTest, Destructor)
{
    to_.reset();
}

TEST_F(LearningLayerTest, Reset)
{
    EXPECT_NO_THROW(to_->Reset(LayerConfig()));
}

TEST_F(LearningLayerTest, Reset_unique_ptr)
{
    unique_ptr<base_Layer> to(new DummyChildLearningLayer());
    EXPECT_NO_THROW(to_->Reset(LayerConfig()));
}

/**
 * @brief Repeat abover tests with layer instance declared directly as base_LearningLayer
 */
class LearningLayerInstTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        to_.reset(new DummyChildLearningLayer());
    }

    shared_ptr<base_LearningLayer> to_; ///< test object
};

TEST_F(LearningLayerInstTest, Destructor)
{
    to_.reset();
}

TEST_F(LearningLayerInstTest, Reset)
{
    EXPECT_NO_THROW(to_->Reset(LayerConfig()));
}

TEST_F(LearningLayerInstTest, Reset_unique_ptr)
{
    unique_ptr<base_LearningLayer> to(new DummyChildLearningLayer());
    EXPECT_NO_THROW(to_->Reset(LayerConfig()));
}

TEST_F(LearningLayerInstTest, Reset_unique_ptr_as_base_Layer)
{
    EXPECT_NO_THROW(unique_ptr<base_Layer> to(new DummyChildLearningLayer()));
}
