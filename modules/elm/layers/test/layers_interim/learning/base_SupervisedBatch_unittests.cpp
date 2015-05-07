/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layers_interim/learning/base_SupervisedBatch.h"

#include <memory>

#include "gtest/gtest.h"

#include "elm/core/defs.h"
#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/ts/learninglayer_assertions.h"

using namespace elm;

class DummySupervisedBatch : public base_SupervisedBatch
{
public:
    DummySupervisedBatch()
        : base_SupervisedBatch(),
          flag(false)
    {}

    void Learn(const cv::Mat1f& features, const cv::Mat1f &labels)
    {
        flag = true;
    }

    void Clear()
    {}

    void Reset(const elm::LayerConfig &cfg)
    {}

    void Reconfigure(const elm::LayerConfig &cfg)
    {}

    void Activate(const Signal &signal)
    {}

    void Response(Signal &signal)
    {}

    bool getFlag() const
    {
        return flag;
    }

protected:
    bool flag;
};

class SupervisedBatchTest : public ::testing::Test
{
protected:
    void SetUp()
    {
        to_.reset(new DummySupervisedBatch());
    }

    std::shared_ptr<base_Layer> to_;
};

TEST_F(SupervisedBatchTest, Constructor)
{
    EXPECT_NO_THROW(DummySupervisedBatch());
}

TEST_F(SupervisedBatchTest, Learn_batch)
{
    ASSERT_FALSE(ELM_DYN_CAST(DummySupervisedBatch, to_)->getFlag());

    EXPECT_NO_THROW(ELM_DYN_CAST(base_LearningLayer, to_)->Learn(cv::Mat(), cv::Mat()));

    EXPECT_TRUE(ELM_DYN_CAST(DummySupervisedBatch, to_)->getFlag());
}

TEST_F(SupervisedBatchTest, Learn_online)
{
    EXPECT_THROW(ELM_DYN_CAST(base_LearningLayer, to_)->Learn(), ExceptionNotImpl);
}

ELM_INSTANTIATE_LEARNING_LAYER_TYPED_TEST_CASE_P(DummySupervisedBatch);

/**
 * @brief Repeat above with pointer declared of type base_SupervisedBatch
 * Only relevant for Constructor and Destructor coverage, irrelevant for methods.
 */
class SupervisedBatchInstTest : public ::testing::Test
{
protected:
    void SetUp()
    {
        to_.reset(new DummySupervisedBatch());
    }

    std::shared_ptr<base_SupervisedBatch> to_;
};

TEST_F(SupervisedBatchInstTest, Constructor)
{
    EXPECT_NO_THROW(to_.reset(new DummySupervisedBatch()));
    EXPECT_NO_THROW(DummySupervisedBatch());

    EXPECT_NO_THROW(DummySupervisedBatch x);
}

TEST_F(SupervisedBatchInstTest, Destructor)
{
    EXPECT_NO_THROW(to_.reset());

    std::shared_ptr<DummySupervisedBatch> to2;
    EXPECT_NO_THROW(to2.reset(new DummySupervisedBatch()));
    EXPECT_NO_THROW(to2.reset());
}

