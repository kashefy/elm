/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file routines to help test layer functionality
  */
#ifndef _ELM_TS_LEARNINGLAYER_ASSERTIONS_H_
#define _ELM_TS_LEARNINGLAYER_ASSERTIONS_H_

#include <memory>

#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/stl/stl_inl.h"
#include "elm/layers/layerfactory.h"
#include "elm/layers/layers_interim/base_LearningLayer.h"
#include "elm/ts/layerattr_.h"
#include "elm/ts/ts.h"

namespace elm {

/**
 * @brief A type-parameterized test case for repeating tests with different layer types
 */
template <class TLayer>
class LearningLayer_TP_ : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        learning_layer_ptr_.reset(new TLayer());
    }

    // members
    std::shared_ptr<base_LearningLayer > learning_layer_ptr_; ///< pointer to test object used in fixtures
};

TYPED_TEST_CASE_P(LearningLayer_TP_);

TYPED_TEST_P(LearningLayer_TP_, Clear)
{
    EXPECT_NO_THROW(this->learning_layer_ptr_->Clear());
}

TYPED_TEST_P(LearningLayer_TP_, Constructor)
{
    EXPECT_NO_THROW(TypeParam());
}

TYPED_TEST_P(LearningLayer_TP_, Destructor)
{
    EXPECT_NO_THROW(this->learning_layer_ptr_.reset());
    EXPECT_FALSE(bool(this->learning_layer_ptr_));
}

/* Write additional type+value parameterized tests here.
 * Acquaint yourself with the values passed to along with each type.
 */

// Register test names
REGISTER_TYPED_TEST_CASE_P(LearningLayer_TP_,
                           Clear,
                           Constructor,
                           Destructor
                           ); ///< register additional typed_test_p (i.e. unit test) routines here

/** Macro for easier registration for subscribing layers for executing standard/generalized layer tests
 */
#define ELM_INSTANTIATE_LEARNING_LAYER_TYPED_TEST_CASE_P(Layer) INSTANTIATE_TYPED_TEST_CASE_P(LearningLayer_TP_ ## Layer ## _Test, LearningLayer_TP_, Layer)

} // namespace elm

#endif // _ELM_TS_LEARNINGLAYER_ASSERTIONS_H_
