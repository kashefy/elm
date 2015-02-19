/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file routines to help test layer functionality
  */
#ifndef _ELM_TS_LAYER_FEAT_TRANSF_ASSERTIONS_H_
#define _ELM_TS_LAYER_FEAT_TRANSF_ASSERTIONS_H_

#include "elm/ts/layer_assertions.h"
#include "elm/core/signal.h"

#include <opencv2/core/core.hpp>

namespace elm {

/**
 * @brief A type-parameterized test case
 * for repeating tests with different layer types derived from
 * intermediate feature transfomation class
 */
template <class TLayer>
class LayerFeatTransf_TP_ : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        LayerConfig cfg;

        LayerIONames io_names;

        typedef std::vector<std::string > VecS;
        using namespace elm;
        MapIONames io_pairs = LayerAttr_<TLayer >::io_pairs;
        VecS keys = Keys(io_pairs);

        for(size_t i=0; i<keys.size(); i++) {

            std::string key = keys[i];
            IOName _v = io_pairs.at(key);

            if(_v.first == LayerIOKeyType::INPUT) {
                io_names.Input(key, _v.second);
                name_in_ = _v.second;
            }
            else if(_v.first == LayerIOKeyType::OUTPUT) {
                io_names.Output(key, _v.second);
                name_out_ = _v.second;
            }
        }

        layer_ptr_.reset(new TLayer());
        layer_ptr_->Reset(cfg);
        layer_ptr_->IONames(io_names);
    }

    // members
    std::shared_ptr<TLayer > layer_ptr_; ///< pointer to test object used in fixtures
    std::string name_in_;
    std::string name_out_;
};

TYPED_TEST_CASE_P(LayerFeatTransf_TP_);

TYPED_TEST_P(LayerFeatTransf_TP_, Response_exists)
{
    Signal sig;
    sig.Append(this->name_in_, cv::Mat1f(10, 10, 1.f));

    this->layer_ptr_->Activate(sig);

    EXPECT_FALSE(sig.Exists(this->name_out_));

    this->layer_ptr_->Response(sig);

    EXPECT_TRUE(sig.Exists(this->name_out_));
}

/* Write additional type+value parameterized tests here.
 * Acquaint yourself with the values passed to along with each type.
 */

// Register test names
REGISTER_TYPED_TEST_CASE_P(LayerFeatTransf_TP_,
                           Response_exists
                           ); ///< register additional typed_test_p (i.e. unit test) routines here

/** Macro for easier registration for subscribing layers for executing standard/generalized layer tests
 */
#define ELM_INSTANTIATE_LAYER_FEAT_TRANSF_TYPED_TEST_CASE_P(Layer) INSTANTIATE_TYPED_TEST_CASE_P(LayerFeatTransf_TP_ ## Layer ## _Test, LayerFeatTransf_TP_, Layer)

} // namespace elm

#endif // _ELM_TS_FEAT_TRANSF_LAYER_ASSERTIONS_H_
