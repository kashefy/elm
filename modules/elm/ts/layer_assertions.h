/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file routines to help test layer functionality
  */
#ifndef _ELM_TS_LAYER_ASSERTIONS_H_
#define _ELM_TS_LAYER_ASSERTIONS_H_

#include "gtest/gtest.h"

#include <memory>

#include "elm/core/exception.h"
#include "elm/core/inputname.h"
#include "elm/core/layerconfig.h"
#include "elm/core/stl/stl_inl.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/layerattr_.h"

extern template class std::shared_ptr<elm::base_Layer >;

namespace elm {

/**
 * @brief Facotry function for initializing io names from a subset of io pairs
 * @param[in] io_pairs
 * @param[in] keys I/O keys to use in initialization
 * @param[out] dst resulting object
 */
void InitializeIONames(const MapIONames &io_pairs, const VecS& keys, LayerIONames& dst);

/**
 * @brief A type-parameterized test case for repeating tests with different layer types
 */
template <class TLayer>
class Layer_TP_ : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        layer_ptr_.reset(new TLayer());
    }

    // members
    std::shared_ptr<base_Layer > layer_ptr_; ///< pointer to test object used in fixtures
};

TYPED_TEST_CASE_P(Layer_TP_);

TYPED_TEST_P(Layer_TP_, Clear)
{
    EXPECT_NO_THROW(this->layer_ptr_->Clear());
}

TYPED_TEST_P(Layer_TP_, Constructor)
{
    EXPECT_NO_THROW(TypeParam());
}

TYPED_TEST_P(Layer_TP_, Destructor)
{
    EXPECT_NO_THROW(this->layer_ptr_.reset());
    EXPECT_FALSE(bool(this->layer_ptr_));
}

/**
 * @brief test around a layer's ability to validate its required io names
 * Basically tests that elm::ExceptionKeyError is thrown whenever a required I/O name is not provided.
 *
 * Generates various combinations of present and missing IO names
 *
 * uses:
 * - LayerAttr_<TypeParam >::io_pairs map of io name pairs
 * - pointer to layer under test
 */
TYPED_TEST_P(Layer_TP_, RequiredIONamesValidation)
{
    typedef std::vector<std::string > VecS;
    using namespace elm;
    MapIONames io_pairs = LayerAttr_<TypeParam >::io_pairs;
    VecS keys = Keys(io_pairs);

    for(VecS::iterator itr1=keys.end(); itr1 != keys.begin(); --itr1) {

        VecS subset(keys.begin(), itr1);
        do {

            // populate layer io names with subset
            LayerIONames io;
            InitializeIONames(io_pairs, subset, io);

            if(subset.size() == io_pairs.size()) {

                EXPECT_NO_THROW(this->layer_ptr_->IONames(io));
            }
            else {

                EXPECT_THROW(this->layer_ptr_->IONames(io), ExceptionKeyError);
            }

        } while ( next_permutation(subset.begin(), subset.end()) );
    }
}

/* Write additional type+value parameterized tests here.
 * Acquaint yourself with the values passed to along with each type.
 */

// Register test names
REGISTER_TYPED_TEST_CASE_P(Layer_TP_,
                           Clear,
                           Constructor,
                           Destructor,
                           RequiredIONamesValidation
                           ); ///< register additional typed_test_p (i.e. unit test) routines here

/**
 * @brief A type-parameterized test case for repeating tests with different layer types
 */
template <class TLayer>
class LayerInst_TP_ : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        layer_inst_ptr_.reset(new TLayer());
    }

    // members
    std::shared_ptr<TLayer > layer_inst_ptr_; ///< pointer to test object used in fixtures
};

TYPED_TEST_CASE_P(LayerInst_TP_);

TYPED_TEST_P(LayerInst_TP_, Clear)
{
    EXPECT_NO_THROW(this->layer_inst_ptr_->Clear());
}

TYPED_TEST_P(LayerInst_TP_, Constructor)
{
    EXPECT_NO_THROW(TypeParam());
}

TYPED_TEST_P(LayerInst_TP_, Destructor)
{
    EXPECT_NO_THROW(this->layer_inst_ptr_.reset());
    EXPECT_FALSE(bool(this->layer_inst_ptr_));
}

/* Write additional type+value parameterized tests here.
 * Acquaint yourself with the values passed to along with each type.
 */

// Register test names
REGISTER_TYPED_TEST_CASE_P(LayerInst_TP_,
                           Clear,
                           Constructor,
                           Destructor
                           ); ///< register additional typed_test_p (i.e. unit test) routines here

/** Macro for easier registration for subscribing layers for executing standard/generalized layer tests
 */
#define ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(Layer) INSTANTIATE_TYPED_TEST_CASE_P(Layer_TP_ ## Layer ## _Test, Layer_TP_, Layer); INSTANTIATE_TYPED_TEST_CASE_P(LayerInst_TP_ ## Layer ## _Test, LayerInst_TP_, Layer)

} // namespace elm

#endif // _ELM_TS_LAYER_ASSERTIONS_H_
