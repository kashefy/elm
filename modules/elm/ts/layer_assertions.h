/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
/** @file routines to help test layer functionality
  */
#ifndef ELM_TS_LAYER_UTILS_H_
#define ELM_TS_LAYER_UTILS_H_

#include <memory>

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/stl/stl.h"
#include "elm/layers/layerfactory.h"
#include "elm/ts/ts.h"

class base_Layer;

namespace elm {

enum LayerIOKeyType
{
    INPUT,
    OUTPUT
};

typedef std::pair<bool, std::string> IOName; ///< convinience typedef to layer io name with bool indicating input (0) or output (1)
typedef std::map< std::string, std::pair<bool, std::string> > MapIONamesB; ///< convinience typedef for a map of io keys and corresponding io name

typedef std::map< std::string, std::pair<LayerIOKeyType, std::string> > MapIONames; ///< convinience typedef for a map of io keys and corresponding io name


/**
 * @brief test around a layer's validation of its required io names
 * Basically tests that elm::ExceptionKeyError is thrown whenever a I/O name is not provided.
 *
 * Generates various combinations of present and missing IO names
 *
 * @param map of io name pairs
 * @param pointer to layer under test
 * @todo rewrite to return AssertionResult
 */
void ValidateRequiredIONames(const MapIONamesB &io_pairs, std::shared_ptr<base_Layer> &layer_ptr);

}


/**
 * @brief the struct below, is a helper struct.
 * It enables defining values to be used inside the tests
 *
 * Caution when defining members for you layer type:
 *
 * - statc members such as io_pairs depend on static initializations in the layer.
 *   Initilizing them in the layer's unittest source file, leads to a "static initialization order fiasco"
 *   To avoid this, initialize the LayerAttr_ member for your layer type in your layer's implementation file.
 *   Preferrably inside __WITH_GTEST define-guards.
 *
 * These values are set once per type before INSTANTIATE_TYPED_TEST_CASE_P(...)
 * in subscribing layer's unittest source file.
 */
template<class TLayer>
struct LayerAttr_
{
    static elm::MapIONames io_pairs; ///< needs to be initialized in layer's implementation file.
};

/** Macros for populating LayerAttr_::io_pairs member by client layer in its respective unittest source file
 */
#define ELM_ADD_INPUT_PAIR(key)   ( key, std::make_pair(elm::LayerIOKeyType::INPUT, "n"+key) )
#define ELM_ADD_OUTPUT_PAIR(key)  ( key, std::make_pair(elm::LayerIOKeyType::OUTPUT, "n"+key) )

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
    std::shared_ptr<TLayer > layer_ptr_; ///< pointer to test object used in fixtures
};

TYPED_TEST_CASE_P(Layer_TP_);

TYPED_TEST_P(Layer_TP_, Constructor)
{
    EXPECT_NO_THROW(TypeParam());
}

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
            for(size_t i=0; i<subset.size(); i++) {

                IOName _v = io_pairs.at(subset[i]);

                if(_v.first == LayerIOKeyType::INPUT) {
                    io.Input(subset[i], _v.second);
                }
                else if(_v.first == LayerIOKeyType::OUTPUT) {
                    io.Output(subset[i], _v.second);
                }
            }

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
                           Constructor,
                           RequiredIONamesValidation
                           ); ///< register additional typed_test_p (i.e. unit test) routines here

/** Macro for easier registration for subscribing layers for executing standard/generalized layer tests
 */
#define ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(Layer) INSTANTIATE_TYPED_TEST_CASE_P(Layer_TP_ ## Layer ## _Test, Layer_TP_, Layer)

#endif // ELM_TS_LAYER_UTILS_H_
