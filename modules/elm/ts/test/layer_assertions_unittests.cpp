/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/layer_assertions.h"

#include "gtest/gtest.h"

#include "elm/core/base_Layer.h"
#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"

using namespace std;
using namespace elm;

/** class for deriving from base Layer for test purposes
  */
class DummyChildLayer : public base_Layer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {}

    virtual void InputNames(const LayerInputNames &io) {}

    virtual void OutputNames(const LayerOutputNames &io) {}

    void Activate(const Signal &signal) {}

    void Response(Signal &signal) {}

    DummyChildLayer() {}
};

TEST(LayerAssertionsNoReqIONamesTest, EmptyIONames)
{
    shared_ptr<base_Layer> to_ptr(new DummyChildLayer());
    EXPECT_NO_THROW(to_ptr->IONames(LayerIONames()));
}

const string NAME_IN        = "i";
const string NAME_OUT       = "o";
const string NAME_OPT_OUT   = "oo";

class DummyChildLayerWithReqIONames : public DummyChildLayer
{
public:
    // I/O Names
    static const string KEY_INPUT_IN;
    static const string KEY_OUTPUT_OUT;
    static const string KEY_OUTPUT_OPT_OUT;

    virtual void InputNames(const LayerInputNames &io) {

        name_in_    = io.Input(KEY_INPUT_IN);
    }

    virtual void OutputNames(const LayerOutputNames &io) {

        name_out_   = io.Output(KEY_OUTPUT_OUT);
        name_opt_out_ = io.OutputOpt(KEY_OUTPUT_OPT_OUT);
    }

    DummyChildLayerWithReqIONames() : DummyChildLayer() {}

    // members
    string name_in_;
    string name_out_;
    OptS name_opt_out_;
};
const string DummyChildLayerWithReqIONames::KEY_INPUT_IN        = "in";
const string DummyChildLayerWithReqIONames::KEY_OUTPUT_OUT      = "out";
const string DummyChildLayerWithReqIONames::KEY_OUTPUT_OPT_OUT  = "optout";

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<DummyChildLayerWithReqIONames>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(DummyChildLayerWithReqIONames::KEY_INPUT_IN)
        ELM_ADD_OUTPUT_PAIR(DummyChildLayerWithReqIONames::KEY_OUTPUT_OUT)
        ;

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(DummyChildLayerWithReqIONames);

TEST(LayerAssertionsReqIONamesTest, EmptyIONames)
{
    shared_ptr<base_Layer> to_ptr(new DummyChildLayerWithReqIONames());
    EXPECT_THROW(to_ptr->IONames(LayerIONames()), ExceptionKeyError);
}
