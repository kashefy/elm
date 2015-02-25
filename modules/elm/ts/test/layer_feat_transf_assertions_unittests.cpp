/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/layer_feat_transf_assertions.h"

#include "gtest/gtest.h"

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/layers/layers_interim/base_featuretransformationlayer.h"

using namespace std;
using namespace elm;

/** class for deriving from intermediate Feat. transformation Layer for test purposes
  */
class DummyChildFeatTransfLayer : public base_FeatureTransformationLayer
{
public:
    void Clear() {}

    void Reconfigure(const LayerConfig &config) {}

    void Activate(const Signal &signal) {

        m_ = signal.MostRecentMat1f(name_input_)*2.f + 2.f;
    }

    DummyChildFeatTransfLayer() {}
};

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<DummyChildFeatTransfLayer>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(detail::BASE_SINGLE_INPUT_FEATURE_LAYER__KEY_INPUT_STIMULUS)
        ELM_ADD_OUTPUT_PAIR(detail::BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE)
        ;

ELM_INSTANTIATE_LAYER_TYPED_TEST_CASE_P(DummyChildFeatTransfLayer);

TEST(LayerFeatTransfAssertionsReqIONamesTest, EmptyIONames)
{
    shared_ptr<base_Layer> to_ptr(new DummyChildFeatTransfLayer());
    EXPECT_THROW(to_ptr->IONames(LayerIONames()), ExceptionKeyError);
}
