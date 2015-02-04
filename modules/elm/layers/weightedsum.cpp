/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/weightedsum.h"

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace cv;
using namespace elm;

/** Define parameters and I/O keys
  */
const std::string WeightedSum::PARAM_A              = "a";
const std::string WeightedSum::PARAM_B              = "b";

/** @todo why does define guard lead to undefined reference error?
 */
//#ifdef __WITH_GTEST
#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<WeightedSum>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(detail::BASE_SINGLE_INPUT_FEATURE_LAYER__KEY_INPUT_STIMULUS)
        ELM_ADD_OUTPUT_PAIR(detail::BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE)
        ;
//#endif

void WeightedSum::Clear()
{
    m_ = Mat1f();
}

void WeightedSum::Reset(const LayerConfig &config)
{
    Reconfigure(config);
}

void WeightedSum::Reconfigure(const LayerConfig &config)
{
    // params
    PTree params = config.Params();
    a_ = params.get<float>(PARAM_A);
    b_ = params.get<float>(PARAM_B);
}

void WeightedSum::Activate(const Signal &signal)
{
    Mat1f stimulus = signal[name_input_][0];
    if(stimulus.cols > 2) {

        ELM_THROW_BAD_DIMS("Cannot handle stimulus with > 2 columns.");
    }

    m_ = Mat1f(stimulus.rows, 1);
    for(int r=0; r<stimulus.rows; r++) {

        float tmp = a_ * stimulus(r, 0);
        if(stimulus.cols > 1) {

            tmp += b_ * stimulus(r, 1);
        }
        m_(r) = tmp;
    }
}

WeightedSum::WeightedSum()
    : base_FeatureTransformationLayer()
{
    Clear();
}

WeightedSum::WeightedSum(const LayerConfig& config)
    : base_FeatureTransformationLayer(config)
{
    Clear();
    Reconfigure(config);
    IONames(config);
}
