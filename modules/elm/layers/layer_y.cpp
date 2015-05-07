/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layer_y.h"

#include "elm/core/exception.h"
#include "elm/core/layerconfig.h"
#include "elm/core/boost/ptree_utils.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using std::string;
using namespace elm;

const string LayerY::PARAM_DELTA_T_MSEC = "dt";
const string LayerY::PARAM_FREQ         = "f";

/** @todo why does define guard lead to undefined reference error?
 */
//#ifdef __WITH_GTEST
#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<LayerY>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(detail::BASE_SINGLE_INPUT_FEATURE_LAYER__KEY_INPUT_STIMULUS)
        ELM_ADD_OUTPUT_PAIR(detail::BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE)
        ;
//#endif

LayerY::LayerY()
    : base_FeatureTransformationLayer(),
      YNeuron()
{
}

void LayerY::Clear()
{
}

void LayerY::Reconfigure(const LayerConfig &config)
{
    PTree params = config.Params();
    float frequency = params.get<float>(PARAM_FREQ);
    if(frequency < 0) {
        ELM_THROW_VALUE_ERROR("Poisson frequency must be >= 0 Hz");
    }
    // todo: log warning if f == 0

    float dt_msec = params.get<float>(PARAM_DELTA_T_MSEC);
    if(dt_msec <= 0) {
        ELM_THROW_VALUE_ERROR("dt must be > 0 milliseconds");
    }

    VecS unused_keys;
    if(elm::UnusedNodes(params,
                        VecS({PARAM_FREQ, PARAM_DELTA_T_MSEC}),
                        unused_keys)) {

        std::stringstream s;
        s << "Extranous keys found in paramters (";
        for( VecS::const_iterator itr=unused_keys.begin(); itr != unused_keys.end(); ++itr)
            s << *itr << ",";
        s << ")";
        ELM_THROW_KEY_ERROR(s.str());
    }

    init(frequency, dt_msec);
}

void LayerY::Activate(const Signal &signal)
{
    m_ = static_cast<cv::Mat1f>(State(signal.MostRecentMat1f(name_input_)));
}

cv::Mat1i LayerY::State(cv::Mat1f state)
{
    cv::Mat1i spikes(state.size(), 0);
    for(size_t i=0; i<state.total(); i++) {

        float state_at_i = state(i);
        if(state_at_i == 1 || state_at_i == -1) {

            // determine spiking through poisson process
            spikes(i) = poisson_.Sample();
        }
        else if(state_at_i != 0) {
            ELM_THROW_NOT_IMPLEMENTED_WMSG("Analog state not supported.");
        }
    }

    return spikes;
}
