#include "sem/layers/layer_y.h"

#include "sem/core/exception.h"
#include "sem/core/layerconfig.h"
#include "sem/core/boost/ptree_utils.h"
#include "sem/core/signal.h"

using std::string;

const string LayerY::PARAM_DELTA_T_MSEC = "dt";
const string LayerY::PARAM_FREQ         = "f";

const string LayerY::KEY_INPUT_STIMULUS = "stimulus";
const string LayerY::KEY_OUTPUT_SPIKES  = "spikes";

LayerY::LayerY()
    : base_Layer(),
      YNeuron()
{
}

LayerY::LayerY(const LayerConfig &config)
    : base_Layer(config),
      YNeuron()
{
    Reset(config);
    IONames(config);
}

void LayerY::Clear()
{
}

void LayerY::Reset(const LayerConfig &config)
{
    Reconfigure(config);
}

void LayerY::Reconfigure(const LayerConfig &config)
{
    PTree params = config.Params();
    float frequency = params.get<float>(PARAM_FREQ);
    if(frequency < 0) {
        SEM_THROW_VALUE_ERROR("Poisson frequency must be >= 0 Hz");
    }
    // todo: log warning if f == 0

    float dt_msec = params.get<float>(PARAM_DELTA_T_MSEC);
    if(dt_msec <= 0) {
        SEM_THROW_VALUE_ERROR("dt must be > 0 milliseconds");
    }

    VecS unused_keys;
    if(sem::UnusedNodes(params,
                        VecS({PARAM_FREQ, PARAM_DELTA_T_MSEC}),
                        unused_keys)) {

        std::stringstream s;
        s << "Extranous keys found in paramters (";
        for( VecS::const_iterator itr=unused_keys.begin(); itr != unused_keys.end(); ++itr)
            s << *itr << ",";
        s << ")";
        SEM_THROW_KEY_ERROR(s.str());
    }

    init(frequency, dt_msec);
}

void LayerY::IONames(const LayerIONames &config)
{
    name_stimulus_ = config.Input(KEY_INPUT_STIMULUS);
    name_spikes_ = config.Output(KEY_OUTPUT_SPIKES);
}

void LayerY::Activate(const Signal &signal)
{
    state_ = State(signal.MostRecentMat(name_stimulus_));
}

void LayerY::Response(Signal &signal)
{
    signal.Append(name_spikes_, static_cast<cv::Mat1f>(state_));
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
            SEM_THROW_NOT_IMPLEMENTED_WMSG("Analog state not supported.");
        }
    }

    return spikes;
}
