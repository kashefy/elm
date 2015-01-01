#include "layers/layer_y.h"

#include "core/exception.h"
#include "core/layerconfig.h"
#include "core/ptree_utils.h"
#include "core/signal.h"

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
    state_ = State(signal.MostRecent(name_stimulus_).at<float>(0));
}

void LayerY::Response(Signal &signal)
{
    signal.Append(name_spikes_, cv::Mat1f(1, 1, static_cast<float>(state_)));
}

