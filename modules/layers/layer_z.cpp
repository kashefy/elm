#include "layers/layer_z.h"

#include "core/exception.h"
#include "core/signal.h"

using cv::Mat1f;

// I/O keys
const std::string LayerZ::KEY_INPUT_SPIKES          = "spikes_in";
const std::string LayerZ::KEY_OUTPUT_SPIKES         = "spikes_out";
const std::string LayerZ::KEY_OUTPUT_MEMBRANE_POT   = "u";

// Parameter keys
const std::string LayerZ::PARAM_NB_AFFERENTS        = "nb_afferents";
const std::string LayerZ::PARAM_NB_OUTPUT_NODES     = "nb_outputs";
const std::string LayerZ::PARAM_LEN_HISTORY         = "len_history";
const std::string LayerZ::PARAM_DELTA_T             = "delta_t";
const std::string LayerZ::PARAM_WTA_FREQ            = "wta_f";

// defaults
const int LayerZ::DEFAULT_LEN_HISTORY = 5;
const float LayerZ::DEFAULT_DELTA_T = 1000.f;
const float LayerZ::DEFAULT_WTA_FREQ = 1.f;

LayerZ::~LayerZ()
{
}

LayerZ::LayerZ()
    : base_LearningLayer(),
      wta_(DEFAULT_WTA_FREQ, DEFAULT_DELTA_T) // will get overriden anyway
{
}

LayerZ::LayerZ(const LayerConfig &config)
    : base_LearningLayer(config),
      wta_(DEFAULT_WTA_FREQ, DEFAULT_DELTA_T) // will get overriden anyway
{
    Reset(config);
    IO(config);
}

void LayerZ::Clear()
{
    for(VecZ::iterator itr=z_.begin(); itr != z_.end(); ++itr) {

        (*itr).Clear();
    }
    //todo: either define clear() for wta or re-initialize object..
}

void LayerZ::Reset(const LayerConfig &config)
{
    PTree params = config.Params();
    z_.clear();

    // afferents
    int tmp = params.get<int>(PARAM_NB_AFFERENTS);
    if(tmp < 1) {
        SEM_THROW_VALUE_ERROR("No. of afferents must be > 0");
    }
    nb_afferents_ = tmp;

    // output nodes
    tmp = params.get<int>(PARAM_NB_OUTPUT_NODES);
    if(tmp < 1) {
        SEM_THROW_VALUE_ERROR("No. of output nodes must be > 0");
    }
    int nb_outputs = tmp;

    tmp = params.get<int>(PARAM_LEN_HISTORY, DEFAULT_LEN_HISTORY);
    if(tmp < 1) {
        SEM_THROW_VALUE_ERROR("History length must be > 0");
    }
    int len_history = tmp;

    // push them into a vector
    z_.reserve(nb_outputs);
    while(nb_outputs-- > 0) {

        ZNeuron z = ZNeuron();
        z.Init(nb_afferents_, len_history);
        z_.push_back(z);
    }

    // wta
    float freq = params.get<float>(PARAM_WTA_FREQ, DEFAULT_WTA_FREQ);
    if(freq < 0.f) {

        SEM_THROW_VALUE_ERROR("Frequency must be >= 0");
    }
    // TODO: log warning when freq == 0

    float delta_t = params.get<float>(PARAM_DELTA_T, DEFAULT_DELTA_T);
    if(delta_t <= 0.f) {

        SEM_THROW_VALUE_ERROR("time resolution delta t must be > 0");
    }
    wta_ = WTAPoisson(freq, delta_t);
}

void LayerZ::Reconfigure(const LayerConfig &config)
{
    SEM_THROW_NOT_IMPLEMENTED;
}

void LayerZ::IO(const LayerIO &config)
{
    name_input_spikes_   = config.Input(KEY_INPUT_SPIKES);
    name_output_spikes_  = config.Output(KEY_OUTPUT_SPIKES);
    name_output_mem_pot_ = config.OutputOpt(KEY_OUTPUT_MEMBRANE_POT);
}

void LayerZ::Stimulus(const Signal &signal)
{
    input_spikes_ = signal.MostRecent(name_input_spikes_);
}

void LayerZ::Apply()
{
    for(VecZ::iterator itr=z_.begin(); itr != z_.end(); ++itr) {

        (*itr).Predict(input_spikes_);
    }
}

void LayerZ::Learn()
{

}

void LayerZ::Response(Signal &signal)
{

}
