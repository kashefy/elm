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

}

void LayerZ::Reset()
{

}

void LayerZ::Reset(const LayerConfig &config)
{
    PTree params = config.Params();
    z_.clear();
    nb_afferents_ = params.get<int>(PARAM_NB_AFFERENTS);
    if(nb_afferents_ < 1) {
        SEM_THROW_VALUE_ERROR("No. of afferents must be > 0");
    }

}

void LayerZ::Reconfigure(const LayerConfig &config)
{
    name_input_spikes_   = config.Input(KEY_INPUT_SPIKES);
    name_output_spikes_  = config.Input(KEY_OUTPUT_SPIKES);
    name_output_mem_pot_ = config.Input(KEY_OUTPUT_MEMBRANE_POT);
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
