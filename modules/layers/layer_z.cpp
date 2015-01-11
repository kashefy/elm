#include "layers/layer_z.h"

#include <iostream>

#include "core/exception.h"
#include "core/signal.h"

using std::shared_ptr;
using cv::Mat1f;

// I/O keys
const std::string LayerZ::KEY_INPUT_SPIKES          = "spikes_in";
const std::string LayerZ::KEY_OUTPUT_SPIKES         = "spikes_out";
const std::string LayerZ::KEY_OUTPUT_MEMBRANE_POT   = "u";
const std::string LayerZ::KEY_OUTPUT_WEIGHTS        = "w";
const std::string LayerZ::KEY_OUTPUT_BIAS           = "w0";         ///< not the same as weights[0]

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
    IONames(config);
}

void LayerZ::Clear()
{
    for(VecLPtr::iterator itr=z_.begin(); itr != z_.end(); ++itr) {

        (*itr)->Clear();
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

    InitLearners(nb_afferents_, nb_outputs, len_history);

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

void LayerZ::IONames(const LayerIONames &config)
{
    name_input_spikes_   = config.Input(KEY_INPUT_SPIKES);
    name_output_spikes_  = config.Output(KEY_OUTPUT_SPIKES);
    name_output_mem_pot_ = config.OutputOpt(KEY_OUTPUT_MEMBRANE_POT);
    name_output_weights_ = config.OutputOpt(KEY_OUTPUT_WEIGHTS);
    name_output_bias_ = config.OutputOpt(KEY_OUTPUT_BIAS);
}

void LayerZ::Activate(const Signal &signal)
{
    cv::Mat1f spikes_in = signal.MostRecentMat(name_input_spikes_);
    if(spikes_in.total() != static_cast<size_t>(nb_afferents_)) {

        std::stringstream s;
        s << "Expecting " << nb_afferents_ << " input spikes";
        SEM_THROW_BAD_DIMS(s.str());
    }

    // compute membrane potential for each neuron
    u_ = Mat1f(1, static_cast<int>(z_.size()));
    int i=0;
    for(VecLPtr::iterator itr=z_.begin(); itr != z_.end(); ++itr) {

        u_(i++) = (*itr)->Predict(spikes_in.reshape(1, 1)).at<float>(0);
    }

    // let them compete
    spikes_out_ = wta_.Compete(z_);
    //std::cout<<spikes_out_<<std::endl;
}

void LayerZ::Learn()
{
    int i=0;
    for(VecLPtr::iterator itr=z_.begin(); itr != z_.end(); ++itr) {

        (*itr)->Learn(spikes_out_.col(i++));
    }
}

void LayerZ::Response(Signal &signal)
{
    signal.Append(name_output_spikes_, spikes_out_);

    // optional outputs
    if(name_output_mem_pot_) {

        signal.Append(name_output_mem_pot_.get(), u_);
    }

    if(name_output_weights_) {

        Mat1f weights(z_.size(), nb_afferents_);
        int r=0;
        for(VecLPtr::iterator itr=z_.begin(); itr != z_.end(); ++itr) {

            Mat1f w = std::static_pointer_cast<ZNeuron>(*itr)->Weights();
            w.copyTo(weights.row(r++));
        }
        signal.Append(name_output_weights_.get(), weights);
    }

    if(name_output_bias_) {

        Mat1f bias(1, z_.size());
        int i=0;
        for(VecLPtr::iterator itr=z_.begin(); itr != z_.end(); ++itr) {

            bias(i++) = std::static_pointer_cast<ZNeuron>(*itr)->Bias()(0);
        }
        signal.Append(name_output_bias_.get(), bias);
    }
}

void LayerZ::InitLearners(int nb_features, int nb_outputs, int len_history)
{
    z_.clear();
    z_.reserve(nb_outputs);
    while(nb_outputs-- > 0) {

        shared_ptr<ZNeuron> ptr(new ZNeuron);
        ptr->Init(nb_features, len_history);
        z_.push_back(ptr);
    }
}

