#include "elm/layers/gradassignment.h"

#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"

using namespace cv;
using namespace elm;

GradAssignment::~GradAssignment()
{
}

GradAssignment::GradAssignment()
    : base_Layer()
{
    Clear();
}

GradAssignment::GradAssignment(const LayerConfig &cfg)
    : base_Layer(cfg)
{
    Reset(cfg);
}

void GradAssignment::Clear()
{
    m_ = Mat1f();
}

void GradAssignment::Reset(const LayerConfig &config)
{
    Clear();
    Reconfigure(config);
}

void GradAssignment::Reconfigure(const LayerConfig &config)
{
    PTree params = config.Params();

    beta_0_ = params.get<float>(PARAM_BETA);
    if(beta_0_ <= 0.f) {

        ELM_THROW_VALUE_ERROR("Control parameter beta must be positive.");
    }

    beta_max_ = params.get<float>(PARAM_BETA_MAX);

    beta_rate_ = params.get<float>(PARAM_BETA_RATE);
    if(beta_rate_ <= 1.f) {

        ELM_THROW_VALUE_ERROR("rate must be > 1.");
    }

    max_iter_per_beta_ = params.get<int>(PARAM_MAX_ITER_SINKHORN);
    max_iter_sinkhorn_ = params.get<int>(PARAM_MAX_ITER_PER_BETA);
}

void GradAssignment::IONames(const LayerIONames &config)
{
    name_g_ab_ = config.Input(KEY_INPUT_GRAPH_AB);
    name_g_ij_ = config.Input(KEY_INPUT_GRAPH_IJ);
    name_out_m_ = config.Output(KEY_OUTPUT_M);
}

void GradAssignment::Activate(const Signal &signal)
{

}

void GradAssignment::Response(Signal &signal)
{
    signal.Append(name_out_m_, m_);
}
