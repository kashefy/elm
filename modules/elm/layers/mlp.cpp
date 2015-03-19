#include "elm/layers/mlp.h"

#include "elm/core/boost/translators/transl_str_cvtermcriteria.h"
#include "elm/core/boost/translators/transl_str_veci.h"
#include "elm/core/layerconfig.h"
#include "elm/core/signal.h"
#include "elm/ts/layerattr_.h"

using namespace std;
using namespace cv;
using namespace elm;

const string MLP::PARAM_ARCH;
const string MLP::PARAM_TERM_CRITERIA;
const string MLP::PARAM_TRAIN_METHOD;
const string MLP::PARAM_BP_DW_SCALE;
const string MLP::PARAM_BP_MOMENT_SCALE;

#include <boost/assign/list_of.hpp>
template <>
elm::MapIONames LayerAttr_<MLP>::io_pairs = boost::assign::map_list_of
        ELM_ADD_INPUT_PAIR(detail::BASE_SINGLE_INPUT_FEATURE_LAYER__KEY_INPUT_STIMULUS)
        ELM_ADD_OUTPUT_PAIR(detail::BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE)
        ;

MLP::~MLP()
{
}

MLP::MLP()
    : base_SupervisedBatch()
{
    Clear();
}

MLP::MLP(const LayerConfig &cfg)
    : base_SupervisedBatch(cfg)
{
    Reset(cfg);
}

void MLP::Clear()
{
    mlp_.clear();
}

void MLP::Reset(const LayerConfig &config)
{
    PTree p = config.Params();
    VecI arch = p.get<VecI >(PARAM_ARCH);

    const int NB_LAYERS = static_cast<int>(arch.size());

    ELM_THROW_BAD_DIMS_IF(NB_LAYERS < 2, "Need at least 2 layers for mlp.");

    Mat1i layers(arch);
    layers = layers.reshape(1, NB_LAYERS); // resize to column matrix, row per layer

    mlp_.create(layers);

    Reconfigure(config);
}

void MLP::Reconfigure(const LayerConfig &config)
{
    PTree p = config.Params();

    params_.train_method    = p.get<int>(PARAM_TRAIN_METHOD, CvANN_MLP_TrainParams::BACKPROP);
    params_.bp_dw_scale     = p.get<float>(PARAM_BP_DW_SCALE, 0.05f);
    params_.bp_moment_scale = p.get<float>(PARAM_BP_MOMENT_SCALE, 0.05f);

    params_.term_crit = p.get<CvTermCriteria>(PARAM_TERM_CRITERIA);
}

void MLP::Activate(const Signal &signal)
{
    Mat1f sample = signal.MostRecentMat1f(name_input_);
    mlp_.predict(sample, m_);
}

void MLP::Learn(const Mat1f &features, const Mat1f &labels)
{
    mlp_.train(features, labels, Mat(), Mat(), params_);
}
