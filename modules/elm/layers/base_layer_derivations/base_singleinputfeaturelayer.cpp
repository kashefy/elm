#include "elm/layers/base_layer_derivations/base_singleinputfeaturelayer.h"

#include "elm/core/layerionames.h"
#include "elm/core/signal.h"

using namespace std;
using namespace elm;

const string base_SingleInputFeatureLayer::KEY_INPUT = detail::BASE_SINGLE_INPUT_FEATURE_LAYER__KEY_INPUT;

base_SingleInputFeatureLayer::~base_SingleInputFeatureLayer()
{
}

base_SingleInputFeatureLayer::base_SingleInputFeatureLayer()
    : base_Layer()
{
}

base_SingleInputFeatureLayer::base_SingleInputFeatureLayer(const LayerConfig &cfg)
    : base_Layer(cfg)
{
}

void base_SingleInputFeatureLayer::IONames(const LayerIONames &io)
{
    name_input_ = io.Input(KEY_INPUT);
}

