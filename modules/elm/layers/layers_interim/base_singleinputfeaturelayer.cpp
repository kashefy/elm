#include "elm/layers/layers_interim/base_singleinputfeaturelayer.h"

#include "elm/core/inputname.h"
#include "elm/core/layerionames.h"
#include "elm/core/signal.h"

using namespace std;
using namespace elm;

const string base_SingleInputFeatureLayer::KEY_INPUT_STIMULUS = detail::BASE_SINGLE_INPUT_FEATURE_LAYER__KEY_INPUT_STIMULUS;

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

void base_SingleInputFeatureLayer::InputNames(const LayerInputNames &io)
{
    name_input_ = io.Input(KEY_INPUT_STIMULUS);
}

