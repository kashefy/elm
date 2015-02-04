#include "elm/layers/base_layer_derivations/base_matoutputlayer.h"

#include "elm/core/layerionames.h"
#include "elm/core/signal.h"

using namespace std;
using namespace elm;

const string base_MatOutputLayer::KEY_OUTPUT_RESPONSE = detail::BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE;

base_MatOutputLayer::~base_MatOutputLayer()
{
}

base_MatOutputLayer::base_MatOutputLayer()
    : base_Layer()
{
}

base_MatOutputLayer::base_MatOutputLayer(const LayerConfig &cfg)
    : base_Layer(cfg)
{
}

void base_MatOutputLayer::IONames(const LayerIONames &io)
{
    name_output_ = io.Output(KEY_OUTPUT_RESPONSE);
}

void base_MatOutputLayer::Response(Signal &signal)
{
    signal.Append(name_output_, m_);
}
