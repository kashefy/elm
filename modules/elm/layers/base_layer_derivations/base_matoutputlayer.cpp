#include "elm/layers/base_layer_derivations/base_matoutputlayer.h"

#include "elm/core/layerionames.h"
#include "elm/core/signal.h"

using namespace std;
using namespace elm;

const string base_MatOutputLayer::KEY_OUTPUT_M = "m";

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
    name_out_m_ = io.Output(KEY_OUTPUT_M);
}

void base_MatOutputLayer::Response(Signal &signal)
{
    signal.Append(name_out_m_, m_);
}
