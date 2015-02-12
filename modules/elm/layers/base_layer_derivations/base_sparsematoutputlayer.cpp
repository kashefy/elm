/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/base_layer_derivations/base_sparsematoutputlayer.h"

#include "elm/core/layerionames.h"
#include "elm/core/signal.h"

using namespace std;
using namespace elm;

const string base_SparseMatOutputLayer::KEY_OUTPUT_RESPONSE = detail::BASE_SPARSEMATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE;

base_SparseMatOutputLayer::~base_SparseMatOutputLayer()
{
}

base_SparseMatOutputLayer::base_SparseMatOutputLayer()
    : base_Layer()
{
}

base_SparseMatOutputLayer::base_SparseMatOutputLayer(const LayerConfig &cfg)
    : base_Layer(cfg)
{
}

void base_SparseMatOutputLayer::OutputNames(const LayerOutputNames &io)
{
    name_output_ = io.Output(KEY_OUTPUT_RESPONSE);
}

void base_SparseMatOutputLayer::Response(Signal &signal)
{
    signal.Append(name_output_, m_);
}
