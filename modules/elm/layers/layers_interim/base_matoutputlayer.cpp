/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layers_interim/base_matoutputlayer.h"

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

void base_MatOutputLayer::OutputNames(const LayerOutputNames &io)
{
    name_output_ = io.Output(KEY_OUTPUT_RESPONSE);
}

void base_MatOutputLayer::Response(Signal &signal)
{
    signal.Append(name_output_, m_);
}
