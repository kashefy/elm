/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/layerconfig.h"

using namespace elm;

LayerConfig::LayerConfig()
    : LayerIONames()
{
}

void LayerConfig::Params(const PTree &params)
{
    params_ = params;
}

PTree LayerConfig::Params() const
{
    return params_;
}
