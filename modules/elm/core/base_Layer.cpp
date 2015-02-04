/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/base_Layer.h"

#include "elm/core/exception.h"
#include "elm/core/layerionames.h"

using namespace elm;

base_Layer::~base_Layer()
{
}

base_Layer::base_Layer()
{
}

base_Layer::base_Layer(const LayerConfig &config)
{
    // these calls belong in child constructor
    // Reset(config);
    // IONames(config);
}

void base_Layer::Reset(const LayerConfig &config)
{
    ELM_THROW_NOT_IMPLEMENTED;
}

void base_Layer::IONames(const LayerIONames &io)
{
    InputNames(io);
    OutputNames(io);
}
