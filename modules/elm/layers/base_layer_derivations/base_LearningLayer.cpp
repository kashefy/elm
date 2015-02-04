/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/base_layer_derivations/base_LearningLayer.h"

using namespace elm;

base_LearningLayer::~base_LearningLayer()
{
}

base_LearningLayer::base_LearningLayer()
    : base_Layer()
{
}

base_LearningLayer::base_LearningLayer(const LayerConfig &config)
    : base_Layer(config)
{
}
