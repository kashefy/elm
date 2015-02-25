/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/layers/layers_interim/base_smoothlayer.h"

#include "elm/core/layerconfig.h"

using namespace cv;
using namespace elm;

/** Define parameters, defaults and I/O keys
  */
// paramters
const std::string base_SmoothLayer::PARAM_APERTURE_SIZE = "aperture_size";

base_SmoothLayer::~base_SmoothLayer()
{
}

base_SmoothLayer::base_SmoothLayer()
    : base_FeatureTransformationLayer()
{
}

base_SmoothLayer::base_SmoothLayer(const LayerConfig &config)
    : base_FeatureTransformationLayer(config)
{
}

void base_SmoothLayer::Clear()
{
    m_ = Mat1f();
}

void base_SmoothLayer::Reset(const LayerConfig &config)
{
    Clear();
    Reconfigure(config);
}

void base_SmoothLayer::Reconfigure(const LayerConfig &config)
{
    PTree p = config.Params();
    ksize_ = p.get<int>(PARAM_APERTURE_SIZE);
}
