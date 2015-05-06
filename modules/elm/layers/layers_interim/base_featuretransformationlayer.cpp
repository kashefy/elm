#include "elm/layers/layers_interim/base_featuretransformationlayer.h"

#include "elm/core/layerconfig.h"

using namespace elm;

base_FeatureTransformationLayer::~base_FeatureTransformationLayer()
{
}

base_FeatureTransformationLayer::base_FeatureTransformationLayer()
    : base_SingleInputFeatureLayer(),
      base_MatOutputLayer()
{
}

void base_FeatureTransformationLayer::IONames(const LayerIONames &io)
{
    base_SingleInputFeatureLayer::InputNames(io);
    base_MatOutputLayer::OutputNames(io);
}

