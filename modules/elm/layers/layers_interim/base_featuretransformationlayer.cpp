#include "elm/layers/layers_interim/base_featuretransformationlayer.h"

#include "elm/core/layerconfig.h"

using namespace elm;

base_FeatureTransformationLayer::base_FeatureTransformationLayer()
    : base_SingleInputFeatureLayer(),
      base_MatOutputLayer()
{
}

base_FeatureTransformationLayer::base_FeatureTransformationLayer(const LayerConfig &config)
    : base_SingleInputFeatureLayer(config),
      base_MatOutputLayer(config)
{
}

void base_FeatureTransformationLayer::IONames(const LayerIONames &io)
{
    base_SingleInputFeatureLayer::InputNames(io);
    base_MatOutputLayer::OutputNames(io);
}

