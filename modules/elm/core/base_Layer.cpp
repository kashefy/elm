#include "sem/core/base_Layer.h"

#include "sem/core/exception.h"

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
