#include "core/base_Layer.h"

#include "core/exception.h"

base_Layer::~base_Layer()
{
}

base_Layer::base_Layer()
{
}

base_Layer::base_Layer(const LayerConfig &config)
{
    Reset(config);
}

void base_Layer::Reset(const LayerConfig &config)
{
    SEM_THROW_NOT_IMPLEMENTED;
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
