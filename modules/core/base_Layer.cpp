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

