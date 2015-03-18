#include "elm/layers/mlp.h"

using namespace elm;

MLP::~MLP()
{
}

MLP::MLP()
    : base_SupervisedBatch()
{
    Clear();
}

MLP::MLP(const LayerConfig &cfg)
    : base_SupervisedBatch(cfg)
{
    Reset(cfg);
}

void MLP::Clear()
{

}

void MLP::Reset(const LayerConfig &config)
{
    Reconfigure(config);
}

void MLP::Reconfigure(const LayerConfig &config)
{

}

void MLP::Activate(const Signal &signal)
{

}

void MLP::Learn(const cv::Mat1f &features, const cv::Mat1f &labels)
{

}
