#include "encoding/intensitycontrast.h"

using namespace cv;

base_IntensityContrast::~base_IntensityContrast()
{
}

base_IntensityContrast::base_IntensityContrast()
{
}

RetGang::RetGang()
    : base_IntensityContrast()
{
}

void RetGang::Init(int radius, float scale)
{
    rg_.reset(new DiffOfGaussians2dSq());
    rg_->Init(radius, scale, true);
}

void RetGang::Compute(cv::InputArray stimulus)
{
    state_ = rg_->Compute(stimulus);
}

Mat RetGang::Response()
{
    return state_;
}
