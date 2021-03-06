/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/encoding/intensitycontrast.h"

#include "elm/core/zerocrossings.h"

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
    state_ = Mat1f();
}

void RetGang::Compute(InputArray stimulus)
{
    state_ = rg_->Compute(stimulus);
}

Mat RetGang::Response()
{
    ZeroCrossingsDiff zc;
    Mat1f r;
    zc(state_, r);
    return r;
}

Mat RetGang::State() const
{
    return state_;
}
