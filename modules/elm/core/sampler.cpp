/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/sampler.h"

#include <opencv2/core/core.hpp>

#include "elm/core/cv/mat_utils.h"

using namespace cv;

void base_Sampler::pdf(const Mat1f &pdf) {

    pdf_ = pdf/sum(pdf)(0);
    elm::CumSum(pdf_, pdf_);
}

Mat1f base_Sampler::pdf() const
{
    return pdf_;
}

base_Sampler::base_Sampler()
{
}

Sampler1D::Sampler1D()
    : base_Sampler()
{
}

int Sampler1D::Sample() const {

    const float RANDOM = float(theRNG());
    int i = 0;
    while(pdf_(i) < RANDOM && i < static_cast<int>(pdf_.total())){
        i++;
    }
    return i;
}

PoissonProcess::PoissonProcess(float frequency, float delta_t_msec)
    : Sampler1D()
{
    Mat1f firing_prob(1, 1);
    firing_prob(0, 0) = frequency*delta_t_msec/1000.f;
    pdf_ = 1.f-firing_prob;
}

Sampler2D::Sampler2D()
    : base_Sampler()
{
}

Point2i Sampler2D::Sample() const {

    const float RANDOM = float(theRNG());
    int i = 0;
    while(pdf_(i) < RANDOM){
        i++;
    }
    return Point2i(i%pdf_.cols, i/pdf_.cols);
}

float elm::randexp(float lambda)
{
    Mat u(1, 1, CV_64FC1);
    randu(u, 0, 1);
    return static_cast<float>(-log(1.-u.at<double>(0)))/lambda;
}

