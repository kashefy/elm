#include "encoding/distributionsampler.h"

#include "core/mat_utils.h"
#include <iostream>

using namespace cv;

cv::RNG base_DistributionSampler::rng_(0xFFFFFFFF);

void base_DistributionSampler::pdf(const MatF &pdf) {

    pdf_ = pdf/sum(pdf)(0);
    sem::CumSum(pdf_, pdf_);
}

base_DistributionSampler::base_DistributionSampler()
{
}

DistributionSampler1D::DistributionSampler1D()
    : base_DistributionSampler()
{
}

int DistributionSampler1D::Sample() const {

    const float R = float(rng_);
    int i = 0;
    while(pdf_(i) < R){
        i++;
    }
    return i;
}

DistributionSampler2D::DistributionSampler2D()
    : base_DistributionSampler()
{
}

Point2i DistributionSampler2D::Sample() const {

    const float R = float(rng_);
    int i = 0;
    while(pdf_(i) < R){
        i++;
    }
    return Point2i(i%pdf_.cols, i/pdf_.cols);
}
