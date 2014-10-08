#include "encoding/distributionsampler.h"

#include "core/mat_utils.h"

using namespace cv;

cv::RNG base_DistributionSampler::rng_(0xFFFFFFFF);

void base_DistributionSampler::pdf(const MatF &pdf) {

    pdf_ = pdf/sum(pdf)(0);
    sem::CumSum(pdf_, pdf_);
}

MatF base_DistributionSampler::pdf() const
{
    return pdf_;
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

PoissonProcess::PoissonProcess(float frequency, float delta_t_msec)
    : DistributionSampler1D()
{
    MatF firing_prob(1, 1);
    firing_prob(0, 0) = frequency*delta_t_msec;
    pdf_ = firing_prob;
}


