#include "encoding/distributionsampler.h"

#include "core/mat_utils.h"

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
{
}

int DistributionSampler1D::Sample() const {

    const float R = float(rng_);
    int i = 0;
    while(pdf_(i) < R ){
        i++;
    }
    return i;
}
