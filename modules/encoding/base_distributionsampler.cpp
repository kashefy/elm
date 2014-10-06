#include "base_distributionsampler.h"

using namespace cv;

//cv::RNG base_DistributionSampler::rng_(0xFFFFFFFF);

void base_DistributionSampler::pdf(const MatF &pdf)
{
    pdf_ = pdf/sum(pdf)(0);
}

base_DistributionSampler::base_DistributionSampler()
{
}
