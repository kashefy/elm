#include "encoding/distributionsampler1d.h"

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
