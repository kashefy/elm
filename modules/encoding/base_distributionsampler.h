#ifndef SEM_ENCODING_BASE_DISTRIBUTIONSAMPLER_H
#define SEM_ENCODING_BASE_DISTRIBUTIONSAMPLER_H

#include <core/typedefs.h>

class base_DistributionSampler
{
public:
    virtual ~base_DistributionSampler() {}

    void pdf(const MatF &pdf);

protected:
    base_DistributionSampler();

    //static cv::RNG rng_;
    MatF pdf_;

};

#endif // SEM_ENCODING_BASE_DISTRIBUTIONSAMPLER_H
