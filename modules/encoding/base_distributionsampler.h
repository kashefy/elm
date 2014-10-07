#ifndef SEM_ENCODING_BASE_DISTRIBUTIONSAMPLER_H
#define SEM_ENCODING_BASE_DISTRIBUTIONSAMPLER_H

#include <core/typedefs.h>

class base_DistributionSampler
{
public:
    virtual ~base_DistributionSampler() {}

    /**
     * @brief set probability density function.
     * Prepare pdf for sampling by converting into a series of bin widths
     * @param probability density function
     */
    void pdf(const MatF &pdf);

    float sample() const;

protected:
    base_DistributionSampler();

    static cv::RNG rng_;
    MatF pdf_;

};

#endif // SEM_ENCODING_BASE_DISTRIBUTIONSAMPLER_H
