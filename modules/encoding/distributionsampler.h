#ifndef SEM_ENCODING_DISTRIBUTIONSAMPLER_H_
#define SEM_ENCODING_DISTRIBUTIONSAMPLER_H_

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

protected:
    base_DistributionSampler();

    static cv::RNG rng_;        ///< random number generator
    MatF pdf_;                  ///< probability density function
};

class DistributionSampler1D : public base_DistributionSampler
{
public:
    DistributionSampler1D();

    int Sample() const;
};

#endif // SEM_ENCODING_DISTRIBUTIONSAMPLER_H_
