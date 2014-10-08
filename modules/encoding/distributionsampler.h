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

    /**
     * @brief Get stored pdf
     * @return probability density function
     */
    MatF pdf() const;

protected:
    base_DistributionSampler();

    static cv::RNG rng_;        ///< random number generator
    MatF pdf_;                  ///< probability density function
};

/**
 * @brief The DistributionSampler from 1D PDF
 */
class DistributionSampler1D : public base_DistributionSampler
{
public:
    DistributionSampler1D();

    /**
     * @brief Sample from 1D pdf
     * @return index (bin index)
     */
    int Sample() const;
};

/**
 * @brief The DistributionSampler from two-dimensional PDF
 */
class DistributionSampler2D : public base_DistributionSampler
{
public:
    DistributionSampler2D();

    /**
     * @brief Sample from two-dimensional distribution
     * @return point index
     */
    cv::Point2i Sample() const;
};

/**
 * @brief Class for simulating f-Hz Poisson process
 */
class PoissonProcess : public DistributionSampler1D
{
public:
    /**
     * @brief Construct a Poisson Process instance
     * @param frequency
     * @param time resolution in milliseconds
     */
    PoissonProcess(float frequency, float delat_t_msec);
};

#endif // SEM_ENCODING_DISTRIBUTIONSAMPLER_H_
