/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef ELM_CORE_SAMPLER_H_
#define ELM_CORE_SAMPLER_H_

#include <opencv2/core/mat.hpp>

/**
 * @brief base class for sampling from a distribution
 */
class base_Sampler
{
public:
    virtual ~base_Sampler() {}

    /**
     * @brief set probability density function.
     * Prepare pdf for sampling by converting into a series of bin widths
     * @param probability density function
     */
    void pdf(const cv::Mat1f &pdf);

    /**
     * @brief Get stored pdf
     * @return probability density function
     */
    cv::Mat1f pdf() const;

protected:
    base_Sampler();

    cv::Mat1f pdf_;                  ///< probability density function
};

/**
 * @brief class for sampling from 1D PDF
 */
class Sampler1D : public base_Sampler
{
public:
    Sampler1D();

    /**
     * @brief Sample from 1D pdf
     * @return index (bin index)
     */
    int Sample() const;
};

/**
 * @brief Class for simulating f-Hz Poisson process
 */
class PoissonProcess : public Sampler1D
{
public:
    /**
     * @brief Construct a Poisson Process instance
     * @param frequency
     * @param time resolution in milliseconds
     */
    PoissonProcess(float frequency, float delat_t_msec);
};

/**
 * @brief class for sampling from two-dimensional PDF
 */
class Sampler2D : public base_Sampler
{
public:
    Sampler2D();

    /**
     * @brief Sample from two-dimensional distribution
     * @return point index
     */
    cv::Point2i Sample() const;
};

namespace elm
{

/**
 * @brief Draw sample from exponential distribution
 * @param lambda
 * @return sample drawn from exponential distribution
 */
float randexp(float lambda);

}

#endif // ELM_CORE_SAMPLER_H_
