#ifndef SEM_ENCODING_POPULATIONCODE_H_
#define SEM_ENCODING_POPULATIONCODE_H_

#include <core/typedefs.h>

/**
 * @brief The base class for population codes
 */
class base_PopulationCode
{
public:
    virtual ~base_PopulationCode() {}

    /**
     * @brief Compute internal state
     * @param in input
     * @param kernels vector of kernels
     */
    virtual void State(const cv::Mat1f& in, const VecMat1f& kernels=VecMat1f()) = 0;

    /**
     * @brief get population code
     * @return population code
     */
    virtual cv::Mat1f PopCode() = 0;

protected:
    base_PopulationCode();
};

/**
 * @brief Mutually exclusive population code (a.k.a simple pop. code)
 */
class MutexPopulationCode : public base_PopulationCode
{
public:
    MutexPopulationCode();

    virtual void State(const cv::Mat1f& in, const VecMat1f& kernels=VecMat1f());

    virtual cv::Mat1f PopCode();

protected:
    cv::Mat1f state_;    ///< internal state
};

/**
 * @brief Population code sampled from distribution
 * (i.e. soft-max oriented gabor filter response)
 */
class SoftMaxPopulationCode : public base_PopulationCode
{
public:
    SoftMaxPopulationCode();

    /**
     * @brief Compute distribution for each node from different kernel responses
     * @param input stimulus
     * @param kernels (e.g. filter bank)
     */
    virtual void State(const cv::Mat1f& in, const VecMat1f& kernels=VecMat1f());

    /**
     * @brief Determine population code per input node by sampling from response distribution
     * @return population code for all input nodes
     */
    virtual cv::Mat1f PopCode();

protected:
    VecMat1f state_;            ///< internal state
    VecMat1f response_distr_;    ///< response distribution
};

#endif // SEM_ENCODING_POPULATIONCODE_H_
