#ifndef SEM_CORE_PERCENTILE_H_
#define SEM_CORE_PERCENTILE_H_

#include <opencv2/core.hpp>

/**
 * @brief class for calculating percentile in a matrix
 */
class Percentile
{
public:
    Percentile();

    /**
     * @brief Calculate percentile from matrix
     * @param input matrix of floats
     * @param percentile [0, 1] (e.g. 0.5 to target median)
     * @return element value in requested percentile
     */
    float CalcPercentile(const cv::Mat1f &in, float percentile) const;
};

#endif // SEM_CORE_PERCENTILE_H_
