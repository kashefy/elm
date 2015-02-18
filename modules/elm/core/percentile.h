/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_PERCENTILE_H_
#define _ELM_CORE_PERCENTILE_H_

#include <opencv2/core/core.hpp>

namespace elm {

/**
 * @brief class for calculating percentile in a matrix
 */
class Percentile
{
public:
    static const float MEDIAN;  /// 0.5f

    Percentile();

    /**
     * @brief Calculate percentile from matrix
     *
     * To calculate median of an image, the Mat needs to be flattened first.
     * (e.g. Percentile().CalcPercentile(in.reshape(1, 1), percentile);)
     *
     * @param input matrix of floats
     * @param percentile [0, 1] (e.g. 0.5 to target median)
     * @return element value in requested percentile
     */
    float CalcPercentile(const cv::Mat1f &in, float percentile) const;
};

} // namespace elm

#endif // _ELM_CORE_PERCENTILE_H_
