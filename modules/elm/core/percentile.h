/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_PERCENTILE_H_
#define _ELM_CORE_PERCENTILE_H_

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

#endif // _ELM_CORE_PERCENTILE_H_
