/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_ENCODING_SOFTMAX_POPULATIONCODE_H_
#define _ELM_ENCODING_SOFTMAX_POPULATIONCODE_H_

#include "elm/encoding/base_populationcode.h"

namespace elm {

/**
 * @brief Population code sampled from distribution
 * (e.g. soft-max oriented gabor filter response)
 * Fanout = input distributions passed to State()
 */
class SoftMaxPopulationCode : public base_PopulationCode
{
public:
    SoftMaxPopulationCode();

    /**
     * @brief Compute distribution for each node from different kernel responses
     * @param input stimulus state (i.e. expecting distribution per row)
     * @param kernels (e.g. filter bank)
     */
    virtual void State(const cv::Mat1f& in);

    /**
     * @brief Determine population code per input node by sampling from response distribution
     * @return population code for all input nodes (dims match that of stimulus)
     */
    virtual cv::Mat1f PopCode();

protected:
    cv::Mat1f state_;   ///< internal state (columns may represent no. kernels)
    float min_distr_sum_; ///< minimum value for sampling from element state (default is zero)
};

} // namespace elm

#endif // _ELM_ENCODING_SOFTMAX_POPULATIONCODE_H_
