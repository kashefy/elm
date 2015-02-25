/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_ENCODING_SOFTMAX_POPULATIONCODE_H_
#define _ELM_ENCODING_SOFTMAX_POPULATIONCODE_H_

#include <memory>

#include "elm/encoding/base_populationcode.h"

namespace elm {

class base_FilterBank;

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
     * @param input stimulus (expecting distribution per row)
     * @param kernels (e.g. filter bank)
     */
    virtual void State(const cv::Mat1f& in);

    /**
     * @brief Determine population code per input node by sampling from response distribution
     * @return population code for all input nodes (dims match that of stimulus)
     */
    virtual cv::Mat1f PopCode();

protected:
    VecMat1f state_;            ///< internal state
    int fan_out_;               ///< dimensions of state per node (e.g. no. of kernels)

};

} // namespace elm

#endif // _ELM_ENCODING_SOFTMAX_POPULATIONCODE_H_
