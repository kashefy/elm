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

#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>

#include "elm/core/base_Layer.h"
#include "elm/core/layerconfig.h"
#include "elm/core/inputname.h"
#include "elm/core/typedefs_sfwd.h"
#include "elm/encoding/base_populationcode.h"

namespace elm {

class base_FilterBank;

/**
 * @brief Population code sampled from distribution
 * (e.g. soft-max oriented gabor filter response)
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
    virtual void State(const cv::Mat1f& in, const elm::VecMat1f& kernels=elm::VecMat1f());

    /**
     * @brief Compute distribution for each node from different kernel responses
     * Use predefined kernels inside a filter bank
     * @param filter_bank, ownership kept by caller
     */
    virtual void State(const cv::Mat1f &in, std::unique_ptr<base_FilterBank> const &filter_bank);

    /**
     * @brief Determine population code per input node by sampling from response distribution
     * @return population code for all input nodes
     */
    virtual cv::Mat1f PopCode();

protected:
    /**
     * @brief Normalize kernel response by global factor
     * @param response normalized in-place
     */
    virtual void Normalize(elm::VecMat1f &response) const;

    VecMat1f state_;            ///< internal state
    VecMat1f response_distr_;   ///< response distribution
    int fan_out_;               ///< dimensions of state per node (e.g. no. of kernels)

};

} // namespace elm

#endif // _ELM_ENCODING_SOFTMAX_POPULATIONCODE_H_
