/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** @file Define classes for encapsulating, applying filter banks
 */
#ifndef _ELM_ENCODING_BASE_FILTERBANKLAYER_H_
#define _ELM_ENCODING_BASE_FILTERBANKLAYER_H_

#include <opencv2/core/core.hpp>    // complete definition for VecMat1f

#include "elm/core/exception.h"
#include "elm/core/typedefs_sfwd.h"
#include "elm/layers/base_layer_derivations/base_singleinputfeaturelayer.h"

namespace elm {

/**
 * @brief base class for filter banks
 *  The interface should aid in iterating through response
 */
class base_FilterBankLayer :
        public base_SingleInputFeatureLayer
{
public:
    static const std::string KEY_OUTPUT_RESPONSE;   ///< key to output/response

    /** Empty default destructor
     */
    virtual ~base_FilterBankLayer();

    virtual void Clear();

    virtual void Reset(const LayerConfig &config);

    virtual void OutputNames(const LayerOutputNames &io);

    virtual void Activate(const Signal &signal);

    virtual void Response(Signal &signal);

    /**
     * @brief Convolve stimulus with each kernel
     * @param stimulus
     * @return response per kernel
     */
    virtual elm::VecMat1f Convolve(const cv::Mat1f &stimulus);

    /**
     * @brief Get element response across all kernels
     * @param row
     * @param column
     * @return element response row matrix no. of cols==filter bank fan-out (e.g. no. of kernels)
     */
    virtual cv::Mat1f ElementResponse(int r, int c) const;

    /**
     * @brief Get underlying kernels
     * called by Reset for initializing kernels_ member
     * @return vector of kernels
     */
    virtual elm::VecMat1f Kernels() const = 0;

    /**
     * @brief size of the filter bank (e.g. no. of kernels)
     * @return size of the filter bank
     */
    virtual size_t size() const;

protected:
    /**
     * @brief Empty default constructor, only accessible by child classes
     */
    base_FilterBankLayer();

    base_FilterBankLayer(const LayerConfig &cfg);

    /**
     * @brief Called by Compute for rectifying response from individual kernels
     * (e.g. square response). Default is to do nothing and leave response as is.
     * @param response modified in-place
     */
    virtual void Rectify(cv::Mat1f &response);

    std::string name_out_;  ///< desintation name for response in signal object

    elm::VecMat1f kernels_;  ///< individual kernels
    elm::VecMat1f response_; ///< response per kernel for most recent input
};

} // namespace elm

#endif // _ELM_ENCODING_BASE_FILTERBANKLAYER_H_
