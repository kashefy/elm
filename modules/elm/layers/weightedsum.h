/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef ELM_LAYERS_WEIGHTEDSUM_H_
#define ELM_LAYERS_WEIGHTEDSUM_H_

#include <string>

#include <opencv2/core.hpp>

#include "elm/core/base_Layer.h"

namespace elm {

/** class to implement base layer methods
  * Yields weighted sum of stimulus with 2 elements
  * Simple example on how to derive from base Layer class
  */
class WeightedSum : public base_Layer
{
public:
    virtual void Clear();

    virtual void Reconfigure(const LayerConfig &config);

    virtual void Reset(const LayerConfig &config);

    virtual void IONames(const LayerIONames &config);

    virtual void Activate(const Signal &signal);

    virtual void Response(Signal &signal);

    /** Default constructor, still requires configurations
      * \see Reconfigure
      */
    WeightedSum();

    /** Constructor with configuration
      * @param layer configuration
      */
    WeightedSum(const LayerConfig& config);

public:
    static const std::string PARAM_A;    ///< key to factor 'a' in params structure
    static const std::string PARAM_B;    ///< key to factor 'b' in params structure

    static const std::string KEY_INPUT_STIMULUS;     ///< key to stimulus
    static const std::string KEY_OUTPUT_RESPONSE;    ///< key to response

protected:
    float a_;               ///< factor 'a'
    float b_;               ///< facotr 'b'

    std::string name_stimulus_;  ///< cached name of stimulus in signal
    std::string name_response_;  ///< cahced name of repsonse in signal

    cv::Mat1f response_;        ///< most recent response
};

} // namespace elm

#endif // ELM_LAYERS_WEIGHTEDSUM_H_
