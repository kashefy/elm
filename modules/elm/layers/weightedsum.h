/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_WEIGHTEDSUM_H_
#define _ELM_LAYERS_WEIGHTEDSUM_H_

#include "elm/layers/base_layer_derivations/base_featuretransformationlayer.h"

namespace elm {

/** class to implement base layer methods
  * Yields weighted sum of stimulus with 2 elements
  * Simple example on how to derive from base_FeatureTransformationLayer class
  *
  * Key to stimulus defined in parent.
  * Key to response defined in parent.
  *
  * Cached name of stimulus in signal defined in parent.
  * Cached name of repsonse in signal defined in parent.
  *
  */
class WeightedSum : public base_FeatureTransformationLayer
{
public:
    virtual void Clear();

    virtual void Reconfigure(const LayerConfig &config);

    virtual void Reset(const LayerConfig &config);

    virtual void Activate(const Signal &signal);

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

protected:
    float a_;               ///< factor 'a'
    float b_;               ///< factor 'b'
};

} // namespace elm

#endif // _ELM_LAYERS_WEIGHTEDSUM_H_
