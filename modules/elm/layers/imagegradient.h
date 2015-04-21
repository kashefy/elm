/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_IMAGEGRADIENT_H_
#define _ELM_LAYERS_IMAGEGRADIENT_H_

#include "elm/layers/layers_interim/base_featuretransformationlayer.h"

namespace elm {

/**
 * @brief Layer for computing image gradients using forward difference
 */
class ImageGradient : public base_FeatureTransformationLayer
{
public:
    ImageGradient();

    /** Constructor with configuration
      * @param layer configuration
      */
    ImageGradient(const LayerConfig& cfg);

    void Clear();

    void Reconfigure(const LayerConfig &config);

    void Reset(const LayerConfig &config);

    void Activate(const Signal &signal);
};

} // namespace elm

#endif // _ELM_LAYERS_IMAGEGRADIENT_H_
