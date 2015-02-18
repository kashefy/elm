/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_BASE_LAYER_DERIVS_BASE_SMOOTHLAYER_H_
#define _ELM_LAYERS_BASE_LAYER_DERIVS_BASE_SMOOTHLAYER_H_

#include "elm/layers/base_layer_derivations/base_featuretransformationlayer.h"

namespace elm {

/**
 * @brief intermediate Layer for applying smoothing algorithms
 * I/O keys already defined by parent class
 */
class base_SmoothLayer : public base_FeatureTransformationLayer
{
public:
    virtual ~base_SmoothLayer();

    static const std::string PARAM_APERTURE_SIZE;   ///< aperture linear size; it must be odd and greater than 1, for example: 3, 5, 7 ...

    virtual void Clear();

    void Reset(const LayerConfig &config);

    virtual void Reconfigure(const LayerConfig &config);

protected:
    base_SmoothLayer();

    base_SmoothLayer(const LayerConfig &config);

    // members
    int ksize_; ///< aperture/kernel size
};

} // namespace elm

#endif // _ELM_LAYERS_BASE_LAYER_DERIVS_BASE_SMOOTHLAYER_H_
