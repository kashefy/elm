/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_MEDIANBLUR_H_
#define _ELM_LAYERS_MEDIANBLUR_H_

#include "elm/layers/base_layer_derivations/base_featuretransformationlayer.h"

namespace elm {

/**
 * @brief Wrap layer around median blur
 * input and output keys defined by parent
 *
 * from OpneCV's docs:
 * when ksize is 3 or 5,
 * the image depth should be CV_8U, CV_16U, or CV_32F,
 * for larger aperture sizes, it can only be CV_8U
 *
 **/
class MedianBlur : public base_FeatureTransformationLayer
{
public:
    static const std::string PARAM_APERTURE_SIZE;   ///< aperture linear size; it must be odd and greater than 1, for example: 3, 5, 7 ...

    MedianBlur();

    MedianBlur(const LayerConfig config);

    void Clear();

    void Reset(const LayerConfig &config);

    void Reconfigure(const LayerConfig &config);

    void Activate(const Signal &signal);

protected:
    // members
    int ksize_; ///< aperture linear size
};

} // namespace elm

#endif // _ELM_LAYERS_MEDIANBLUR_H_
