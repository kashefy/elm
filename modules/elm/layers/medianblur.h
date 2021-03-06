/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_MEDIANBLUR_H_
#define _ELM_LAYERS_MEDIANBLUR_H_

#include "elm/layers/layers_interim/base_smoothlayer.h"

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
class MedianBlur : public base_SmoothLayer
{
public:
    static const std::string PARAM_NAN_SURROGTATE;      ///< value used as nan replacement to reverse effect of nan values in response, must not occur in data

    static const float DEFAULT_NAN_SURROGATE;           ///< = 1e7

    MedianBlur();

    void Reconfigure(const LayerConfig &config);

    void Activate(const Signal &signal);

protected:
    // members
    float nan_surrogate_;
};

} // namespace elm

#endif // _ELM_LAYERS_MEDIANBLUR_H_
