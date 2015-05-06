/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERS_INTERIM_BASE_MATOUTPUTLAYER_H
#define _ELM_LAYERS_LAYERS_INTERIM_BASE_MATOUTPUTLAYER_H

#include <string>

#include <opencv2/core/core.hpp>

#include "elm/core/base_Layer.h"

namespace elm {

class Signal;

namespace detail {

const std::string BASE_MATOUTPUT_LAYER__KEY_OUTPUT_RESPONSE = "m";   ///< define string here to ensure early initialization for test purposes

}

/**
 * @brief class for time-invariant layer
 * @todo generalize to FeatureOutputLayer, or maybe keep this (compiles faster)
 */
class base_MatOutputLayer : virtual public base_Layer
{
public:
    static const std::string KEY_OUTPUT_RESPONSE;

    virtual ~base_MatOutputLayer();

    virtual void OutputNames(const LayerOutputNames &io);

    virtual void Response(Signal &signal);

protected:
    base_MatOutputLayer();

    // members
    std::string name_output_;    ///< destination name in signal object

    cv::Mat1f m_;   ///< most recent response/output
};

} // namespace elm

#endif // _ELM_LAYERS_LAYERS_INTERIM_BASE_MATOUTPUTLAYER_H
