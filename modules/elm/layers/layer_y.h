/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_LAYERS_LAYERY_H_
#define _ELM_LAYERS_LAYERY_H_

#include "elm/layers/base_layer_derivations/base_featuretransformationlayer.h"
#include "elm/neuron/neuron.h"

namespace elm {

/**
 * @brief The LayerY class
 */
class LayerY :
        public base_FeatureTransformationLayer,
        protected YNeuron
{
public:
    static const std::string PARAM_FREQ;            ///< poisson frequnecy, spiking frequency [Hz]
    static const std::string PARAM_DELTA_T_MSEC;    ///< time resolution

    LayerY();

    LayerY(const LayerConfig &config);

    virtual void Clear();

    virtual void Reset(const LayerConfig& config);

    virtual void Reconfigure(const LayerConfig& config);

    virtual void Activate(const Signal &signal);

protected:
    /**
     * @brief Vectorized version of YNeuron::State()
     * @param new states
     * @return spike events (0 == no spike)
     */
    cv::Mat1i State(cv::Mat1f states);
};

} // namespace elm

#endif // LAYERY_H
