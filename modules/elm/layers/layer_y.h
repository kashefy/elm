/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef ELM_LAYERS_LAYERY_H_
#define ELM_LAYERS_LAYERY_H_

#include <string>

#include "elm/core/base_Layer.h"
#include "elm/neuron/neuron.h"

/**
 * @brief The LayerY class
 */
class LayerY : public base_Layer, protected YNeuron
{
public:
    static const std::string PARAM_FREQ;            ///< poisson frequnecy, spiking frequency [Hz]
    static const std::string PARAM_DELTA_T_MSEC;    ///< time resolution

    static const std::string KEY_INPUT_STIMULUS;    ///< key to stimulus in signal object
    static const std::string KEY_OUTPUT_SPIKES;     ///< key to output spikes in signal object

    LayerY();

    LayerY(const LayerConfig &config);

    virtual void Clear();

    virtual void Reset(const LayerConfig& config);

    virtual void Reconfigure(const LayerConfig& config);

    virtual void IONames(const LayerIONames& config);

    virtual void Activate(const Signal &signal);

    virtual void Response(Signal &signal);

protected:
    /**
     * @brief Vectorized version of YNeuron::State()
     * @param new states
     * @return spike events (0 == no spike)
     */
    cv::Mat1i State(cv::Mat1f states);

    std::string name_stimulus_; ///< name of input spikes in signal object
    std::string name_spikes_;   ///< destination of output spikes in signal object

    cv::Mat1i state_; ///< neuron state from which we determin spiking activity
};

#endif // LAYERY_H
