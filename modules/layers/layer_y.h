#ifndef SEM_LAYERS_LAYERY_H_
#define SEM_LAYERS_LAYERY_H_

#include <string>

#include "core/base_Layer.h"
#include "neuron/neuron.h"

class LayerY : public base_Layer, protected YNeuron
{
public:
    static const std::string PARAM_FREQ;
    static const std::string PARAM_DELTA_T_MSEC;

    static const std::string KEY_INPUT_STIMULUS;
    static const std::string KEY_OUTPUT_SPIKES;

    LayerY();

    LayerY(const LayerConfig &config);

    virtual void Clear();

    virtual void Reset(const LayerConfig& config);

    virtual void Reconfigure(const LayerConfig& config);

    virtual void IONames(const LayerIONames& config);

    virtual void Activate(const Signal &signal);

    virtual void Response(Signal &signal);

protected:
    std::string name_stimulus_;
    std::string name_spikes_;

    int state_;
};

#endif // LAYERY_H
