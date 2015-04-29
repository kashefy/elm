/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_NEURON_H_
#define _ELM_NEURON_H_

#include "elm/core/sampler.h"

namespace elm {

/**
 * @brief layer for trivial spiking neuron.
 *
 * Does not learn.
 * Its spiking is controlled by its state and a poisson process
 *
 * @cite Nessler2010
 */
class YNeuron
{
public:
    YNeuron();

    /**
     * @brief initialize Neuron
     * @param frequency: maximum firing rate, poisson frequency
     * @param delta_t_msec time resolution between spike events
     */
    void init(float frequency, float delta_t_msec);

    /**
     * @brief Set Neuron state and determine if it will spike
     * @param new state
     * @return spike event 0 - no spike
     */
    int State(float state);

protected:
    PoissonProcess poisson_;
};

} // namespace elm

#endif // _ELM_NEURON_H_
