#ifndef SEM_NEURON_H_
#define SEM_NEURON_H_

#include "core/sampler.h"

/**
 * @brief Class for trivial spiking neuron.
 *
 * Does not learn.
 * Its spiking is controlled by its state and a poisson process
 *
 * TODO: cite SEM paper
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

private:
    PoissonProcess poisson_;

    cv::RNG rng_;           ///< random number generator to use in poisson process
};

#endif // SEM_NEURON_H_
