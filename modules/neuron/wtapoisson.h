#ifndef SEM_NEURON_WTAPOISSON_H_
#define SEM_NEURON_WTAPOISSON_H_

#include "neuron/competition.h"

/** WTA circuit allowing learner neurons to fire a given Poisson-rate
 * At the time of a spike, a WTA competition for firing
 * with soft-max is computed
 */
class WTAPoisson : public base_WTA
{
public:
    /**
     * @brief WTA circuit spiking at a Poisson rate
     * @param max_frequency maximum frequency in Hz
     * @param delta_t_msec time resolution in milliseconds
     */
    WTAPoisson(float max_frequency, float delta_t_msec);

    virtual cv::Mat Compete(std::vector<base_Learner> &learners);

    /**
     * @brief Compute next spike time for inhibiting neuron
     * @return next spike time in milliseconds
     */
    float NextSpikeTime() const;

private:

    float lambda_;  ///< Lambda variable for Poisson Rate
    float next_spike_time_; ///< timestamp for next spike event in msec
};

#endif // SEM_NEURON_WTAPOISSON_H_
