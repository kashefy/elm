#ifndef SEM_NEURON_ZNEURON_H_
#define SEM_NEURON_ZNEURON_H_

#include "neuron/base_learner.h"
#include "neuron/spikinghistory.h"

/**
 * @brief Integrate and fire neuron
 */
class ZNeuron : public base_Learner
{
public:
    ZNeuron();

    /**
     * @brief initialize neuron with random weights and bias
     *
     * Values are drawn from a half-normal distribution, where x < 0
     * then scaled by 0.01
     *
     * @param number of features
     * @param spiking input history length
     */
    void init(int nb_features, int len_history);

    void Learn(const cv::Mat &target);

    cv::Mat Predict(const cv::Mat &evidence);

    /**
     * @brief Get neuron's weight vector, including bias term
     * @return neuron weights excluding bias term, log scale
     */
    MatF Weights() const;

    /**
     * @brief get bias term
     * @return neuron's bias term (single element) log scale
     */
    MatF Bias() const;

protected:

    float bias_;                ///< bias term
    MatF weights_;              ///< Neuron weights, excluding bias term, log scale
    SpikingHistory history_;    ///< spiking input history
};

#endif // SEM_NEURON_ZNEURON_H_
