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
     * @brief initialize object
     * @param number of features
     * @param spiking input history length
     */
    void init(int nb_features, int len_history);

    void Learn();

    cv::Mat Predict(const cv::Mat &evidence);

    /**
     * @brief Get neuron's weight vector, including bias term
     * @return neuron weights excluding bias term
     */
    MatF Weights() const;

    /**
     * @brief get bias term
     * @return neuron's bias term (single element)
     */
    MatF Bias() const;

protected:

    bool has_fired_;            ///< indicate whether neuron fired since last input
    float bias_;                 ///< bias term
    MatF weights_;              ///< Neuron weights, excluding bias term
    SpikingHistory history_;    ///< spiking input history
};

#endif // SEM_NEURON_ZNEURON_H_
