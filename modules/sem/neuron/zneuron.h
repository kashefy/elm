#ifndef SEM_NEURON_ZNEURON_H_
#define SEM_NEURON_ZNEURON_H_

#include "sem/neuron/base_learner.h"
#include "sem/neuron/spikinghistory.h"

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
    void Init(int nb_features, int len_history);

    void Learn(const cv::Mat &target);

    /**
     * @brief Predict
     * @param evidence
     * @return membrane potential
     */
    cv::Mat Predict(const cv::Mat &evidence);

    /**
     * @brief Z Neuron state State
     * @return membrane potential
     */
    cv::Mat State() const;

    /**
     * @brief Get neuron's weight vector, including bias term
     * Involves deep copy
     * @return neuron weights excluding bias term, log scale
     */
    cv::Mat1f Weights() const;

    /**
     * @brief get bias term
     * @return neuron's bias term (single element) log scale
     */
    cv::Mat1f Bias() const;

    /**
     * @brief Clear spiking history
     */
    void Clear();

protected:
    /**
     * @brief Update of weights according to afferent spiking activity using STDP
     *
     * This implements the code learning algorithm from @cite Nessler2010
     *
     * @param weights update in-place
     * @param recent spiking history (binary mask)
     */
    void Update(cv::Mat &weights, const cv::Mat &has_spiked_recently) const;

    cv::Mat1f weights_all_;     ///< Neuron weights, including bias term, log scale
    cv::Mat1f bias_;            ///< bias term
    cv::Mat1f weights_;         ///< Neuron weights, excluding bias term, log scale, effectively a colRange of weights_all_ member

    SpikingHistory history_all_;    ///< spiking input history, excluding bias
    SpikingHistory history_afferents_;    ///< spiking input history, excluding bias
    SpikingHistory history_self_;    ///< spiking input history, excluding bias

    float u_;                   ///< membrane potential
};

#endif // SEM_NEURON_ZNEURON_H_
