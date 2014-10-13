#ifndef SEM_NEURON_SPIKINGHISTORY_H_
#define SEM_NEURON_SPIKINGHISTORY_H_

#include <opencv2/core.hpp>

/**
 * @brief Class for tracking spiking history
 */
class SpikingHistory
{
public:
    /**
     * @brief Constructor
     * @param no. of input dimensions to track
     * @param length of history to keep
     */
    SpikingHistory(int dims, int len);

    /**
     * @brief Advance history
     */
    virtual void Advance();

    /**
     * @brief Get History
     * @return history
     */
    virtual cv::Mat1i History() const;

    /**
     * @brief Reset history so far
     */
    virtual void Reset();

    /**
     * @brief Check if input at given index has fired recently
     * @param index
     * @return true on recent firing
     */
    virtual bool Recent(int index) const;

    /**
     * @brief Obtain vector indicating which indice recently fired
     * @return mask indices that recently fired
     */
    virtual cv::Mat Recent() const;

    /**
     * @brief update history with new set of spiking input
     * @param spiking input mask vector (spike vector != 0)
     */
    virtual void Update(const cv::Mat &spike_mask);

protected:

    int len_;        ///< history length
    int dims_;       ///< no. of input dimensions to track
    cv::Mat1i history_;  ///< input history
};

#endif // SEM_NEURON_SPIKINGHISTORY_H_
