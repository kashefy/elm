#ifndef SEM_NEURON_SPIKINGHISTORY_H_
#define SEM_NEURON_SPIKINGHISTORY_H_

#include "core/typedefs.h"

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
    virtual MatI History() const;

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
     * @brief update history with new set of spiking input
     * @param spiking input mask vector (spike vector != 0)
     */
    virtual void Update(const cv::Mat &spike_mask);

protected:

    int len_;        ///< history length
    int dims_;       ///< no. of input dimensions to track
    MatI history_;  ///< input history
};

#endif // SEM_NEURON_SPIKINGHISTORY_H_
