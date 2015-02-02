/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_NEURON_SPIKINGHISTORY_H_
#define _ELM_NEURON_SPIKINGHISTORY_H_

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
     * @brief Reset history at a given index
     */
    virtual void Reset(int index);

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

    /**
     * @brief Obtain reference of a spiking history with a column range of underlying histry
     * @param start index (inclusive)
     * @param end index (exclusive)
     * @return reference to column range of history data
     * @throws cv::Exception for invalid column ranges (e.g. out of bounds, negative indcies), should we throw our own Exceptions?
     */
    virtual SpikingHistory ColRange(int start, int end) const;

protected:
    /**
     * @brief Directly overwrite underlying history data
     * @param new history data
     */
    virtual void History(const cv::Mat1i& h);

    int len_;        ///< history length
    int dims_;       ///< no. of input dimensions to track
    cv::Mat1i history_;  ///< input history
};

#endif // _ELM_NEURON_SPIKINGHISTORY_H_
